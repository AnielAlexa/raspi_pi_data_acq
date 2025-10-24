// Raspberry Pi Pico + BMI088 → Precise 400Hz using Hardware Timer
// UPDATED: Added camera trigger timestamp packets for ROS2 synchronization
// Sends TWO packet types over Serial:
//   1. IMU packets @ 400Hz (header 0xAA55, 32 bytes)
//   2. Trigger packets @ 20Hz (header 0xBB66, 12 bytes)

#include <Arduino.h>
#include <SPI.h>
#include <BMI088.h>

// --- Pin Definitions ---
constexpr uint8_t PIN_SPI_SCK = 18;
constexpr uint8_t PIN_SPI_MOSI = 19;
constexpr uint8_t PIN_SPI_MISO = 16;
constexpr uint8_t PIN_CS_ACCEL = 17;
constexpr uint8_t PIN_CS_GYRO = 20;
constexpr uint8_t PIN_CAMERA_TRIGGER = 22;  // Camera trigger output

// Camera trigger settings
constexpr uint32_t CAMERA_TRIGGER_PULSE_US = 100;  // Pulse width in microseconds

// BMI088 object
Bmi088 bmi(SPI, PIN_CS_ACCEL, PIN_CS_GYRO);

// IMU Packet structure (32 bytes)
struct __attribute__((packed)) ImuPacket {
  uint16_t header;        // 0xAA55
  uint64_t timestamp_us;  // Microseconds
  float ax, ay, az;       // m/s²
  float gx, gy, gz;       // rad/s
  uint16_t crc16;         // CRC checksum
};

// NEW: Camera Trigger Packet structure (12 bytes)
struct __attribute__((packed)) TriggerPacket {
  uint16_t header;        // 0xBB66 (different from IMU)
  uint64_t timestamp_us;  // Microseconds when trigger fired
  uint16_t frame_id;      // Frame counter
  uint16_t reserved;      // Reserved for alignment
  uint16_t crc16;         // CRC checksum
};

// Double buffering for IMU
volatile ImuPacket imu_buffer[2];
volatile uint8_t imu_write_idx = 0;
volatile uint8_t imu_read_idx = 0;

// NEW: Double buffering for triggers
volatile TriggerPacket trigger_buffer[2];
volatile uint8_t trigger_write_idx = 0;
volatile uint8_t trigger_read_idx = 0;

// Flags for synchronization
volatile bool sample_ready = false;        // Timer says: time to read IMU
volatile bool imu_packet_ready = false;    // IMU packet filled and ready to send
volatile bool trigger_packet_ready = false; // NEW: Trigger packet ready to send
volatile uint32_t sample_timestamp = 0;    // Timestamp captured in ISR
volatile uint32_t trigger_timestamp = 0;   // NEW: Trigger timestamp captured in ISR
volatile bool camera_trigger_flag = false; // Camera trigger flag

// Statistics
volatile uint32_t timer_ticks = 0;
volatile uint32_t missed_samples = 0;
volatile uint32_t camera_triggers = 0;
volatile uint16_t frame_counter = 0;  // NEW: Global frame counter

// --- CRC16 Calculation ---
uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// --- Timer Callback (FAST - just sets flags!) ---
bool timer_callback(struct repeating_timer *t) {
  // Check if previous sample was processed
  if (sample_ready) {
    missed_samples++;  // Main loop didn't read fast enough
  }

  // Capture timestamp and signal main loop
  sample_timestamp = micros();
  sample_ready = true;
  timer_ticks++;

  // Camera trigger logic (400Hz / 20 = 20Hz)
  static uint8_t camera_divider = 0;
  camera_divider++;
  if (camera_divider >= 20) {
    camera_divider = 0;
    camera_trigger_flag = true;
    camera_triggers++;
    trigger_timestamp = sample_timestamp;  // NEW: Capture trigger timestamp
    frame_counter++;  // NEW: Increment frame counter

    // Generate trigger pulse (HIGH)
    digitalWrite(PIN_CAMERA_TRIGGER, HIGH);
  }

  return true;  // Continue timer
}

// --- Setup ---
void setup() {
  Serial.begin(921600);

  // LED for status
  pinMode(LED_BUILTIN, OUTPUT);

  // Camera trigger pin
  pinMode(PIN_CAMERA_TRIGGER, OUTPUT);
  digitalWrite(PIN_CAMERA_TRIGGER, LOW);

  // Configure SPI pins
  pinMode(PIN_CS_ACCEL, OUTPUT);
  pinMode(PIN_CS_GYRO, OUTPUT);
  digitalWrite(PIN_CS_ACCEL, HIGH);
  digitalWrite(PIN_CS_GYRO, HIGH);

  // Initialize SPI
  SPI.setRX(PIN_SPI_MISO);
  SPI.setTX(PIN_SPI_MOSI);
  SPI.setSCK(PIN_SPI_SCK);
  SPI.begin();

  // Wait briefly for serial
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) {}

  // Initialize BMI088
  int status = bmi.begin();
  if (status <= 0) {
    // Error: fast blink forever
    Serial.println("ERROR: BMI088 init failed!");
    while(1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }

  // Configure sensor: 400Hz, ±6g, ±500dps
  bmi.setOdr(Bmi088::ODR_400HZ);
  bmi.setRange(Bmi088::ACCEL_RANGE_6G, Bmi088::GYRO_RANGE_500DPS);

  // Small delay for sensor to stabilize
  delay(100);

  // Setup hardware timer for EXACT 400Hz (2500µs period)
  static struct repeating_timer timer;
  bool timer_ok = add_repeating_timer_us(
    -2500,  // Negative = measure from end of last callback
    timer_callback,
    NULL,
    &timer
  );

  if (!timer_ok) {
    Serial.println("ERROR: Timer setup failed!");
    while(1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(50);
    }
  }

  // Ready
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("# BMI088 + Camera Trigger Ready - 400Hz IMU + 20Hz Trigger Packets");
}

// --- Main Loop (handles SPI and packet sending) ---
void loop() {
  static uint32_t last_stats = 0;
  static uint32_t imu_packets_sent = 0;
  static uint32_t trigger_packets_sent = 0;
  static uint32_t camera_pulse_start = 0;

  // Handle camera trigger pulse (end the pulse after specified duration)
  if (camera_trigger_flag) {
    camera_pulse_start = micros();

    // NEW: Create trigger packet immediately when trigger fires
    noInterrupts();
    uint8_t next_write = (trigger_write_idx + 1) % 2;
    TriggerPacket* tpkt = (TriggerPacket*)&trigger_buffer[next_write];

    tpkt->header = 0xBB66;  // Different header for trigger packets
    tpkt->timestamp_us = trigger_timestamp;
    tpkt->frame_id = frame_counter;
    tpkt->reserved = 0;
    tpkt->crc16 = crc16((uint8_t*)tpkt, sizeof(TriggerPacket) - 2);

    trigger_write_idx = next_write;
    trigger_packet_ready = true;
    camera_trigger_flag = false;
    interrupts();
  }

  // End camera pulse after pulse width
  if (camera_pulse_start > 0 && (micros() - camera_pulse_start) >= CAMERA_TRIGGER_PULSE_US) {
    digitalWrite(PIN_CAMERA_TRIGGER, LOW);
    camera_pulse_start = 0;
  }

  // Check if timer says it's time to sample IMU
  if (sample_ready) {
    // Clear flag immediately
    noInterrupts();
    bool ready = sample_ready;
    sample_ready = false;
    uint32_t timestamp = sample_timestamp;
    interrupts();

    if (ready) {
      // Read sensor (SPI transaction - safe in main loop!)
      bmi.readSensor();

      // Get next write buffer
      uint8_t next_write = (imu_write_idx + 1) % 2;

      // Fill IMU packet
      ImuPacket* pkt = (ImuPacket*)&imu_buffer[next_write];
      pkt->header = 0xAA55;
      pkt->timestamp_us = timestamp;
      pkt->ax = bmi.getAccelX_mss();
      pkt->ay = bmi.getAccelY_mss();
      pkt->az = bmi.getAccelZ_mss();
      pkt->gx = bmi.getGyroX_rads();
      pkt->gy = bmi.getGyroY_rads();
      pkt->gz = bmi.getGyroZ_rads();
      pkt->crc16 = crc16((uint8_t*)pkt, sizeof(ImuPacket) - 2);

      // Atomically swap buffers and signal packet ready
      noInterrupts();
      imu_write_idx = next_write;
      imu_packet_ready = true;
      interrupts();
    }
  }

  // Send IMU packet if ready
  if (imu_packet_ready) {
    // Atomically get buffer to send
    noInterrupts();
    uint8_t idx = imu_read_idx;
    imu_read_idx = imu_write_idx;
    imu_packet_ready = false;
    interrupts();

    // Send IMU packet
    Serial.write((uint8_t*)&imu_buffer[idx], sizeof(ImuPacket));
    imu_packets_sent++;
  }

  // NEW: Send trigger packet if ready
  if (trigger_packet_ready) {
    // Atomically get buffer to send
    noInterrupts();
    uint8_t idx = trigger_read_idx;
    trigger_read_idx = trigger_write_idx;
    trigger_packet_ready = false;
    interrupts();

    // Send trigger packet
    Serial.write((uint8_t*)&trigger_buffer[idx], sizeof(TriggerPacket));
    trigger_packets_sent++;
  }

  // Periodic status (every 1 second)
  uint32_t now = millis();
  if (now - last_stats >= 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // Debug stats (comment out for production)
    // Uncomment these lines to see statistics on Serial Monitor:
    /*
    Serial.print("# Timer: ");
    Serial.print(timer_ticks);
    Serial.print(" Hz, IMU: ");
    Serial.print(imu_packets_sent);
    Serial.print(", Trigger: ");
    Serial.print(trigger_packets_sent);
    Serial.print(" Hz, Frames: ");
    Serial.print(frame_counter);
    Serial.print(", Missed: ");
    Serial.println(missed_samples);
    */

    imu_packets_sent = 0;
    trigger_packets_sent = 0;
    timer_ticks = 0;
    camera_triggers = 0;
    missed_samples = 0;
    last_stats = now;
  }

  // No delays - loop as fast as possible
}
