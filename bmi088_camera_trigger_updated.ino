// Raspberry Pi Pico + BMI088 + BMP388 → Synchronized Sensor Head
// IMU @400 Hz, Camera Trigger @20 Hz, Altimeter @10 Hz
// All data streamed over UART Serial1
// Altitude = 0.000 m at first live altimeter sample after startup

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <BMI088.h>
#include <Adafruit_BMP3XX.h>

// ---------------------- Pin definitions ----------------------
constexpr uint8_t PIN_SPI_SCK = 18;
constexpr uint8_t PIN_SPI_MOSI = 19;
constexpr uint8_t PIN_SPI_MISO = 16;
constexpr uint8_t PIN_CS_ACCEL = 17;
constexpr uint8_t PIN_CS_GYRO = 20;

constexpr uint8_t PIN_CAMERA_TRIGGER = 22;

constexpr uint8_t PIN_I2C_SDA = 4;
constexpr uint8_t PIN_I2C_SCL = 5;

constexpr uint32_t CAMERA_TRIGGER_PULSE_US = 100;
constexpr uint32_t UART_BAUD = 230400;

constexpr float SEALEVEL_HPA = 1013.25f;

// ---------------------- Sensor objects ----------------------
Bmi088 bmi(SPI, PIN_CS_ACCEL, PIN_CS_GYRO);
Adafruit_BMP3XX bmp388;

// ---------------------- Packet structs ----------------------
struct __attribute__((packed)) ImuPacket {
  uint16_t header;  // 0xAA55
  uint64_t timestamp_us;
  float ax, ay, az;
  float gx, gy, gz;
  uint16_t crc16;
};

struct __attribute__((packed)) TriggerPacket {
  uint16_t header;  // 0xBB66
  uint64_t timestamp_us;
  uint16_t frame_id;
  uint16_t reserved;
  uint16_t crc16;
};

struct __attribute__((packed)) AltimeterPacket {
  uint16_t header;  // 0xCC77
  uint64_t timestamp_us;
  float altitude_m;  // relative altitude
  uint16_t crc16;
};

// ---------------------- Buffers / shared state ----------------------
volatile ImuPacket imu_buffer[2];
volatile uint8_t imu_write_idx = 0;
volatile uint8_t imu_read_idx = 0;
volatile bool imu_packet_ready = false;

volatile TriggerPacket trigger_buffer[2];
volatile uint8_t trigger_write_idx = 0;
volatile uint8_t trigger_read_idx = 0;
volatile bool trigger_packet_ready = false;

volatile AltimeterPacket alt_buffer[2];
volatile uint8_t alt_write_idx = 0;
volatile uint8_t alt_read_idx = 0;
volatile bool alt_packet_ready = false;

// Queue-based IMU sampling to prevent frame loss
// Allow up to 8 pending IMU samples (enough to cover altimeter I2C blocking)
constexpr uint8_t IMU_QUEUE_SIZE = 8;
volatile uint32_t imu_timestamp_queue[IMU_QUEUE_SIZE];
volatile uint8_t imu_queue_head = 0;       // Write position (ISR)
volatile uint8_t imu_queue_tail = 0;       // Read position (loop)
volatile uint8_t imu_samples_dropped = 0;  // Counter for overflow detection

volatile bool camera_trigger_flag = false;
volatile bool altimeter_sample_flag = false;

volatile uint32_t trigger_timestamp = 0;
volatile uint32_t altimeter_timestamp = 0;

volatile uint16_t frame_counter = 0;

// altitude baseline state
float altitude_baseline_m = 0.0f;
// baseline_ready = we have at least some idea (after setup())
bool baseline_ready = false;
// baseline_finalized = we locked the "true zero" in loop()
bool baseline_finalized = false;

// ---------------------- CRC16 ----------------------
uint16_t crc16_ibm(const uint8_t* data, size_t len) {
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

// ---------------------- Timer ISR @400Hz ----------------------
bool timer_callback(struct repeating_timer* /*t*/) {
  uint32_t now_us = micros();

  // 400 Hz IMU sampling - queue-based to prevent frame loss
  uint8_t next_head = (imu_queue_head + 1) % IMU_QUEUE_SIZE;
  if (next_head != imu_queue_tail) {
    // Queue has space - add timestamp
    imu_timestamp_queue[imu_queue_head] = now_us;
    imu_queue_head = next_head;
  } else {
    // Queue full - count dropped sample (should never happen with size 8)
    imu_samples_dropped++;
  }

  // 20 Hz camera trigger
  static uint8_t camera_divider = 0;
  camera_divider++;
  if (camera_divider >= 20) {
    camera_divider = 0;
    camera_trigger_flag = true;
    trigger_timestamp = now_us;
    frame_counter++;
    digitalWrite(PIN_CAMERA_TRIGGER, HIGH);
  }

  // 10 Hz altimeter
  static uint8_t alt_divider = 0;
  alt_divider++;
  if (alt_divider >= 40) {
    alt_divider = 0;
    altimeter_sample_flag = true;
    altimeter_timestamp = now_us;
  }

  return true;
}

// ---------------------- Baseline calibration (warm-up) ----------------------
// We average a few readings in setup() just to get something reasonable.
// This does NOT lock the final zero. Final zero happens in loop() on first
// live altimeter sample.
bool warmup_altitude_baseline() {
  const uint8_t N = 10;
  float sum_alt = 0.0f;
  uint8_t good = 0;

  uint32_t start_ms = millis();
  while (good < N && (millis() - start_ms) < 5000) {
    if (bmp388.performReading()) {
      float alt_now = bmp388.readAltitude(SEALEVEL_HPA);
      sum_alt += alt_now;
      good++;
    }
    delay(100);  // ~10 Hz samples during warmup
  }

  if (good == 0) {
    return false;
  }

  altitude_baseline_m = sum_alt / (float)good;
  baseline_ready = true;
  baseline_finalized = false;
  return true;
}

// ---------------------- setup() ----------------------
void setup() {
  Serial1.begin(UART_BAUD);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(PIN_CAMERA_TRIGGER, OUTPUT);
  digitalWrite(PIN_CAMERA_TRIGGER, LOW);

  pinMode(PIN_CS_ACCEL, OUTPUT);
  pinMode(PIN_CS_GYRO, OUTPUT);
  digitalWrite(PIN_CS_ACCEL, HIGH);
  digitalWrite(PIN_CS_GYRO, HIGH);

  SPI.setRX(PIN_SPI_MISO);
  SPI.setTX(PIN_SPI_MOSI);
  SPI.setSCK(PIN_SPI_SCK);
  SPI.begin();

  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.begin();

  if (bmi.begin() <= 0) {
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }

  bmi.setOdr(Bmi088::ODR_400HZ);
  bmi.setRange(Bmi088::ACCEL_RANGE_6G, Bmi088::GYRO_RANGE_500DPS);
  delay(100);

  if (!bmp388.begin_I2C()) {
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }

  // Reduced oversampling to minimize I2C blocking time
  // This prevents IMU frame loss during altimeter reads
  bmp388.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);  // Was 8X
  bmp388.setPressureOversampling(BMP3_OVERSAMPLING_8X);     // Was 32X (4× faster!)
  bmp388.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp388.setOutputDataRate(BMP3_ODR_12_5_HZ);  // Was 50 Hz, now 12.5 Hz (slightly above our 10 Hz read rate)

  if (!warmup_altitude_baseline()) {
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }

  static struct repeating_timer timer;
  add_repeating_timer_us(-2500, timer_callback, NULL, &timer);

  digitalWrite(LED_BUILTIN, HIGH);
}

// ---------------------- loop() ----------------------
void loop() {
  static uint32_t camera_pulse_start_us = 0;

  // --- Handle camera trigger pulse ---
  if (camera_trigger_flag) {
    noInterrupts();
    camera_trigger_flag = false;
    uint32_t trig_ts = trigger_timestamp;
    uint16_t frame_id_local = frame_counter;
    interrupts();

    camera_pulse_start_us = micros();

    // Build trigger packet (CRC OUTSIDE critical section to avoid blocking ISR)
    uint8_t next_t = (trigger_write_idx + 1) & 0x01;
    TriggerPacket* tpkt = (TriggerPacket*)&trigger_buffer[next_t];
    tpkt->header = 0xBB66;
    tpkt->timestamp_us = (uint64_t)trig_ts;
    tpkt->frame_id = frame_id_local;
    tpkt->reserved = 0;
    tpkt->crc16 = crc16_ibm((uint8_t*)tpkt, sizeof(TriggerPacket) - 2);  // CRC calc with interrupts ENABLED

    // Only update index with interrupts disabled
    noInterrupts();
    trigger_write_idx = next_t;
    trigger_packet_ready = true;
    interrupts();
  }

  if (camera_pulse_start_us != 0) {
    uint32_t dt = micros() - camera_pulse_start_us;
    if (dt >= CAMERA_TRIGGER_PULSE_US) {
      digitalWrite(PIN_CAMERA_TRIGGER, LOW);
      camera_pulse_start_us = 0;
    }
  }

  // --- Handle IMU samples @400 Hz (queue-based, no frame loss) ---
  // Process ONE IMU sample per loop iteration to avoid boolean flag race condition
  if (imu_queue_tail != imu_queue_head) {
    // Get timestamp from queue
    uint32_t imu_ts;
    bool have_sample = false;

    // Atomically pop one timestamp from the queue
    noInterrupts();
    if (imu_queue_tail != imu_queue_head) {
      imu_ts = imu_timestamp_queue[imu_queue_tail];
      imu_queue_tail = (imu_queue_tail + 1) % IMU_QUEUE_SIZE;
      have_sample = true;
    }
    interrupts();

    if (have_sample) {
      // Read IMU now (SPI)
      bmi.readSensor();

      // Build and send packet immediately
      ImuPacket pkt;
      pkt.header = 0xAA55;
      pkt.timestamp_us = (uint64_t)imu_ts;
      pkt.ax = bmi.getAccelX_mss();
      pkt.ay = bmi.getAccelY_mss();
      pkt.az = bmi.getAccelZ_mss();
      pkt.gx = bmi.getGyroX_rads();
      pkt.gy = bmi.getGyroY_rads();
      pkt.gz = bmi.getGyroZ_rads();
      pkt.crc16 = crc16_ibm((uint8_t*)&pkt, sizeof(ImuPacket) - 2);

      Serial1.write((uint8_t*)&pkt, sizeof(ImuPacket));
    }
  }

  // --- Handle altimeter sample @10 Hz ---
  if (altimeter_sample_flag && baseline_ready) {
    noInterrupts();
    altimeter_sample_flag = false;
    uint32_t alt_ts = altimeter_timestamp;
    interrupts();

    if (bmp388.performReading()) {
      float raw_alt_m = bmp388.readAltitude(SEALEVEL_HPA);

      // Finalize baseline on FIRST live reading in loop()
      if (!baseline_finalized) {
        altitude_baseline_m = raw_alt_m;
        baseline_finalized = true;
      }

      float rel_alt_m = raw_alt_m - altitude_baseline_m;

      uint8_t next_a = (alt_write_idx + 1) & 0x01;
      AltimeterPacket* apkt = (AltimeterPacket*)&alt_buffer[next_a];
      apkt->header = 0xCC77;
      apkt->timestamp_us = (uint64_t)alt_ts;
      apkt->altitude_m = rel_alt_m;
      apkt->crc16 = crc16_ibm((uint8_t*)apkt, sizeof(AltimeterPacket) - 2);

      noInterrupts();
      alt_write_idx = next_a;
      alt_packet_ready = true;
      interrupts();
    }
  }

  // --- UART transmit packets ---
  // (IMU is transmitted immediately above to avoid race condition)

  if (trigger_packet_ready) {
    noInterrupts();
    uint8_t idx = trigger_read_idx;
    trigger_read_idx = trigger_write_idx;
    trigger_packet_ready = false;
    interrupts();

    Serial1.write((uint8_t*)&trigger_buffer[idx], sizeof(TriggerPacket));
  }

  if (alt_packet_ready) {
    noInterrupts();
    uint8_t idx = alt_read_idx;
    alt_read_idx = alt_write_idx;
    alt_packet_ready = false;
    interrupts();

    Serial1.write((uint8_t*)&alt_buffer[idx], sizeof(AltimeterPacket));
  }

  // no delay()
}
