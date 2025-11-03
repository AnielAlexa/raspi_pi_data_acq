// Raspberry Pi Pico + BMI088 + BMP388 → Synchronized Sensor Head
// Hardware interrupt-driven: IMU @400 Hz, Camera @20 Hz, Altimeter @10 Hz
// Binary packets over UART Serial1

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <BMI088.h>
#include <BMP388_DEV.h>

// SPI0 pins (BMI088)
constexpr uint8_t PIN_SPI_SCK = 18;
constexpr uint8_t PIN_SPI_MOSI = 19;
constexpr uint8_t PIN_SPI_MISO = 16;
constexpr uint8_t PIN_CS_ACCEL = 17;
constexpr uint8_t PIN_CS_GYRO = 20;

// I2C0 pins (BMP388)
constexpr uint8_t PIN_BMP_SDA = 4;  // adjust to your wiring (RP2040 default SDA0)
constexpr uint8_t PIN_BMP_SCL = 5;  // adjust to your wiring (RP2040 default SCL0)

// Interrupt pins
constexpr uint8_t PIN_IMU_INT = 21;  // BMI088 INT1 connected here
constexpr uint8_t PIN_BMP_INT = 14;

// Camera trigger
constexpr uint8_t PIN_CAMERA_TRIGGER = 22;
constexpr uint32_t CAMERA_TRIGGER_PULSE_US = 100;

constexpr float SEALEVEL_HPA = 1013.25f;

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
  float altitude_m;
  uint16_t crc16;
};

// ---------------------- Global variables ----------------------
volatile bool imuDataReady = false;
volatile bool bmpDataReady = false;


// BMP388 data storage
float bmpTemperature, bmpPressure, bmpAltitude;
uint32_t lastCameraTrigger = 0;
uint32_t cameraPulseStartUs = 0;
volatile uint16_t frameCounter = 0;

float altitudeBaselineM = 0.0f;
bool baselineReady = false;

// ---------------------- Sensor objects ----------------------
Bmi088 bmi(SPI, PIN_CS_ACCEL, PIN_CS_GYRO);
BMP388_DEV bmp388(Wire);

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

// ---------------------- Interrupt handlers ----------------------
void imuInterruptHandler() {
  imuDataReady = true;
  // Read sensor in ISR to clear interrupt flag
  bmi.readSensor();
}

void bmpInterruptHandler() {
  // Mark data-ready; defer sensor reads to main loop to keep ISR short
  bmpDataReady = true;
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  delay(1000);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_CAMERA_TRIGGER, OUTPUT);
  digitalWrite(PIN_CAMERA_TRIGGER, LOW);

  // Test 1: Basic LED (1 blink)
  blinkLED(1);
  delay(1000);

  // Test 2: Serial init (2 blinks)
  Serial1.begin(230400);
  blinkLED(2);
  delay(1000);
  
  // Test 3: SPI0 init (3 blinks)
  pinMode(PIN_CS_ACCEL, OUTPUT);
  pinMode(PIN_CS_GYRO, OUTPUT);
  digitalWrite(PIN_CS_ACCEL, HIGH);
  digitalWrite(PIN_CS_GYRO, HIGH);
  
  SPI.setRX(PIN_SPI_MISO);
  SPI.setTX(PIN_SPI_MOSI);
  SPI.setSCK(PIN_SPI_SCK);
  SPI.begin();
  blinkLED(3);
  delay(1000);
  
  // Test 4: BMI088 init with sync + interrupt (4 blinks)
  int status;
  status = bmi.begin();
  if (status > 0) {
    // Set range then ODR
    if (!bmi.setRange(Bmi088::ACCEL_RANGE_6G, Bmi088::GYRO_RANGE_500DPS)) {
      while(1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(200); }
    }
    if (!bmi.setOdr(Bmi088::ODR_400HZ)) {
      while(1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(300); }
    }

    // Configure synchronized mode with interrupt
    if (!bmi.mapSync(Bmi088::PIN_3)) {  // INT3 for internal sync
      while(1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(400); }
    }
    if (!bmi.mapDrdy(Bmi088::PIN_2)) {  // INT2 outputs data-ready (per example)
      while(1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(500); }
    }
    if (!bmi.pinModeDrdy(Bmi088::PUSH_PULL, Bmi088::ACTIVE_HIGH)) {
      while(1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(600); }
    }

    // Attach interrupt to Pi Pico
    pinMode(PIN_IMU_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuInterruptHandler, RISING);

    blinkLED(4);
  } else {
    // BMI088 failed - continuous fast blink
    while(1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }
  delay(1000);
  
  // Test 5: I2C0 (BMP388) init (5 blinks)
  Wire.setSDA(PIN_BMP_SDA);
  Wire.setSCL(PIN_BMP_SCL);
  Wire.begin();
  blinkLED(5);
  delay(1000);

  // Test 6: BMP388 init with interrupt/FIFO (6 blinks)
  // Try default address (0x77), then alternate (0x76)
  if (!bmp388.begin()) {
    if (!bmp388.begin(0x76)) {
    while(1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
    }
    }
  }
  // Configure I2C clock and ODR/filter for BMP388
  bmp388.setClock(400000);                      // 400 kHz I2C on Wire
  bmp388.setTimeStandby(TIME_STANDBY_80MS);     // 12.5 Hz ~ 10 Hz
  bmp388.setSeaLevelPressure(SEALEVEL_HPA);

  // Enable data ready interrupt (open-drain active-low is common on breakouts)
  bmp388.enableInterrupt(OPEN_COLLECTOR, ACTIVE_LOW, UNLATCHED);

  // Start continuous measurements
  bmp388.startNormalConversion();
  // No SPI interrupt registration needed for I2C

  // Calibrate altitude baseline (single blocking read during setup)
  delay(200);
  if (bmp388.getMeasurements(bmpTemperature, bmpPressure, bmpAltitude)) {
    altitudeBaselineM = bmpAltitude;
    baselineReady = true;
  }

  // Attach interrupt to Pi Pico
  pinMode(PIN_BMP_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BMP_INT), bmpInterruptHandler, FALLING);

  blinkLED(6);
  delay(1000);

  // Test 7: Success! (solid ON)
  // WIRING:
  //   BMI088 INT2 → Pi Pico GPIO 21 (IMU data-ready @ 400 Hz)
  //   BMP388 INT  → Pi Pico GPIO 14 (altimeter data-ready @ ~10 Hz)
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  uint32_t now = millis();

  // --- Handle camera trigger pulse ---
  if (cameraPulseStartUs != 0) {
    uint32_t dt = micros() - cameraPulseStartUs;
    if (dt >= CAMERA_TRIGGER_PULSE_US) {
      digitalWrite(PIN_CAMERA_TRIGGER, LOW);
      cameraPulseStartUs = 0;
    }
  }

  // --- BMI088: Hardware-timed at 400 Hz via interrupt ---
  if (imuDataReady) {
    imuDataReady = false;
    uint32_t imuTs = micros();

    // Sensor already read in ISR, just send the packet
    ImuPacket pkt;
    pkt.header = 0xAA55;
    pkt.timestamp_us = (uint64_t)imuTs;
    pkt.ax = bmi.getAccelX_mss();
    pkt.ay = bmi.getAccelY_mss();
    pkt.az = bmi.getAccelZ_mss();
    pkt.gx = bmi.getGyroX_rads();
    pkt.gy = bmi.getGyroY_rads();
    pkt.gz = bmi.getGyroZ_rads();
    pkt.crc16 = crc16_ibm((uint8_t*)&pkt, sizeof(ImuPacket) - 2);

    Serial1.write((uint8_t*)&pkt, sizeof(ImuPacket));
  }

  // --- Camera trigger: 20 Hz (every 50ms) ---
  if (now - lastCameraTrigger >= 50) {
    lastCameraTrigger = now;
    uint32_t trigTs = micros();

    frameCounter++;
    digitalWrite(PIN_CAMERA_TRIGGER, HIGH);
    cameraPulseStartUs = trigTs;

    TriggerPacket tpkt;
    tpkt.header = 0xBB66;
    tpkt.timestamp_us = (uint64_t)trigTs;
    tpkt.frame_id = frameCounter;
    tpkt.reserved = 0;
    tpkt.crc16 = crc16_ibm((uint8_t*)&tpkt, sizeof(TriggerPacket) - 2);

    Serial1.write((uint8_t*)&tpkt, sizeof(TriggerPacket));
  }

  // --- BMP388: DRDY interrupt at ~10-12.5 Hz ---
  if (bmpDataReady && baselineReady) {
    bmpDataReady = false;
    uint32_t altTs = micros();

    if (bmp388.getMeasurements(bmpTemperature, bmpPressure, bmpAltitude)) {
      float relAltM = bmpAltitude - altitudeBaselineM;

      AltimeterPacket apkt;
      apkt.header = 0xCC77;
      apkt.timestamp_us = (uint64_t)altTs;
      apkt.altitude_m = relAltM;
      apkt.crc16 = crc16_ibm((uint8_t*)&apkt, sizeof(AltimeterPacket) - 2);
      Serial1.write((uint8_t*)&apkt, sizeof(AltimeterPacket));
    }
  }
}
