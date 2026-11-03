#include <Arduino.h>
#include <Wire.h>
#include "full.h"

// =========================
// Konfiguracja
// =========================
static constexpr uint8_t EEPROM_ADDR = 0x50;
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;
static constexpr uint32_t I2C_FREQ = 100000;

// CAT24C512 ma page write 128 B, ale na start bezpiecznie
static constexpr size_t WRITE_CHUNK = 16;

// =========================
// I2C utils
// =========================
bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

bool waitEepromReady(uint32_t timeoutMs = 100) {
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    if (i2cPing(EEPROM_ADDR)) return true;
    delay(2);
  }
  return false;
}

bool writeChunk(uint16_t memAddr, const uint8_t* data, size_t len) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((uint8_t)(memAddr >> 8));
  Wire.write((uint8_t)(memAddr & 0xFF));

  for (size_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }

  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf("[FAIL] Write error=%u at 0x%04X\n", err, memAddr);
    return false;
  }

  if (!waitEepromReady()) {
    Serial.printf("[FAIL] EEPROM timeout at 0x%04X\n", memAddr);
    return false;
  }

  return true;
}

bool readBytes(uint16_t memAddr, uint8_t* data, size_t len) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((uint8_t)(memAddr >> 8));
  Wire.write((uint8_t)(memAddr & 0xFF));

  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    Serial.printf("[FAIL] Set address error=%u at 0x%04X\n", err, memAddr);
    return false;
  }

  size_t got = Wire.requestFrom((int)EEPROM_ADDR, (int)len);
  if (got != len) {
    Serial.printf("[FAIL] Read len mismatch at 0x%04X req=%u got=%u\n",
                  memAddr, (unsigned)len, (unsigned)got);
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    if (!Wire.available()) {
      Serial.printf("[FAIL] No data at 0x%04X\n", memAddr);
      return false;
    }
    data[i] = Wire.read();
  }

  return true;
}

void scanI2C() {
  Serial.println("=== I2C SCAN ===");
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2cPing(addr)) {
      Serial.printf("[I2C] Found 0x%02X\n", addr);
    }
  }
  Serial.println("================");
}

void dumpBytes(uint16_t start, uint16_t count) {
  uint8_t buf[16];
  Serial.printf("[DUMP] From 0x%04X:\n", start);

  for (uint16_t addr = start; addr < start + count; addr += 16) {
    uint16_t len = min((uint16_t)16, (uint16_t)(start + count - addr));

    if (!readBytes(addr, buf, len)) {
      Serial.printf("[FAIL] Dump read failed at 0x%04X\n", addr);
      return;
    }

    Serial.printf("0x%04X : ", addr);
    for (uint16_t i = 0; i < len; i++) {
      Serial.printf("%02X ", buf[i]);
    }
    Serial.println();
  }
}

void printRegionPointers() {
  uint8_t buf[4];

  if (readBytes(0x0000, buf, 4)) {
    uint32_t lowPtr = (uint32_t)buf[0]
                    | ((uint32_t)buf[1] << 8)
                    | ((uint32_t)buf[2] << 16)
                    | ((uint32_t)buf[3] << 24);
    Serial.printf("[INFO] LowRegionStart  @0x0000 = 0x%08lX\n", (unsigned long)lowPtr);
  } else {
    Serial.println("[FAIL] Cannot read LowRegionStart");
  }

  if (readBytes(0x0400, buf, 4)) {
    uint32_t highPtr = (uint32_t)buf[0]
                     | ((uint32_t)buf[1] << 8)
                     | ((uint32_t)buf[2] << 16)
                     | ((uint32_t)buf[3] << 24);
    Serial.printf("[INFO] HighRegionStart @0x0400 = 0x%08lX\n", (unsigned long)highPtr);
  } else {
    Serial.println("[FAIL] Cannot read HighRegionStart");
  }
}

bool programFullFlash() {
  const size_t total = sizeof(tps25750x_fullFlash_i2c_array);
  Serial.printf("[INFO] Full flash size: %u bytes\n", (unsigned)total);

  uint8_t tx[WRITE_CHUNK];
  uint8_t rx[WRITE_CHUNK];

  for (size_t offset = 0; offset < total; offset += WRITE_CHUNK) {
    size_t len = min((size_t)WRITE_CHUNK, total - offset);

    for (size_t i = 0; i < len; i++) {
      tx[i] = (uint8_t)tps25750x_fullFlash_i2c_array[offset + i];
    }

    if (!writeChunk((uint16_t)offset, tx, len)) {
      Serial.printf("[FAIL] Write failed at 0x%04X\n", (unsigned)offset);
      return false;
    }

    if (!readBytes((uint16_t)offset, rx, len)) {
      Serial.printf("[FAIL] Readback failed at 0x%04X\n", (unsigned)offset);
      return false;
    }

    for (size_t i = 0; i < len; i++) {
      if (rx[i] != tx[i]) {
        Serial.printf("[FAIL] Verify error @0x%04X exp=0x%02X got=0x%02X\n",
                      (unsigned)(offset + i), tx[i], rx[i]);
        return false;
      }
    }

    if ((offset % 1024) == 0 || (offset + len) == total) {
      Serial.printf("[OK] Progress %u / %u\n",
                    (unsigned)(offset + len), (unsigned)total);
    }
  }

  Serial.println("[OK] Full flash programmed and verified.");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("====================================");
  Serial.println("ESP32 TPS25751 FULL FLASH WRITER");
  Serial.println("====================================");

  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
  delay(100);

  scanI2C();

  if (!i2cPing(EEPROM_ADDR)) {
    Serial.printf("[FAIL] EEPROM 0x%02X not found\n", EEPROM_ADDR);
    return;
  }

  Serial.printf("[OK] EEPROM found at 0x%02X\n", EEPROM_ADDR);

  if (programFullFlash()) {
    printRegionPointers();
    dumpBytes(0x0000, 64);
    dumpBytes(0x03F0, 32);
    dumpBytes(0x07F0, 32);
    Serial.println("[OK] DONE");
    Serial.println("[INFO] Disconnect ESP from I2Cc and power-cycle TPS25751.");
  } else {
    Serial.println("[FAIL] Programming failed.");
  }
}

void loop() {
}