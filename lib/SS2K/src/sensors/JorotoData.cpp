/*
 * Copyright (C) 2020  Anthony Doud & Joel Baranick
 * All rights reserved
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "Main.h"
#include <chrono>
#include "Data.h"
#include "endian.h"
#include "sensors/JorotoData.h"

// Replacement for Arduino's millis() using standard C++
static unsigned long getTimeMillis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()
  ).count();
}

bool JorotoData::hasHeartRate() { return false; }

bool JorotoData::hasCadence() { return !std::isnan(this->cadence); }

bool JorotoData::hasPower() { return this->power != INT_MIN; }

bool JorotoData::hasSpeed() { return false; }

bool JorotoData::hasResistance() { return true; }

int JorotoData::getHeartRate() { return INT_MIN; }

float JorotoData::getCadence() { return this->cadence; }

int JorotoData::getPower() { return this->power; }

int JorotoData::getResistance() { return this->resistance; }

float JorotoData::getSpeed() { return nanf(""); }

void JorotoData::decode(uint8_t *data, size_t length) {
  uint8_t flags = data[0];
  int pos = 1;  // Start after flags byte

  // Check wheel revolution data present flag (bit 0)
  if (flags & 0x01) {
    uint32_t wheelRevolutions = get_le32(&data[pos]);
    pos += 4;
    uint16_t wheelEventTime = get_le16(&data[pos]);
    pos += 2;

    // Calculate speed if we have previous measurements
    if (lastWheelEventTime > 0) {
      // Handle timer wraparound (16-bit timer)
      uint16_t timeDiff = (wheelEventTime >= lastWheelEventTime) ? 
                         (wheelEventTime - lastWheelEventTime) : 
                         (65535 - lastWheelEventTime + wheelEventTime);
      
      if (timeDiff > 0) {
        // Convert to meters/second then km/h
        // Time is in 1/1024th of a second
        float wheelCircumference = 2.095f; // Default 700c wheel circumference in meters
        float revolutions = wheelRevolutions - lastWheelRevolutions;
        float timeSeconds = timeDiff / 1024.0f;
        float speedMS = (revolutions * wheelCircumference) / timeSeconds;
        this->speed = speedMS * 3.6f; // Convert m/s to km/h
      }
    }

    lastWheelRevolutions = wheelRevolutions;
    lastWheelEventTime = wheelEventTime;
  }

  // Check crank revolution data present flag (bit 1)
  if (flags & 0x02) {
    uint16_t crankRevolutions = get_le16(&data[pos]);
    pos += 2;
    uint16_t crankEventTime = get_le16(&data[pos]);
    pos += 2;

    // Calculate cadence if we have previous measurements
    if (lastCrankEventTime > 0) {
      // Handle timer wraparound (16-bit timer)
      uint16_t timeDiff = (crankEventTime >= lastCrankEventTime) ?
                         (crankEventTime - lastCrankEventTime) :
                         (UINT16_MAX - lastCrankEventTime + crankEventTime);
      
      if (timeDiff > 0) {
        // Time is in 1/1024th of a second
        float revolutions = crankRevolutions - lastCrankRevolutions;
        float timeMinutes = (timeDiff / 1024.0f) / 60.0f;
        float cadence = std::round(revolutions / timeMinutes);
        
        if (cadence > 1) {
          if (cadence > 200 || cadence < 0) {  // Human is unlikely producing 200+ cadence
            // Cadence Error: Could happen if cadence measurements were missed
            //                Leave cadence unchanged
            cadence = this->cadence;
          }
          this->cadence = cadence;
          this->lastCadUpdateTime = getTimeMillis();
        }
      } else {
        unsigned long currentTime = getTimeMillis();
        if (currentTime - lastCadUpdateTime > 2500) {  // Require five seconds before setting 0 cadence
          this->cadence = 0;
        }
      }
    }

    lastCrankRevolutions = crankRevolutions;
    lastCrankEventTime = crankEventTime;
  }
  
  // Joroto testing
  const int kLogBufMaxLength = 250;
  char logBuf[kLogBufMaxLength];
  int logBufLength = 0;

  int jorotoPower = 0;
  float jorotoCadence = 0;
  int potPercent = 0;
  int potValue = 0;

  // Do 2 reads to smooth things a bit
  potValue = analogRead(POT_PIN);
  potValue += analogRead(POT_PIN);
  potValue /= 2;
  if (potValue == 4095) {
    //SS2K_LOG(JOROTO_LOG_TAG, "Pot not connected.");
    this->power = 0;
    this->lastPwrUpdateTime = getTimeMillis();
  }
  else {
    // Potentiometer < 50 is min according to Joroto display
    if (potValue < 50) { potPercent = 1; }
    // Potentiometer >=2500 is max according to Joroto display
    else if(potValue >= 2500) { potPercent = 100; }
    // Percentage on Joroto display is the potentiometer value / 25
    else { potPercent = potValue / 25; }
    rtConfig->setMinResistance(MIN_JOROTO_RESISTANCE);
    rtConfig->setMaxResistance(MAX_JOROTO_RESISTANCE);
    rtConfig->resistance.setValue(potPercent);
    //SS2K_LOG(JOROTO_LOG_TAG, " RS(%d)", potPercent);
    if (!this->hasCadence()) { jorotoCadence = 0; }
    else { jorotoCadence = this->getCadence(); }
    // Cadence < 15 is treated as 0 by the Joroto display
    if (jorotoCadence < 15) { jorotoCadence = 0; }
    // Joroto power calc gets weird when resistance is below 10%
    if (potPercent < 10) {
      jorotoPower = potValue / 25.0f * pow((jorotoCadence / 100.0f), 1.5f) * 7.228958f + jorotoCadence - (jorotoCadence / 100.0f) * 60.0f;
    } else { 
      jorotoPower = potValue / 25.0f * pow((jorotoCadence / 100.0f), 1.5f) * 7.228958f + jorotoCadence - 40;
    }
    this->resistance = potPercent;

    if (jorotoPower > 0) {
      this->power = jorotoPower;
      this->lastPwrUpdateTime = getTimeMillis();
    } else {
      unsigned long currentTime = getTimeMillis();
      if (currentTime - lastPwrUpdateTime > 2500) {  // Require 2.5 seconds before setting 0 power
        this->power = 0;
      }
    }
    //SS2K_LOG(JOROTO_LOG_TAG, " PW(%d)", jorotoPower);
  }
}
