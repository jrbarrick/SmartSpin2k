/*
 * Copyright (C) 2020  Anthony Doud & Joel Baranick
 * All rights reserved
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "Main.h"
#include "SS2KLog.h"
#include "Constants.h"
#include <Arduino.h>

#include <sensors/SensorData.h>
#include <sensors/SensorDataFactory.h>

SensorDataFactory sensorDataFactory;

void collectAndSet(NimBLEUUID charUUID, NimBLEUUID serviceUUID, NimBLEAddress address, uint8_t *pData, size_t length) {
  const int kLogBufMaxLength = 250;
  char logBuf[kLogBufMaxLength];
  SS2K_LOGD(BLE_COMMON_LOG_TAG, "Data length: %d", length);
  int logBufLength = ss2k_log_hex_to_buffer(pData, length, logBuf, 0, kLogBufMaxLength);
  
  logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, "<- %.8s | %.8s", serviceUUID.toString().c_str(), charUUID.toString().c_str());

  std::shared_ptr<SensorData> sensorData = sensorDataFactory.getSensorData(charUUID, (uint64_t)address, pData, length);

  logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, " | %s[", sensorData->getId().c_str());
  if (sensorData->hasHeartRate() && !rtConfig->hr.getSimulate()) {
    int heartRate = sensorData->getHeartRate();
    rtConfig->hr.setValue(heartRate);
    spinBLEClient.connectedHRM = true;
    logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, " HR(%d)", heartRate % 1000);
  }

  if (sensorData->hasCadence() && !rtConfig->cad.getSimulate()) {
    if ((charUUID == PELOTON_DATA_UUID) && !((String(userConfig->getConnectedPowerMeter()) == "none") || (String(userConfig->getConnectedPowerMeter()) == "any"))) {
      // Peloton connected but using BLE Power Meter. So skip cad for Peloton UUID.
    } else {
      float cadence = sensorData->getCadence();
      rtConfig->cad.setValue(cadence);
      spinBLEClient.connectedCD = true;
      logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, " CD(%.2f)", fmodf(cadence, 1000.0));
    }
  }

  if (sensorData->hasPower() && !rtConfig->watts.getSimulate()) {
    if ((charUUID == PELOTON_DATA_UUID) && !((String(userConfig->getConnectedPowerMeter()) == "none") || (String(userConfig->getConnectedPowerMeter()) == "any"))) {
      // Peloton connected but using BLE Power Meter. So skip power for Peloton UUID.
    } else {
      /*
      // Original code
      int power = sensorData->getPower() * userConfig->getPowerCorrectionFactor();
      rtConfig->watts.setValue(power);
      spinBLEClient.connectedPM = true;
      logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, " PW(%d)", power % 10000);
      */

      // Joroto testing
      int jorotoPower = 0;
      int jorotoCadence = 0;
      int potPercent = 0;
      int potValue = 0;
      
      // Do 2 reads to smooth things a bit
      potValue = analogRead(POT_PIN);
      potValue += analogRead(POT_PIN);
      potValue /= 2;
      if (potValue == 4095) {
        logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, "Pot not connected.");
      }
      else if (potValue == 0) {
        logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, "Pot too low.");
      }
      else {
        // Potentiometer < 50 is min according to Joroto display
        if (potValue < 50) { potPercent = 1; }
        // Potentiometer >=2500 is max according to Jorot display
        else if(potValue >= 2500) { potPercent = 100; }
        // Percentage on Joroto display is the potentiometer value / 25
        else { potPercent = potValue / 25; }
        rtConfig->setMinResistance(5);
        rtConfig->setMaxResistance(98);
        rtConfig->resistance.setValue(potPercent);
        logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, " RS(%d)", potPercent);
        jorotoCadence = rtConfig->cad.getValue();
        // Cadence < 15 is treated as 0 by the Joroto display
        if (jorotoCadence < 15) { jorotoCadence = 0; }
        // Joroto power calc gets weird when resistance is below 10%
        if (potPercent < 10) {
          jorotoPower = potPercent * pow((jorotoCadence / 100.0), 1.5) * 7.228958 + (jorotoCadence - (jorotoCadence / 100) * 60);
        } else { 
          jorotoPower = potPercent * pow((jorotoCadence / 100.0), 1.5) * 7.228958 + (jorotoCadence - 40);
        }
        // Can sometimes get weird power cals at very low cadence/resistance levels, just set power to 0
        if (jorotoPower < 0) { jorotoPower = 0; }
        logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, " PW(%d)", jorotoPower);
        rtConfig->watts.setValue(jorotoPower);
        spinBLEClient.connectedPM = true;
      } 
    }
  }

  if (sensorData->hasSpeed()) {
    float speed = sensorData->getSpeed();
    rtConfig->setSimulatedSpeed(speed);
    spinBLEClient.connectedSpeed = true;
    logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, " SD(%.2f)", fmodf(speed, 1000.0));
  }

  if (sensorData->hasResistance()) {
    if ((rtConfig->getMaxResistance() == MAX_PELOTON_RESISTANCE) && (charUUID != PELOTON_DATA_UUID)) {
      // Peloton connected but using BLE Power Meter. So skip resistance for UUID's that aren't Peloton.
    } else {
      int resistance = sensorData->getResistance();
      rtConfig->resistance.setValue(resistance);
      logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, " RS(%d)", resistance % 1000);
    }
  }

  //////adding incline so that i can plot it
  logBufLength += snprintf(logBuf + logBufLength, kLogBufMaxLength - logBufLength, " POS(%d)", ss2k->currentPosition);
  strncat(logBuf + logBufLength, " ]", kLogBufMaxLength - logBufLength);

  SS2K_LOG(BLE_COMMON_LOG_TAG, "%s", logBuf);

#ifdef USE_TELEGRAM
  SEND_TO_TELEGRAM(String(logBuf));
#endif
}