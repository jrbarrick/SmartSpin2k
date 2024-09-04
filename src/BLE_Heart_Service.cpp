/*
 * Copyright (C) 2020  Anthony Doud & Joel Baranick
 * All rights reserved
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "BLE_Heart_Service.h"
#include <Constants.h>

BLE_Heart_Service::BLE_Heart_Service() : pHeartService(nullptr), heartRateMeasurementCharacteristic(nullptr) {}

void BLE_Heart_Service::setupService(NimBLEServer *pServer, MyCallbacks *chrCallbacks) {
  // HEART RATE MONITOR SERVICE SETUP
  pHeartService                      = spinBLEServer.pServer->createService(HEARTSERVICE_UUID);
  heartRateMeasurementCharacteristic = pHeartService->createCharacteristic(HEARTCHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  byte heartRateMeasurement[2]       = {0x00, 0x00};
  heartRateMeasurementCharacteristic->setValue(heartRateMeasurement, 2);
  heartRateMeasurementCharacteristic->setCallbacks(chrCallbacks);
  pHeartService->start();
}

void BLE_Heart_Service::update() {
  /*if (!spinBLEServer.clientSubscribed.Heartrate) {
    return;
  }*/
  byte heartRateMeasurement[2] = {0x00, (byte)rtConfig->hr.getValue()};
  heartRateMeasurementCharacteristic->setValue(heartRateMeasurement, 2);
  heartRateMeasurementCharacteristic->notify();

  const int kLogBufCapacity = 125;  // Data(10), Sep(data/2), Arrow(3), CharId(37), Sep(3), CharId(37), Sep(3), Name(8), Prefix(2), HR(7), Suffix(2), Nul(1), rounded up
  char logBuf[kLogBufCapacity];
  const size_t heartRateMeasurementLength = sizeof(heartRateMeasurement) / sizeof(heartRateMeasurement[0]);
  logCharacteristic(logBuf, kLogBufCapacity, heartRateMeasurement, heartRateMeasurementLength, HEARTSERVICE_UUID, heartRateMeasurementCharacteristic->getUUID(),
                    "HRS(HRM)[ HR(%d) ]", rtConfig->hr.getValue() % 1000);
}