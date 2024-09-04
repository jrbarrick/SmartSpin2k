/*
 * Copyright (C) 2020  Anthony Doud & Joel Baranick
 * All rights reserved
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

// #define CONFIG_SW_COEXIST_ENABLE 1

#include <memory>
#include <NimBLEDevice.h>
#include <Arduino.h>
#include <queue>
#include <deque>
#include "Main.h"
#include "BLE_Definitions.h"

#define BLE_CLIENT_LOG_TAG  "BLE_Client"
#define BLE_COMMON_LOG_TAG  "BLE_Common"
#define BLE_SERVER_LOG_TAG  "BLE_Server"
#define BLE_SETUP_LOG_TAG   "BLE_Setup"
#define FMTS_SERVER_LOG_TAG "FTMS_SERVER"
#define CUSTOM_CHAR_LOG_TAG "Custom_C"

// macros to convert different types of bytes into int The naming here sucks and
// should be fixed.
#define bytes_to_s16(MSB, LSB) (((signed int)((signed char)MSB))) << 8 | (((signed char)LSB))
#define bytes_to_u16(MSB, LSB) (((signed int)((signed char)MSB))) << 8 | (((unsigned char)LSB))
#define bytes_to_int(MSB, LSB) ((static_cast<int>((unsigned char)MSB))) << 8 | (((unsigned char)LSB))
// Potentially, something like this is a better way of doing this ^^

// Setup
void setupBLE();
extern TaskHandle_t BLEClientTask;
// ***********************Common**********************************
void BLECommunications();
// *****************************Server****************************
class MyServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(BLEServer *, ble_gap_conn_desc *desc);
  void onDisconnect(BLEServer *);
  bool onConnParamsUpdateRequest(NimBLEClient *pClient, const ble_gap_upd_params *params);
};

class MyCallbacks : public NimBLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic *);
  void onSubscribe(NimBLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc, uint16_t subValue);
};

// TODO add the rest of the server to this class
class SpinBLEServer {
 private:
  void updateWheelAndCrankRev();

 public:
  struct {
    bool Heartrate : 1;
    bool CyclingPowerMeasurement : 1;
    bool IndoorBikeData : 1;
    bool CyclingSpeedCadence : 1;
  } clientSubscribed;
  NimBLEServer *pServer = nullptr;
  void setClientSubscribed(NimBLEUUID pUUID, bool subscribe);
  void notifyShift();
  double calculateSpeed();
  void update();
  // Queue to store writes to any of the callbacks to the server
  std::queue<std::string> writeCache;
  SpinBLEServer() { memset(&clientSubscribed, 0, sizeof(clientSubscribed)); }
};

extern SpinBLEServer spinBLEServer;

void startBLEServer();
void logCharacteristic(char *buffer, const size_t bufferCapacity, const byte *data, const size_t dataLength, const NimBLEUUID serviceUUID, const NimBLEUUID charUUID,
                       const char *format, ...);
void calculateInstPwrFromHR();
int connectedClientCount();

// BLE FIRMWARE UPDATER
void BLEFirmwareSetup();

// *****************************Client*****************************

// Keeping the task outside the class so we don't need a mask.
// We're only going to run one anyway.
void bleClientTask(void *pvParameters);

// UUID's the client has methods for
// BLEUUID serviceUUIDs[4] = {FITNESSMACHINESERVICE_UUID,
// CYCLINGPOWERSERVICE_UUID, HEARTSERVICE_UUID, FLYWHEEL_UART_SERVICE_UUID};
// BLEUUID charUUIDs[4] = {FITNESSMACHINEINDOORBIKEDATA_UUID,
// CYCLINGPOWERMEASUREMENT_UUID, HEARTCHARACTERISTIC_UUID,
// FLYWHEEL_UART_TX_UUID};

typedef struct NotifyData {
  uint8_t data[25];
  size_t length;
} NotifyData;

class SpinBLEAdvertisedDevice {
 private:
  QueueHandle_t dataBufferQueue = nullptr;

  bool isPostConnected = false;

 public:  // eventually these should be made private
  // // TODO: Do we dispose of this object?  Is so, we need to de-allocate the queue.
  // //       This destructor was called too early and the queue was deleted out from
  // //       under us.
  // ~SpinBLEAdvertisedDevice() {
  //   if (dataBuffer != nullptr) {
  //     Serial.println("Deleting queue");
  //     vQueueDelete(dataBuffer);
  //   }
  // }

  NimBLEAdvertisedDevice *advertisedDevice = nullptr;
  NimBLEAddress peerAddress;

  int connectedClientID = BLE_HS_CONN_HANDLE_NONE;
  BLEUUID serviceUUID   = (uint16_t)0x0000;
  BLEUUID charUUID      = (uint16_t)0x0000;
  bool isHRM            = false;
  bool isPM             = false;
  bool isCSC            = false;
  bool isCT             = false;
  bool isRemote         = false;
  bool doConnect        = false;
  void setPostConnected(bool pc) { isPostConnected = pc; }
  bool getPostConnected() { return isPostConnected; }
  void set(BLEAdvertisedDevice *device, int id = BLE_HS_CONN_HANDLE_NONE, BLEUUID inServiceUUID = (uint16_t)0x0000, BLEUUID inCharUUID = (uint16_t)0x0000);
  void reset();
  void print();
  bool enqueueData(uint8_t data[25], size_t length);
  NotifyData dequeueData();
};

class SpinBLEClient {
 private:
 public:  // Not all of these need to be public. This should be cleaned up
          // later.
  boolean connectedPM            = false;
  boolean connectedHRM           = false;
  boolean connectedCD            = false;
  boolean connectedCT            = false;
  boolean connectedSpeed         = false;
  boolean connectedRemote        = false;
  boolean doScan                 = false;
  bool dontBlockScan             = true;
  int intentionalDisconnect      = 0;
  int noReadingIn                = 0;
  long int cscCumulativeCrankRev = 0;
  double cscLastCrankEvtTime     = 0.0;
  long int cscCumulativeWheelRev = 0;
  double cscLastWheelEvtTime     = 0.0;
  int reconnectTries             = MAX_RECONNECT_TRIES;

  BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;

  // BLEDevices myBLEDevices;
  SpinBLEAdvertisedDevice myBLEDevices[NUM_BLE_DEVICES];

  void start();
  // void serverScan(bool connectRequest);
  bool connectToServer();
  // Check for duplicate services of BLEClient and remove the previously
  // connected one.
  void removeDuplicates(NimBLEClient *pClient);
  void resetDevices(NimBLEClient *pClient);
  void postConnect();
  void FTMSControlPointWrite(const uint8_t *pData, int length);
  void connectBLE_HID(NimBLEClient *pClient);
  void keepAliveBLE_HID(NimBLEClient *pClient);
  void handleBattInfo(NimBLEClient *pClient, bool updateNow);
  // Instead of using this directly, set the .doScan flag to start a scan.
  void scanProcess(int duration = DEFAULT_SCAN_DURATION);
  void checkBLEReconnect();
  // Disconnects all devices. They will then be reconnected if scanned and preferred again.
  void reconnectAllDevices();

  String adevName2UniqueName(NimBLEAdvertisedDevice *inDev);
};

class MyAdvertisedDeviceCallback : public NimBLEAdvertisedDeviceCallbacks {
 public:
  void onResult(NimBLEAdvertisedDevice *);
};

class MyClientCallback : public NimBLEClientCallbacks {
 public:
  void onConnect(BLEClient *);
  void onDisconnect(BLEClient *);
  uint32_t onPassKeyRequest();
  bool onConfirmPIN(uint32_t);
  void onAuthenticationComplete(ble_gap_conn_desc);
};

extern SpinBLEClient spinBLEClient;