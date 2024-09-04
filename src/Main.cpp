/*
 * Copyright (C) 2020  Anthony Doud & Joel Baranick
 * All rights reserved
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "Main.h"
#include "SS2KLog.h"
#include <TMCStepper.h>
#include <Arduino.h>
#include <LittleFS.h>
#include <HardwareSerial.h>
#include "FastAccelStepper.h"
#include "ERG_Mode.h"
#include "UdpAppender.h"
#include "WebsocketAppender.h"
#include "BLE_Custom_Characteristic.h"
#include <Constants.h>
#include "settings.h"

// Stepper Motor Serial
HardwareSerial stepperSerial(2);
TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);  // Hardware Serial

// Peloton Serial
HardwareSerial auxSerial(1);
AuxSerialBuffer auxSerialBuffer;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper     = NULL;

TaskHandle_t maintenanceLoopTask;

Boards boards;
Board currentBoard;

///////////// Initialize the Config /////////////
SS2K *ss2k                       = new SS2K;
userParameters *userConfig       = new userParameters;
RuntimeParameters *rtConfig      = new RuntimeParameters;
physicalWorkingCapacity *userPWC = new physicalWorkingCapacity;

///////////// Log Appender /////////////
UdpAppender udpAppender;
WebSocketAppender webSocketAppender;

///////////// BEGIN SETUP /////////////
#ifndef UNIT_TEST

void SS2K::startTasks() {
  SS2K_LOG(MAIN_LOG_TAG, "Start BLE + ERG Tasks");
  spinBLEClient.intentionalDisconnect = 0;
  setupBLE();
}

void SS2K::stopTasks() {
  SS2K_LOG(BLE_CLIENT_LOG_TAG, "Shutting Down all BLE services");
  spinBLEClient.reconnectTries        = 0;
  spinBLEClient.intentionalDisconnect = NUM_BLE_DEVICES;
  if (NimBLEDevice::getInitialized()) {
    NimBLEDevice::deinit();
    ss2k->stopTasks();
  }
  SS2K_LOG(MAIN_LOG_TAG, "Stop BLE + ERG Tasks");
  if (BLEClientTask != NULL) {
    vTaskDelete(BLEClientTask);
    BLEClientTask = NULL;
  }
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(512000);
  SS2K_LOG(MAIN_LOG_TAG, "Compiled %s%s", __DATE__, __TIME__);
  pinMode(REV_PIN, INPUT);
  int actualVoltage = analogRead(REV_PIN);
  if (actualVoltage - boards.rev1.versionVoltage >= boards.rev2.versionVoltage - actualVoltage) {
    currentBoard = boards.rev2;
  } else {
    currentBoard = boards.rev1;
  }
  SS2K_LOG(MAIN_LOG_TAG, "Current Board Revision is: %s", currentBoard.name);

  // initialize Stepper serial port

  stepperSerial.begin(57600, SERIAL_8N2, currentBoard.stepperSerialRxPin, currentBoard.stepperSerialTxPin);
  // initialize aux serial port (Peloton)
  if (currentBoard.auxSerialTxPin) {
    auxSerial.begin(19200, SERIAL_8N1, currentBoard.auxSerialRxPin, currentBoard.auxSerialTxPin, false);
    if (!auxSerial) {
      SS2K_LOG(MAIN_LOG_TAG, "Invalid Serial Pin Configuration");
    }
    auxSerial.onReceive(SS2K::rxSerial, false);  // setup callback
  }
  // Initialize LittleFS
  SS2K_LOG(MAIN_LOG_TAG, "Mounting Filesystem");
  if (!LittleFS.begin(false)) {
    SS2K_LOG(MAIN_LOG_TAG, "An Error has occurred while mounting LittleFS.");
    LittleFS.format();                     // Format so that the settings can be saved.
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Provide some time for the format to happen.
  }

  // Load Config
  userConfig->loadFromLittleFS();
  userConfig->printFile();  // Print userConfig->contents to serial
  userConfig->saveToLittleFS();

  // load PWC for HR to Pwr Calculation
  userPWC->loadFromLittleFS();
  userPWC->printFile();
  userPWC->saveToLittleFS();

  // Check for firmware update. It's important that this stays before BLE &
  // HTTP setup because otherwise they use too much traffic and the device
  // fails to update which really sucks when it corrupts your settings.
  startWifi();
  httpServer.FirmwareUpdate();

  pinMode(currentBoard.shiftUpPin, INPUT_PULLUP);    // Push-Button with input Pullup
  pinMode(currentBoard.shiftDownPin, INPUT_PULLUP);  // Push-Button with input Pullup
  pinMode(LED_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(currentBoard.enablePin, OUTPUT);
  pinMode(currentBoard.dirPin, OUTPUT);   // Stepper Direction Pin
  pinMode(currentBoard.stepPin, OUTPUT);  // Stepper Step Pin
  digitalWrite(currentBoard.enablePin,
               HIGH);  // Should be called a disable Pin - High Disables FETs
  digitalWrite(currentBoard.dirPin, LOW);
  digitalWrite(currentBoard.stepPin, LOW);
  digitalWrite(LED_PIN, LOW);

  ss2k->setupTMCStepperDriver();

  SS2K_LOG(MAIN_LOG_TAG, "Setting up cpu Tasks");
  disableCore0WDT();  // Disable the watchdog timer on core 0 (so long stepper
                      // moves don't cause problems)

  digitalWrite(LED_PIN, HIGH);

  // Configure and Initialize Logger
  logHandler.addAppender(&webSocketAppender);
  logHandler.addAppender(&udpAppender);
  logHandler.initialize();

  ss2k->startTasks();
  httpServer.start();

  ss2k->resetIfShiftersHeld();
  SS2K_LOG(MAIN_LOG_TAG, "Creating Shifter Interrupts");
  // Setup Interrupts so shifters work anytime
  attachInterrupt(digitalPinToInterrupt(currentBoard.shiftUpPin), ss2k->shiftUp, CHANGE);
  attachInterrupt(digitalPinToInterrupt(currentBoard.shiftDownPin), ss2k->shiftDown, CHANGE);
  digitalWrite(LED_PIN, HIGH);

  xTaskCreatePinnedToCore(SS2K::maintenanceLoop,     /* Task function. */
                          "maintenanceLoopFunction", /* name of task. */
                          MAIN_STACK,                /* Stack size of task */
                          NULL,                      /* parameter of the task */
                          20,                        /* priority of the task */
                          &maintenanceLoopTask,      /* Task handle to keep track of created task */
                          1);                        /* pin task to core */
}

void loop() {  // Delete this task so we can make one that's more memory efficient.
  vTaskDelete(NULL);
}

void SS2K::maintenanceLoop(void *pvParameters) {
  static unsigned long intervalTimer  = millis();
  static unsigned long intervalTimer2 = millis();
  static unsigned long rebootTimer    = millis();
  static bool isScanning              = false;
  static int potValue                 = 0;
  static int potPercent               = 0;
  static int cadence                  = 0;
  static signed int power             = 0;

  while (true) {
    vTaskDelay(5 / portTICK_RATE_MS);

    // Run what used to be in the BLECommunications Task.
    BLECommunications();
    // send BLE notification for any userConfig values that changed.
    BLE_ss2kCustomCharacteristic::parseNemit();
    // Run What used to be in the Stepper Task.
    ss2k->moveStepper();
    // Run what used to be in the ERG Mode Task.
    powerTable->runERG();
    // Run what used to be in the WebClient Task.
    httpServer.webClientUpdate();
    // If we're in ERG mode, modify shift commands to inc/dec the target watts instead.
    ss2k->FTMSModeShiftModifier();
    // If we have a resistance bike attached, slow down when we're close to the limits.
    if (ss2k->pelotonIsConnected) {
      int speed           = userConfig->getStepperSpeed();
      float resistance    = rtConfig->resistance.getValue();
      float maxResistance = rtConfig->getMaxResistance();

      // Slow down when resistance is within 20% of the lower limit
      if (resistance < (maxResistance * 0.2)) {
        float factor = resistance / (maxResistance * 0.2);
        speed        = static_cast<int>(factor * userConfig->getStepperSpeed());
        if (speed < 500) {
          speed = 500;
        }
        if (ss2k->targetPosition > stepper->getCurrentPosition()) {
          speed = userConfig->getStepperSpeed();
        }
      }

      // Slow down when resistance is within 20% of the upper limit
      if (resistance > (maxResistance * 0.8)) {
        float factor = (maxResistance - resistance) / (maxResistance * 0.2);
        speed        = static_cast<int>(factor * userConfig->getStepperSpeed());
        if (speed < 500) {
          speed = 500;
        }
        if (ss2k->targetPosition < stepper->getCurrentPosition()) {
          speed = userConfig->getStepperSpeed();
        }
      }

      ss2k->updateStepperSpeed(speed);
    }

    // if this hardware version has serial pins, check and process their data.
    if (currentBoard.auxSerialTxPin) {
      ss2k->txSerial();
    }

    // Handle flag set for rebooting
    if (ss2k->rebootFlag) {
      static bool _loopOnce = false;
      vTaskDelay(1000 / portTICK_RATE_MS);
      // Let the main task loop complete once before rebooting
      if (_loopOnce) {
        // Important to keep this delay high in order to allow coms to finish.
        vTaskDelay(1000 / portTICK_RATE_MS);
        ESP.restart();
      }
      _loopOnce = true;
    }

    // Handle a flag set to reset SmartSpin2k to defaults
    if (ss2k->resetDefaultsFlag) {
      LittleFS.format();
      userConfig->setDefaults();
      userConfig->saveToLittleFS();
      ss2k->resetDefaultsFlag = false;
      ss2k->rebootFlag        = true;
    }

    // required to set a flag instead of directly calling the function for saving from BLE_Custom Characteristic.
    if (ss2k->saveFlag) {
      ss2k->saveFlag = false;
      userConfig->saveToLittleFS();
      userPWC->saveToLittleFS();
    }

    // Things to do every two seconds
    if ((millis() - intervalTimer) > 2003) {  // add check here for when to restart WiFi
                                              // maybe if in STA mode and 8.8.8.8 no ping return?
      // ss2k->restartWifi();
     
      logHandler.writeLogs();
      webSocketAppender.Loop();
      intervalTimer = millis();
    }

    // Things to do every 6 seconds
    if ((millis() - intervalTimer2) > 6007) {
      // reboot every half hour if not in use.
      static int _oldHR               = 0;
      static int _oldWatts            = 0;
      static double _oldTargetIncline = 0;
      if (_oldHR == rtConfig->hr.getValue() && _oldWatts == rtConfig->watts.getValue() && _oldTargetIncline == rtConfig->getTargetIncline() &&
          NimBLEDevice::getServer()->getConnectedCount() == 0) {
        // Inactivity detected
        if (((millis() - rebootTimer) > 1800000)) {
          // Timer expired
          SS2K_LOGW(MAIN_LOG_TAG, "Rebooting due to inactivity.");
          ss2k->rebootFlag = true;
          logHandler.writeLogs();
          webSocketAppender.Loop();
        }

      } else {
        // We have activity, update monitored values
        _oldHR            = rtConfig->hr.getValue();
        _oldWatts         = rtConfig->watts.getValue();
        _oldTargetIncline = rtConfig->getTargetIncline();
        rebootTimer       = millis();
      }

#ifdef DEBUG_STACK
      Serial.printf("Main Task: %d \n", uxTaskGetStackHighWaterMark(maintenanceLoopTask));
      Serial.printf("Free Heap: %d \n", ESP.getFreeHeap());
      Serial.printf("Best Blok: %d \n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
#endif  // DEBUG_STACK

      intervalTimer2 = millis();
    }
  }
}

#endif  // UNIT_TEST

void SS2K::FTMSModeShiftModifier() {
  int shiftDelta = rtConfig->getShifterPosition() - ss2k->lastShifterPosition;
  if (shiftDelta) {  // Shift detected
    switch (rtConfig->getFTMSMode()) {
      case FitnessMachineControlPointProcedure::SetTargetPower:  // ERG Mode
      {
        rtConfig->setShifterPosition(ss2k->lastShifterPosition);  // reset shifter position because we're remapping it to ERG target
        if ((rtConfig->watts.getTarget() + (shiftDelta * ERG_PER_SHIFT) < userConfig->getMinWatts()) ||
            (rtConfig->watts.getTarget() + (shiftDelta * ERG_PER_SHIFT) > userConfig->getMaxWatts())) {
          SS2K_LOG(MAIN_LOG_TAG, "Shift to %dw blocked", rtConfig->watts.getTarget() + shiftDelta);
          break;
        }
        rtConfig->watts.setTarget(rtConfig->watts.getTarget() + (ERG_PER_SHIFT * shiftDelta));
        SS2K_LOG(MAIN_LOG_TAG, "ERG Shift. New Target: %dw", rtConfig->watts.getTarget());
// Format output for FTMS passthrough
#ifndef INTERNAL_ERG_4EXT_FTMS
        int adjustedTarget         = rtConfig->watts.getTarget() / userConfig->getPowerCorrectionFactor();
        const uint8_t translated[] = {FitnessMachineControlPointProcedure::SetTargetPower, (uint8_t)(adjustedTarget & 0xff), (uint8_t)(adjustedTarget >> 8)};
        spinBLEClient.FTMSControlPointWrite(translated, 3);
#endif
        break;
      }

      case FitnessMachineControlPointProcedure::SetTargetResistanceLevel:  // Resistance Mode
      {
        rtConfig->setShifterPosition(ss2k->lastShifterPosition);  // reset shifter position because we're remapping it to resistance target
        if (pelotonIsConnected) {
          if (rtConfig->resistance.getTarget() + shiftDelta < rtConfig->getMinResistance()) {
            rtConfig->resistance.setTarget(rtConfig->getMinResistance());
            SS2K_LOG(MAIN_LOG_TAG, "Resistance shift less than min %d", rtConfig->getMinResistance());
            break;
          } else if (rtConfig->resistance.getTarget() + shiftDelta > rtConfig->getMaxResistance()) {
            rtConfig->resistance.setTarget(rtConfig->getMaxResistance());
            SS2K_LOG(MAIN_LOG_TAG, "Resistance shift exceeded max %d", rtConfig->getMaxResistance());
            break;
          }
          rtConfig->resistance.setTarget(rtConfig->resistance.getTarget() + shiftDelta);
          SS2K_LOG(MAIN_LOG_TAG, "Resistance Shift. New Target: %d", rtConfig->resistance.getTarget());
        }
        break;
      }

      default:  // Sim Mode
      {
        SS2K_LOG(MAIN_LOG_TAG, "Shift %+d pos %d tgt %d min %d max %d r_min %d r_max %d", shiftDelta, rtConfig->getShifterPosition(), ss2k->targetPosition, rtConfig->getMinStep(),
                 rtConfig->getMaxStep(), rtConfig->getMinResistance(), rtConfig->getMaxResistance());

        if (((ss2k->targetPosition + shiftDelta * userConfig->getShiftStep()) < rtConfig->getMinStep()) ||
            ((ss2k->targetPosition + shiftDelta * userConfig->getShiftStep()) > rtConfig->getMaxStep())) {
          SS2K_LOG(MAIN_LOG_TAG, "Shift Blocked by stepper limits.");
          rtConfig->setShifterPosition(ss2k->lastShifterPosition);
        } else if ((rtConfig->resistance.getValue() <= rtConfig->getMinResistance()) && (shiftDelta > 0)) {
          // User Shifted in the proper direction - allow
        } else if ((rtConfig->resistance.getValue() >= rtConfig->getMaxResistance()) && (shiftDelta < 0)) {
          // User Shifted in the proper direction - allow
        } else if ((rtConfig->resistance.getValue() > rtConfig->getMinResistance()) && (rtConfig->resistance.getValue() < rtConfig->getMaxResistance())) {
          // User Shifted in bounds - allow
        } else {
          // User tried shifting further into the limit - block.
          SS2K_LOG(MAIN_LOG_TAG, "Shift Blocked by resistance limit.");
          rtConfig->setShifterPosition(ss2k->lastShifterPosition);
        }
        uint8_t _controlData[] = {FitnessMachineControlPointProcedure::SetIndoorBikeSimulationParameters, 0x00, 0x00, 0x00, 0x00, 0x28, 0x33};
        spinBLEClient.FTMSControlPointWrite(_controlData, 7);
      }
    }
    ss2k->lastShifterPosition = rtConfig->getShifterPosition();
    BLE_ss2kCustomCharacteristic::notify(BLE_shifterPosition);
  }
}

void SS2K::restartWifi() {
  httpServer.stop();
  vTaskDelay(100 / portTICK_RATE_MS);
  stopWifi();
  vTaskDelay(100 / portTICK_RATE_MS);
  startWifi();
  httpServer.start();
}

void SS2K::moveStepper() {
  bool _stepperDir = userConfig->getStepperDir();
  if (stepper) {
    ss2k->stepperIsRunning = stepper->isRunning();
    ss2k->currentPosition  = stepper->getCurrentPosition();
    if (!ss2k->externalControl) {
      if ((rtConfig->getFTMSMode() == FitnessMachineControlPointProcedure::SetTargetPower) ||
          (rtConfig->getFTMSMode() == FitnessMachineControlPointProcedure::SetTargetResistanceLevel)) {
        ss2k->targetPosition = rtConfig->getTargetIncline();
      } else {
        // Simulation Mode
        ss2k->targetPosition = rtConfig->getShifterPosition() * userConfig->getShiftStep();
        ss2k->targetPosition += rtConfig->getTargetIncline() * userConfig->getInclineMultiplier();
      }
    }

    if (ss2k->syncMode) {
      stepper->stopMove();
      vTaskDelay(100 / portTICK_PERIOD_MS);
      stepper->setCurrentPosition(ss2k->targetPosition);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    if (ss2k->pelotonIsConnected) {
      if ((rtConfig->resistance.getValue() > rtConfig->getMinResistance()) && (rtConfig->resistance.getValue() < rtConfig->getMaxResistance())) {
        stepper->moveTo(ss2k->targetPosition);
      } else if (rtConfig->resistance.getValue() <= rtConfig->getMinResistance()) {  // Limit Stepper to Min Resistance
        if (rtConfig->resistance.getValue() != rtConfig->getMinResistance()) {
          stepper->moveTo(stepper->getCurrentPosition() + 20);
        }
        // Let the user Shift Out of this Position
        if (ss2k->targetPosition > stepper->getCurrentPosition()) {
          stepper->moveTo(ss2k->targetPosition);
        }
      } else {  // Limit Stepper to Max Resistance
        if (rtConfig->resistance.getValue() != rtConfig->getMaxResistance()) {
          stepper->moveTo(stepper->getCurrentPosition() - 20);
        }
        // Let the user Shift Out of this Position
        if (ss2k->targetPosition < stepper->getCurrentPosition()) {
          stepper->moveTo(ss2k->targetPosition);
        }
      }

    } else {
      if ((ss2k->targetPosition >= rtConfig->getMinStep()) && (ss2k->targetPosition <= rtConfig->getMaxStep())) {
        stepper->moveTo(ss2k->targetPosition);
      } else if (ss2k->targetPosition <= rtConfig->getMinStep()) {  // Limit Stepper to Min Position
        stepper->moveTo(rtConfig->getMinStep());
      } else {  // Limit Stepper to Max Position
        stepper->moveTo(rtConfig->getMaxStep());
      }
    }
    rtConfig->setCurrentIncline((float)stepper->getCurrentPosition());
    
    if (connectedClientCount() > 0) {
      stepper->setAutoEnable(false);  // Keep the stepper from rolling back due to head tube slack. Motor Driver still lowers power between moves
      stepper->enableOutputs();
    } else {
      stepper->setAutoEnable(true);  // disable output FETs between moves so stepper can cool. Can still shift.
    }

    if (_stepperDir != userConfig->getStepperDir()) {  // User changed the config direction of the stepper wires
      _stepperDir = userConfig->getStepperDir();
      while (stepper->isRunning()) {  // Wait until the motor stops running
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
      stepper->setDirectionPin(currentBoard.dirPin, _stepperDir);
    }
  }
}

bool IRAM_ATTR SS2K::deBounce() {
  if ((millis() - lastDebounceTime) > debounceDelay) {  // <----------------This should be assigned it's own task and just switch a global bool whatever the reading is at, it's
                                                        // been there for longer than the debounce delay, so take it as the actual current state: if the button state has changed:
    lastDebounceTime = millis();
    return true;
  }

  return false;
}

///////////// Interrupt Functions /////////////
void IRAM_ATTR SS2K::shiftUp() {  // Handle the shift up interrupt IRAM_ATTR is to keep the interrupt code in ram always
  if (ss2k->deBounce()) {
    if (!digitalRead(currentBoard.shiftUpPin)) {  // double checking to make sure the interrupt wasn't triggered by emf
      rtConfig->setShifterPosition(rtConfig->getShifterPosition() - 1 + userConfig->getShifterDir() * 2);
    } else {
      ss2k->lastDebounceTime = 0;
    }  // Probably Triggered by EMF, reset the debounce
  }
}

void IRAM_ATTR SS2K::shiftDown() {  // Handle the shift down interrupt
  if (ss2k->deBounce()) {
    if (!digitalRead(currentBoard.shiftDownPin)) {  // double checking to make sure the interrupt wasn't triggered by emf
      rtConfig->setShifterPosition(rtConfig->getShifterPosition() + 1 - userConfig->getShifterDir() * 2);
    } else {
      ss2k->lastDebounceTime = 0;
    }  // Probably Triggered by EMF, reset the debounce
  }
}

void SS2K::resetIfShiftersHeld() {
  if ((digitalRead(currentBoard.shiftUpPin) == LOW) && (digitalRead(currentBoard.shiftDownPin) == LOW)) {
    SS2K_LOG(MAIN_LOG_TAG, "Resetting to defaults via shifter buttons.");
    for (int x = 0; x < 10; x++) {  // blink fast to acknowledge
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      digitalWrite(LED_PIN, LOW);
    }
    for (int i = 0; i < 20; i++) {
      LittleFS.format();
      userConfig->setDefaults();
      vTaskDelay(200 / portTICK_PERIOD_MS);
      userConfig->saveToLittleFS();
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    ESP.restart();
  }
}

void SS2K::setupTMCStepperDriver() {
  // FastAccel setup
  engine.init();
  stepper = engine.stepperConnectToPin(currentBoard.stepPin);
  stepper->setDirectionPin(currentBoard.dirPin, userConfig->getStepperDir());
  stepper->setEnablePin(currentBoard.enablePin);
  stepper->setAutoEnable(true);
  stepper->setSpeedInHz(DEFAULT_STEPPER_SPEED);
  stepper->setAcceleration(STEPPER_ACCELERATION);
  stepper->setDelayToDisable(1000);

  // TMC Driver Setup
  driver.begin();
  driver.pdn_disable(true);
  driver.mstep_reg_select(true);
  ss2k->updateStepperSpeed();
  ss2k->updateStepperPower();
  driver.microsteps(4);  // Set microsteps to 1/8th
  driver.irun(currentBoard.pwrScaler);
  driver.ihold((uint8_t)(currentBoard.pwrScaler * .5));  // hold current % 0-DRIVER_MAX_PWR_SCALER
  driver.iholddelay(10);                                 // Controls the number of clock cycles for motor
  // power down after standstill is detected
  driver.TPOWERDOWN(128);
  driver.toff(5);
  ss2k->updateStealthChop();
}

// Applies current power to driver
void SS2K::updateStepperPower() {
  uint16_t rmsPwr = (userConfig->getStepperPower());
  driver.rms_current(rmsPwr);
  uint16_t current = driver.cs_actual();
  SS2K_LOG(MAIN_LOG_TAG, "Stepper power is now %d.  read:cs=%U", userConfig->getStepperPower(), current);
}

// Applies current StealthChop to driver
void SS2K::updateStealthChop() {
  bool t_bool = userConfig->getStealthChop();
  driver.en_spreadCycle(!t_bool);
  driver.pwm_autoscale(t_bool);
  driver.pwm_autograd(t_bool);
  SS2K_LOG(MAIN_LOG_TAG, "StealthChop is now %d", t_bool);
}

// Applies userconfig stepper speed if speed not specified
void SS2K::updateStepperSpeed(int speed) {
  if (speed == 0) {
    speed = userConfig->getStepperSpeed();
    SS2K_LOG(MAIN_LOG_TAG, "StepperSpeed is now %d", speed);
  }
  stepper->setSpeedInHz(speed);
}

// Checks the driver temperature and throttles power if above threshold.
void SS2K::checkDriverTemperature() {
  static bool overTemp = false;
  if (static_cast<int>(temperatureRead()) > THROTTLE_TEMP) {  // Start throttling driver power at 72C on the ESP32
    uint8_t throttledPower = (THROTTLE_TEMP - static_cast<int>(temperatureRead())) + currentBoard.pwrScaler;
    driver.irun(throttledPower);
    SS2K_LOG(MAIN_LOG_TAG, "Over temp! Driver is throttling down! ESP32 @ %f C", temperatureRead());
    overTemp = true;
  } else if (static_cast<int>(temperatureRead()) < THROTTLE_TEMP) {
    if (overTemp) {
      SS2K_LOG(MAIN_LOG_TAG, "Temperature is now under control. Driver current reset.");
      driver.irun(currentBoard.pwrScaler);
    }
    overTemp = false;
  }
}

void SS2K::motorStop(bool releaseTension) {
  stepper->stopMove();
  stepper->setCurrentPosition(ss2k->targetPosition);
  if (releaseTension) {
    stepper->moveTo(ss2k->targetPosition - userConfig->getShiftStep() * 4);
  }
}

void SS2K::txSerial() {  // Serial.printf(" Before TX ");
  if (PELOTON_TX && (txCheck >= 1)) {
    static int alternate = 0;
    byte buf[4]          = {PELOTON_REQUEST, 0x00, 0x00, PELOTON_FOOTER};
    switch (alternate) {
      case 0:
        buf[PELOTON_REQ_POS] = PELOTON_POW_ID;
        alternate++;
        break;
      case 1:
        buf[PELOTON_REQ_POS] = PELOTON_CAD_ID;
        alternate++;
        break;
      case 2:
        buf[PELOTON_REQ_POS] = PELOTON_RES_ID;
        alternate            = 0;
        txCheck--;
        break;
    }
    buf[PELOTON_CHECKSUM_POS] = (buf[0] + buf[1]) % 256;
    if (auxSerial.availableForWrite() >= PELOTON_RQ_SIZE) {
      auxSerial.write(buf, PELOTON_RQ_SIZE);
    }
  } else if (PELOTON_TX && txCheck <= 0) {
    if (txCheck == 0) {
      txCheck = -TX_CHECK_INTERVAL;
    } else if (txCheck == -1) {
      txCheck = 1;
    }
    pelotonIsConnected = false;
    rtConfig->setMinResistance(-DEFAULT_RESISTANCE_RANGE);
    rtConfig->setMaxResistance(DEFAULT_RESISTANCE_RANGE);
    txCheck++;
  }
}

void SS2K::pelotonConnected() {
  txCheck = TX_CHECK_INTERVAL;
  if (rtConfig->resistance.getValue() > 0) {
    rtConfig->setMinResistance(MIN_PELOTON_RESISTANCE);
    rtConfig->setMaxResistance(MAX_PELOTON_RESISTANCE);
  } else {
    rtConfig->setMinResistance(-DEFAULT_RESISTANCE_RANGE);
    rtConfig->setMaxResistance(DEFAULT_RESISTANCE_RANGE);
  }
}

void SS2K::rxSerial(void) {
  while (auxSerial.available()) {
    ss2k->pelotonConnected();
    auxSerialBuffer.len = auxSerial.readBytesUntil(PELOTON_FOOTER, auxSerialBuffer.data, AUX_BUF_SIZE);
    for (int i = 0; i < auxSerialBuffer.len; i++) {  // Find start of data string
      if (auxSerialBuffer.data[i] == PELOTON_HEADER) {
        ss2k->pelotonIsConnected = true;
        size_t newLen            = auxSerialBuffer.len - i;  // find length of sub data
        uint8_t newBuf[newLen];
        for (int j = i; j < auxSerialBuffer.len; j++) {
          newBuf[j - i] = auxSerialBuffer.data[j];
        }
        collectAndSet(PELOTON_DATA_UUID, PELOTON_DATA_UUID, PELOTON_ADDRESS, newBuf, newLen);
      }
    }
  }
}
