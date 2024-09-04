/*
 * Copyright (C) 2020  Anthony Doud & Joel Baranick
 * All rights reserved
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include "settings.h"
#include "SmartSpin_parameters.h"

#define ERG_MODE_LOG_TAG     "ERG_Mode"
#define ERG_MODE_LOG_CSV_TAG "ERG_Mode_CSV"
#define POWERTABLE_LOG_TAG   "PTable"
#define ERG_MODE_DELAY       700
#define RETURN_ERROR         INT32_MIN

class PowerEntry {
 public:
  int watts;
  int32_t targetPosition;
  int cad;
  int readings;

  PowerEntry() {
    this->watts          = 0;
    this->targetPosition = 0;
    this->cad            = 0;
    this->readings       = 0;
  }
};

class PowerBuffer {
 public:
  PowerEntry powerEntry[POWER_SAMPLES];
  void set(int);
  void reset();
  int getReadings();
};

// Simplifying the table to save memory since we no longer need watts and cad.
class TableEntry {
 public:
  int16_t targetPosition;
  int8_t readings;
  TableEntry() {
    this->targetPosition = INT16_MIN;
    this->readings       = 0;
  }
};

// Combine Entries to make a row.
class TableRow {
 public:
  TableEntry tableEntry[POWERTABLE_WATT_SIZE];
};

class TestResults {
  struct Neighbor {
    unsigned int found : 1;
    unsigned int passedTest : 1;
    int8_t i;
    int8_t j;
    int16_t targetPosition;

    Neighbor() {
      found          = false;
      passedTest     = false;
      i              = INT8_MIN;
      j              = INT8_MIN;
      targetPosition = INT16_MIN;
    }
  };

 public:
  Neighbor leftNeighbor;
  Neighbor rightNeighbor;
  Neighbor topNeighbor;
  Neighbor bottomNeighbor;
  unsigned int allNeighborsFound : 1;
  unsigned int allNeighborsPassed : 1;

  TestResults() {
    allNeighborsFound  = false;
    allNeighborsPassed = false;
  }
};

class PowerTable {
 public:
  TableRow tableRow[POWERTABLE_CAD_SIZE];

  // What used to be in the ERGTaskLoop(). This is the main control function for ERG Mode and the powertable operations.
  void runERG();

  // Pick up new power value and put them into the power table
  void processPowerValue(PowerBuffer& powerBuffer, int cadence, Measurement power);

  // Sets stepper min/max value from power table
  void setStepperMinMax();

  // Catalogs a new entry into the power table.
  void newEntry(PowerBuffer& powerBuffer);

  // returns incline for wattTarget. Null if not found.
  int32_t lookup(int watts, int cad);

  // automatically load or save the Power Table
  bool _manageSaveState();

  // save powertable from littlefs
  bool _save();

  // Reset the active power table and delete the saved power table.
  bool reset();

  // return number of readings in the table.
  int getNumReadings();

  // Display power table in log
  void toLog();

 private:
  unsigned long lastSaveTime     = millis();
  bool _hasBeenLoadedThisSession = false;
  TestResults testNeighbors(int i, int j, int value);
  void fillTable();
  void extrapFillTable();
  void extrapolateDiagonal();
  int getNumEntries();
  // remove entries with < 1 readings
  void clean();
};

class ErgMode {
 public:
  void computeErg();
  void computeResistance();
  void _writeLogHeader();
  void _writeLog(float currentIncline, float newIncline, int currentSetPoint, int newSetPoint, int currentWatts, int newWatts, int currentCadence, int newCadence);

 private:
  bool engineStopped   = false;
  bool initialized     = false;
  int setPoint         = 0;
  int offsetMultiplier = 0;
  int resistance       = 0;
  int cadence          = 0;

  Measurement watts;

  // check if user is spinning, reset incline if user stops spinning
  bool _userIsSpinning(int cadence, float incline);

  // calculate incline if setpoint (from Zwift) changes
  void _setPointChangeState(int newCadence, Measurement& newWatts);

  // calculate incline if setpoint is unchanged
  void _inSetpointState(int newCadence, Measurement& newWatts);

  // update localvalues + incline, creates a log
  void _updateValues(int newCadence, Measurement& newWatts, float newIncline);
};

extern PowerTable* powerTable;
