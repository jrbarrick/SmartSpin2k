/*
 * Copyright (C) 2020  Anthony Doud & Joel Baranick
 * All rights reserved
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "sensors/JorotoData.h"
#include <Arduino.h>

bool JorotoData::hasHeartRate() { return false; }

bool JorotoData::hasCadence() { return false; }

bool JorotoData::hasPower() { return false; }

bool JorotoData::hasSpeed() { return false; }

bool JorotoData::hasResistance() { return true; }

int JorotoData::getHeartRate() { return INT_MIN; }

float JorotoData::getCadence() { return nanf(""); }

int JorotoData::getPower() { return INT_MIN; }

int JorotoData::getResistance() { return this->resistance; }

float JorotoData::getSpeed() { return nanf(""); }

void JorotoData::decode(uint8_t *data, size_t length) {
    power = resistance * pow(cadence / 100, 1.5) * 7.228958 + (cadence - 40);
}
