#pragma once
#include <Arduino.h>
void eeBenchUpdate();
bool eeBenchCommand(const String& command);
bool eeTelemetryFresh();
uint32_t eeTelemetrySequence();
