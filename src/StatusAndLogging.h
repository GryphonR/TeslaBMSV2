#pragma once

#include "pinouts.h"
#include "globals.h"
#include "Logger.h"

void setBMSstatus(int newStatus, const char *message = nullptr);

void setBMSstatus(int newStatus, int newError = 0, const char *message = nullptr);

const char *getBMSStatusString(int status);

const char *getBMSErrorString(int error);

void alarmupdate();

void printbmsstat();