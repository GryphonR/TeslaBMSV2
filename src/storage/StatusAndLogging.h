#pragma once

#include "config/pinouts.h"
#include "storage/Logger.h"

void setBMSstatus(int newStatus, const char *message = nullptr);

void setBMSstatus(int newStatus, int newError = 0, const char *message = nullptr);

const char *getBMSStatusString(int status);

const char *getBMSErrorString(int error);

void alarmupdate();

void printbmsstat();