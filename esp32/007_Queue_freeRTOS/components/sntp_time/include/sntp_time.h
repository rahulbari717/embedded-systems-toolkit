// ========================================
// FILE: include/sntp_time.h
// ========================================
#ifndef SNTP_TIME_H
#define SNTP_TIME_H

#include "esp_err.h"

void sntp(void);

// Utility: print current system time
void print_time(void);

#endif // SNTP_TIME_H