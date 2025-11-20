/**
  ******************************************************************************
  * @file    debug_utils.h
  * @brief   Debug and utility functions with printf support
  ******************************************************************************
  */

#ifndef DEBUG_UTILS_H_
#define DEBUG_UTILS_H_

#include "main.h"
#include <stdarg.h>

/* Function prototypes */
void DebugPrint(const char *format, ...);  // Now supports printf-style formatting
void DebugPrintSimple(const char *str);    // Original simple version
void SendTestMessage(void);

#endif /* DEBUG_UTILS_H_ */
