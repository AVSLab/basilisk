/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include <stdio.h>
#include <stdarg.h>
#include "utilities/bskLogging.h"

logLevel_t LogLevel = ERROR;

void setDefaultLogLevel(logLevel_t logLevel)
{
  LogLevel = logLevel;
}

logLevel_t getDefaultLogLevel()
{
  return LogLevel;
}

/*! The constructor initialies the logger for a module and uses default verbostiy level for logging */
BSKLogger::BSKLogger()
{
  //Default print verbosity
  this->_logLevel = getDefaultLogLevel();
}

/*! The constructor initialies the logger for a module and uses a user defined verbostiy level for logging */
BSKLogger::BSKLogger(logLevel_t logLevel)
{
  this->_logLevel = logLevel;
}

/*! This method changes the logging verbosity after construction */
void BSKLogger::setLogLevel(logLevel_t logLevel)
{
  this->_logLevel = logLevel;
}

/*! This method reads the current logging verbosity */
void BSKLogger::printLogLevel()
{
  const char* currLevelStr = this->logLevelMap[this->_logLevel];
  printf("Current Message Level: %s\n", currLevelStr);
}

/*! This method logs information. The current behavior is to simply print out the message and the targeted logging level.
    This should be the main method called in user code.
*/
void BSKLogger::bskLog(logLevel_t targetLevel, const char* info, ...)
{
  if(targetLevel >= this->_logLevel)
  {
    const char* targetLevelStr = this->logLevelMap[targetLevel];
    char formatMessage[MAX_LOGGING_LENGTH];
    va_list args;
    va_start (args, info);
    vsnprintf(formatMessage, sizeof(formatMessage), info, args);
    printf("Message Level: %s, Message: %s\n", targetLevelStr, formatMessage);
  }
}

/*! Section contains C interfacing to C++ object */
EXTERN BSKLogger* _BSKLogger(void)
{
    return new BSKLogger();
}

EXTERN void _BSKLogger_d(BSKLogger* bskLogger)
{
    delete bskLogger;
}

EXTERN void _printLogLevel(BSKLogger* bskLogger)
{
    bskLogger->printLogLevel();
}

EXTERN void _setLogLevel(BSKLogger* bskLogger, logLevel_t logLevel)
{
    bskLogger->setLogLevel(logLevel);
}

EXTERN void _bskLog(BSKLogger* bskLogger, logLevel_t logLevel, const char* info)
{
    bskLogger->bskLog(logLevel, info);
}
