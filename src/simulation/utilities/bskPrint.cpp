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
#include "utilities/bskPrint.h"

BSKPrint::BSKPrint()
{
  //Default print verbosity
  this->_msgLevel = MSG_LEVEL_DEFAULT;
}

BSKPrint::BSKPrint(msgLevel_t msgLevel)
{
  this->_msgLevel = msgLevel;
}

void BSKPrint::setPrintLevel(msgLevel_t msgLevel)
{
  this->_msgLevel = msgLevel;
}

void BSKPrint::readPrintLevel()
{
  const char* currLevelStr = this->msgLevelMap[this->_msgLevel];
  printf("Current Message Level: %s\n", currLevelStr);
}

void BSKPrint::printMessage(msgLevel_t targetLevel, const char* message, ...)
{
  if(targetLevel >= this->_msgLevel)
  {
    const char* targetLevelStr = this->msgLevelMap[targetLevel];
    char formatMessage[255];
    va_list args;
    va_start (args, message);
    vsnprintf(formatMessage, sizeof(formatMessage), message, args);
    printf("Message Level: %s, Message: %s\n", targetLevelStr, formatMessage);
  }
}

void BSKPrint::printMessageDefault(msgLevel_t targetLevel, const char* message, ...)
{
  if(targetLevel >= MSG_LEVEL_DEFAULT)
  {
    char formatMessage[255];
    va_list args;
    va_start(args, message);
    vsnprintf(formatMessage, sizeof(formatMessage), message, args);
    printf("Message: %s\n", formatMessage);
  }
}

EXTERN BSKPrint* _BSKPrint(void)
{
    return new BSKPrint();
}

EXTERN void _BSKPrint_D(BSKPrint* bskPrint)
{
    delete bskPrint;
}

EXTERN void _readPrintLevel(BSKPrint* bskPrint)
{
    bskPrint->readPrintLevel();
}

EXTERN void _setPrintLevel(BSKPrint* bskPrint, msgLevel_t msgLevel)
{
    bskPrint->setPrintLevel(msgLevel);
}

EXTERN void _printMessage(BSKPrint* bskPrint, msgLevel_t msgLevel, const char* message)
{
    bskPrint->printMessage(msgLevel, message);
}

EXTERN void _printMessageDefault(msgLevel_t msgLevel, const char* message)
{
    BSKPrint::printMessageDefault(msgLevel, message);
}
