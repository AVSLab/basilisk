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

#ifndef _BSK_PRINT_
#define _BSK_PRINT_

typedef enum {
    MSG_DEBUG,
    MSG_INFORMATION,
    MSG_WARNING,
    MSG_ERROR,
    MSG_SILENT          // the coder should never use this flag when using BSK_PRINT().  It is used to turn off all BSK_PRINT()
} msgLevel_t;

#define MSG_LEVEL_DEFAULT MSG_DEBUG   //Define level for bsk logging calls outside of BSKPrint objects

#ifdef __cplusplus
#include <map>
#include <string>

class BSKPrint
{
  public:
    BSKPrint();
    BSKPrint(msgLevel_t msgLevel);
    virtual ~BSKPrint() = default;
    void setPrintLevel(msgLevel_t msgLevel);
    void readPrintLevel();
    void printMessage(msgLevel_t targetLevel, const char* message, ...);
    static void printMessageDefault(msgLevel_t targetLevel, const char* message, ...);

  public:
    std::map<int, const char*> msgLevelMap
    {
      {0, "DEBUG"},
      {1, "INFORMATION"},
      {2, "WARNING"},
      {3, "ERROR"},
      {4, "SILENT"}
    };
    msgLevel_t _msgLevel;
};

#else
typedef struct BSKPrint BSKPrint;
#endif

#ifdef __cplusplus
    #define EXTERN extern "C"
#else
    #define EXTERN
#endif

EXTERN BSKPrint* _BSKPrint(void);
EXTERN void _BSKPrint_D(BSKPrint*);
EXTERN void _readPrintLevel(BSKPrint*);
EXTERN void _setPrintLevel(BSKPrint*, msgLevel_t);
EXTERN void _printMessage(BSKPrint*, msgLevel_t, const char*);
EXTERN void _printMessageDefault(msgLevel_t, const char*);
#endif
