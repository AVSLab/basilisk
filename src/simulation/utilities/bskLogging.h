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

#ifndef _BSK_LOG_
#define _BSK_LOG_

//maximum length of info to log in a reference to BSKLogging in C, not relevant in C++
#define MAX_LOGGING_LENGTH 255

typedef enum {
    DEBUG,
    INFORMATION,
    WARNING,
    ERROR,
    SILENT          // the coder should never use this flag when using bskLog().  It is used to turn off all output
} logLevel_t;

extern logLevel_t LogLevel;
void printDefaultLogLevel();

#ifdef __cplusplus
#include <map>
#include <string>

void setDefaultLogLevel(logLevel_t logLevel);
logLevel_t getDefaultLogLevel();

class BSKLogger
{
    public:
        BSKLogger();
        BSKLogger(logLevel_t logLevel);
        virtual ~BSKLogger() = default;
        void setLogLevel(logLevel_t logLevel);
        void printLogLevel();
        void bskLog(logLevel_t targetLevel, const char* info, ...);

    //Provides a mapping from log level enum to str
    public:
        std::map<int, const char*> logLevelMap
        {
            {0, "DEBUG"},
            {1, "INFORMATION"},
            {2, "WARNING"},
            {3, "ERROR"},
            {4, "SILENT"}
        };

    private:
        logLevel_t _logLevel;
};

#else
typedef struct BSKLogger BSKLogger;
#endif

#ifdef __cplusplus
    #define EXTERN extern "C"
#else
    #define EXTERN
#endif

EXTERN BSKLogger* _BSKLogger(void);
EXTERN void _BSKLogger_d(BSKLogger*);
EXTERN void _printLogLevel(BSKLogger*);
EXTERN void _setLogLevel(BSKLogger*, logLevel_t);
EXTERN void _bskLog(BSKLogger*, logLevel_t, const char*);
#endif
