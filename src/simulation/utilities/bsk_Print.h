/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include <stdio.h>



typedef enum {
    MSG_ERROR,           /*!< CSS measurement is set to 0 for all future time */
    MSG_WARNING, /*!< CSS measurement is set to current value for all future time */
    MSG_DEBUG,     /*!< CSS measurement is set to maximum value for all future time */
    MSG_INFORMATION,    /*!< CSS measurement is set to randomly selected value for all future time */
} msgLevel_t;



#define MSG_LEVEL MSG_INFORMATION


#define BSK_MESSAGE(...) fprintf(stderr, __VA_ARGS__)

#ifdef __WIN32__
#define BSK_PRINT(X, _fmt, ...) if(X <= MSG_LEVEL) \
                                   BSK_MESSAGE(_fmt, ##__VA_ARGS__)

#else       /* macOS and Linux */

#define WHERESTR "[FILE : %s, FUNC : %s, LINE : %d]:\n"
#define WHEREARG __FILE__,__func__,__LINE__
#define BSK_PRINT(X, _fmt, ...) if(X <= MSG_LEVEL) \
                                   BSK_MESSAGE(WHERESTR _fmt, WHEREARG,## __VA_ARGS__)
#endif


#endif /* _BSK_PRINT_ */
