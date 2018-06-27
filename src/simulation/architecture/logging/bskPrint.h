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

#ifndef _BSKPRINT_
#define _BSKPRINT_

#include <stdio.h>

typedef enum {
    BSK_SILENT,
    BSK_ERROR,
    BSK_WARNING,
    BSK_INFORMATION,
    BSK_DEBUG
} bskMsgLevel_t;

#ifdef __cplusplus
extern "C" {
#endif
    void bskPrint(bskMsgLevel_t msgType, const char *fmt, ...);
    void setMsgLevel(bskMsgLevel_t level);
#ifdef __cplusplus
}
#endif


#endif /* _BSK_PRINT_ */
