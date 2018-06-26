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


#include <stdio.h>
#include "bskPrint.h"
#include <stdarg.h>

bskMsgLevel_t msgOutputLevel = BSK_INFORMATION;

void bskPrint(bskMsgLevel_t msgType, const char *fmt, ...) {

    if (msgType <= msgOutputLevel) {
        va_list vargs;
        va_start(vargs, fmt);

        switch (msgType) {
            case BSK_ERROR:
                printf("BSK_ERROR: ");
                break;
            case BSK_DEBUG:
                printf("BSK_DEBUG: ");
                break;
            case BSK_INFORMATION:
                printf("BSK_INFORMATION: ");
                break;
            case BSK_WARNING:
                printf("BSK_WARNING: ");
                break;
        }
        vprintf(fmt, vargs);
        printf("\n");

        va_end(vargs);
    }

    return;
}

void setMsgLevel(bskMsgLevel_t level) {
    msgOutputLevel = level;
    return;
}

