/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU

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
#include <stdint.h>

#ifndef linkBudgetLogMsg_H
#define linkBudgetLogMsg_H

/*! antenna state message definition */
typedef struct {
    char     antennaName1[20];                         //!< [-]     Antenna identifier
    char     antennaName2[20];                         //!< [-]     Antenna identifier
    uint32_t antennaState2;                          //!< [-]     Current state of the antenna 2 (0: off, 1: Rx, 2: Tx, 3: RxTx)
    uint32_t antennaState1;                          //!< [-]     Current state of the antenna 1 (0: off, 1: Rx, 2: Tx, 3: RxTx)
    double   CNR1;                                   //!< [-]     Carrier to noise ratio at antenna 1
    double   CNR2;                                   //!< [-]     Carrier to noise ratio at antenna 2
    double   distance;                               //!< [m]     Distance between the two antennas
    double   bandwidth;                              //!< [Hz]    Bandwidth of the communication link
    double   frequency;                              //!< [Hz]    Operating frequency of the communication link
}LinkBudgetMsgPayload;
#endif /* linkBudgetLogMsg_H */
