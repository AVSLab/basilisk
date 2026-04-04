/*
 ISC License

Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#ifndef BASILISK_DOWNLINKHANDLINGMSG_H
#define BASILISK_DOWNLINKHANDLINGMSG_H

#include <stdint.h>

/*! @brief Message for reporting downlink performance and data handling metrics */
typedef struct {
    uint32_t linkActive;                    //!< [-]     1 if link-quality inputs are valid for downlink calculations
    uint32_t receiverIndex;                 //!< [-]     Selected receiver antenna index (1, 2) or 0 if none
    uint64_t maxRetransmissions;            //!< [-]     Maximum ARQ retransmission attempts per packet
    uint32_t removalPolicy;                 //!< [-]     Storage-removal policy (0=REMOVE_ATTEMPTED, 1=REMOVE_DELIVERED_ONLY)
    char transmitterAntennaName[20];        //!< [-]     Name of the selected transmitting antenna
    char receiverAntennaName[20];           //!< [-]     Name of the selected receiving antenna
    char dataName[128];                     //!< [-]     Name of the data partition currently downlinked
    double timeStep;                        //!< [s]     Module integration timestep
    double bandwidth;                       //!< [Hz]    Link overlap bandwidth used in the calculation
    double bitRateRequest;                  //!< [bit/s] Requested channel bit rate
    double packetSizeBits;                  //!< [bit]   Packet size used for BER-to-PER conversion
    double cnr;                             //!< [-]     Selected C/N ratio (linear)
    double cnr_dB;                          //!< [dB]    Selected C/N ratio in decibel
    double cNo_dBHz;                        //!< [dBHz]  Carrier-to-noise density ratio
    double ebN0_dB;                         //!< [dB]    Energy-per-bit to noise-density ratio
    double ber;                             //!< [-]     Bit error rate
    double per;                             //!< [-]     Packet error rate
    double packetSuccessProb;               //!< [-]     Packet success probability within max retransmissions
    double packetDropProb;                  //!< [-]     Packet drop probability within max retransmissions
    double expectedAttemptsPerPacket;       //!< [-]     Expected number of attempts needed to complete one source packet
    double attemptedDataRate;               //!< [bit/s] Attempted channel transmission rate including retransmissions
    double storageRemovalRate;              //!< [bit/s] Data removed from spacecraft storage per selected removal policy
    double deliveredDataRate;               //!< [bit/s] Data delivered successfully to receiver
    double droppedDataRate;                 //!< [bit/s] Data dropped after reaching retransmission limit
    double availableDataBits;               //!< [bit]   Data available in selected storage partition at start of step
    double estimatedRemainingDataBits;      //!< [bit]   Estimated remaining data in selected partition after this step
    double cumulativeAttemptedBits;         //!< [bit]   Cumulative attempted bits including retransmissions
    double cumulativeRemovedBits;           //!< [bit]   Cumulative bits removed from storage
    double cumulativeDeliveredBits;         //!< [bit]   Cumulative successfully delivered bits
    double cumulativeDroppedBits;           //!< [bit]   Cumulative dropped bits
} DownlinkHandlingMsgPayload;

#endif
