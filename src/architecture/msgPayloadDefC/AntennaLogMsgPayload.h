/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU,

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

#ifndef antennaLogMsg_H
#define antennaLogMsg_H

/*! antenna state message definition */
typedef struct {
    char        antennaName[20];                       //!< [-]     Antenna Name
    uint32_t    environment;                           //!< [-]     Environment around the antenna (0: space, 1: earth, -1: unknown)
    uint32_t    antennaState;                          //!< [-]     Current state of the antenna (0: off, 1: Rx, 2: Tx, 3: RxTx)
    double      frequency;                             //!< [Hz]    Operating frequency of the antenna
    double      B;                                     //!< [Hz]    Frequency bandwidth of the antenna
    double      HPBW_az;                               //!< [rad]   Half-power beamwidth in azimuth
    double      HPBW_el;                               //!< [rad]   Half-power beamwidth in elevation
    double      P_Tx;                                  //!< [W]     Transmit power
    double      P_Rx;                                  //!< [W]     Receive power
    double      DdB;                                   //!< [dB]    Antenna directivity in decibels
    double      P_N;                                   //!< [W]     Noise power at the receiver
    double      P_eirp_dB;                             //!< [W]     EIRP power transmitted
    double      G_TN;                                  //!< [dB/K]  Antenna gain over system noise temperature
    double      T_Ambient;                             //!< [K]     Ambient temperature around the antenna
    double      r_AN_N[3];                             //!< [m]     Position vector {N} -> {A} of the antenna, decomposed in {N} frame
    double      sigma_AN[3];                           //!< [-]     MRP attitude of antenna relative to inertial frame {N}
    double      v_AN_N[3];                             //!< [m/s]   Velocity vector from antenna to ground station in inertial frame {N}
    double      r_AP_N[3];                             //!< [-]     Position vector of the antenna wrt to the celestial body frame {P} decomposed in {N} (used for ground antennas only)
    double      r_AP_P[3];                             //!< [-]     Position vector of the antenna wrt to the celestial body frame {P} decomposed in {P} (used for ground antennas only)
    double      nHat_LP_N[3];                          //!< [-]     Surface normal vector from the target location in inertial coordinates (used for ground antennas only)
}AntennaLogMsgPayload;
#endif /* antennaLogMsg_H */
