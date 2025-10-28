/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef antennaOutMsg_H
#define antennaOutMsg_H

/*! antenna state message definition */
typedef struct {
    char   antennaID[20];                         //!< -- Antenna identifier
    double r_AN_N[3];                             //!< m  Position vector from antenna to ground station in inertial frame
    double sigma_NA[3];                           //!< -- MRP attitude of antenna relative to ground station
    double v_AN[3];                               //!< m/s Velocity vector from antenna to ground station in inertial frame
    double P_eirp;                                //!< W  EIRP power transmitted
    double P_N;                                   //!< W  Noise power at the receiver
    uint32_t antennaState;                        //!< Current state of the antenna
    double frequency;                             //!< Hz Operating frequency of the antenna
    double B;                                     //!< Hz Frequency bandwidth of the antenna
    uint32_t environment;                         //!< Environment around the antenna
    double T_amb;                                 //!< K  Ambient temperature around the antenna
    double p_amb;                                 //!< Pa Ambient pressure around the antenna
    double rho_amb;                               //!< g/m^3 Ambient humidity around the antenna
    double x_o2;                                  //!< -- Oxygen content in the ambient air
    double x_h2o;                                 //!< -- Water vapor content in the ambient air
    double r_AP_N[3];                             //!< m  Position vector of the antenna wrt to the celestial body frame {P} (used for ground antennas only)
}AntennaOutMsgPayload;
#endif /* antennaOutMsg_H */
