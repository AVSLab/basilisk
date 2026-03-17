/*
 ISC License

 Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder

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

#ifndef WindMsgPayload_H
#define WindMsgPayload_H

/**
 * @brief Local atmospheric velocity information in inertial frame N.
 *
 * `v_air_N` is the full air-mass velocity used for aerodynamic relative velocity calculations.
 * `v_wind_N` is the wind-only contribution, excluding any background co-rotating atmosphere.
 *
 * When a co-rotating atmosphere assumption is used:
 *
 *   `v_air_N = (omega_planet_N x r_N) + v_wind_N`
 */
typedef struct {
    double v_air_N[3];   //!< [m/s] Full local air velocity in frame N.
    double v_wind_N[3];  //!< [m/s] Wind perturbation in frame N, not including atmospheric co-rotation.
} WindMsgPayload;

#endif
