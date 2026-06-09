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

/*
 * EarthRadiationMsgPayload.h
 *
 * Output message for the earthRadiationModel environment module.
 *
 * Contains total albedo and IR fluxes at the satellite, plus the
 * flux-weighted net force direction vector for each component.
 * These are the inputs required by facetERPDynamicEffector.
 *
 * Why direction vectors are needed
 * --------------------------------
 * Unlike SRP (parallel rays from a point source), Earth is an extended
 * source subtending ~130 deg half-angle at 600 km altitude.
 * Each surface patch pushes the satellite from a different direction.
 * Collapsing to a scalar or assuming anti-nadir direction introduces
 * systematic along-track errors [1],[2]. The flux-weighted direction:
 *
 *   albedoDir_N = normalize( sum_i( dF_alb_i * dir_i ) )
 *
 * is the correct net force direction to pass to the effector.
 *
 * References
 * ----------
 * [1] Knocke et al. (1988). DOI: 10.2514/6.1988-4292
 * [2] Rodriguez-Solano et al. (2012). DOI: 10.1007/s00190-011-0517-4
 */

#ifndef EARTH_RADIATION_MSG_PAYLOAD_H
#define EARTH_RADIATION_MSG_PAYLOAD_H

#include <stdint.h>

/**
 * @brief Output message payload for Earth radiation pressure calculations
 *
 * This message contains the total albedo and infrared radiation fluxes at the satellite,
 * along with flux-weighted net force direction vectors for each component. These values
 * are used as inputs to the facetERPDynamicEffector module.
 *
 * The direction vectors account for Earth's extended source nature, providing the correct
 * net force direction rather than assuming parallel rays or anti-nadir direction.
 */
typedef struct {
    double   albedoFlux;     //!< [W/m^2] total (reflected solar) flux
    double   irFlux;         //!< [W/m^2] total IR flux
    double   albedoDir_N[3]; //!< [-] flux-weighted net albedo force direction,
                             //!<     inertial frame N; [0,0,0] if albedoFlux==0
    double   irDir_N[3];     //!< [-] flux-weighted net IR force direction,
                             //!<     inertial frame N; [0,0,0] if irFlux==0
    uint64_t timeTag;        //!< [ns] simulation time of validity
} EarthRadiationMsgPayload;

#endif /* EARTH_RADIATION_MSG_PAYLOAD_H */
