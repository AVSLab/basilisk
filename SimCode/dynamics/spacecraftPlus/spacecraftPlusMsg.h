/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

#ifndef _SPACECRAFT_PLUS_MSG_HH_
#define _SPACECRAFT_PLUS_MSG_HH_

#include <string>
#include <cstring>
#include <stdint.h>
#include <Eigen/Dense>
/*! \addtogroup Sim Utility Group
 * @{
 */

/*! This structure is used in the messaging system to communicate what the mass 
    properties of the vehicle are currently.*/
typedef struct {
    double Mass;                     //!< kg   Current spacecraft mass
    double rC_B[3];                  //!< m    Center of mass of spacecraft (relative to struct)
    double ISC_PntB_B[3][3];         //!< kgm2 Inertia tensor of spacecraft (relative to body)
    double dcm_BS[3][3];             //!< -- Transformation from str to body
}SCPlusMassPropsData;

/*! This structure is used in the messaging system to communicate what the 
    state of the vehicle is currently.*/
typedef struct {
	double r_BN_N[3];                 //!< m  Current position vector (inertial)
	double v_BN_N[3];                 //!< m/s Current velocity vector (inertial)
    double sigma_BN[3];               //!< -- Current MRPs (inertial)
	double omega_BN_B[3];             //!< r/s Current angular velocity (inertial)
	double dcm_BS[3][3];              //!< -- Transformation from str to body
	double TotalAccumDVBdy[3];        //!< m/s Accumulated DV for simulation
    uint64_t MRPSwitchCount;          //!< -- Number of times that MRPs have switched
}SCPlusOutputStateData;

/*! @} */

#endif /* _SPACECRAFT_PLUS_MSG_HH_ */
