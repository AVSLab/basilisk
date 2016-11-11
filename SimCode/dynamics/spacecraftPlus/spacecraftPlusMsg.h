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
    double Mass;                      //!< kg   Current spacecraft mass
    Eigen::Vector3d rC_B;                    //!< m    Center of mass of spacecraft (relative to struct)
    Eigen::Matrix3d ISC_PntB_B;       //!< kgm2 Inertia tensor of spacecraft (relative to body)
    Eigen::Matrix3d dcm_BS;           //!< -- Transformation from str to body
}SCPlusMassPropsData;

/*! This structure is used in the messaging system to communicate what the 
    state of the vehicle is currently.*/
typedef struct {
	Eigen::Vector3d r_N;                    //!< m  Current position vector (inertial)
	Eigen::Vector3d v_N;                    //!< m/s Current velocity vector (inertial)
    Eigen::Vector3d sigma_BN;                  //!< -- Current MRPs (inertial)
	Eigen::Vector3d omega_BN_B;                  //!< r/s Current angular velocity (inertial)
	Eigen::Matrix3d dcm_BS;           //!< -- Transformation from str to body
	Eigen::Vector3d TotalAccumDVBdy;        //!< m/s Accumulated DV for simulation
    uint64_t MRPSwitchCount;          //!< -- Number of times that MRPs have switched
}SCPlusOutputStateData;

/*! @} */

#endif /* _SPACECRAFT_PLUS_MSG_HH_ */
