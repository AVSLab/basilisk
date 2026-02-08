 /*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef SIM_THRUSTER_CONFIG_H
#define SIM_THRUSTER_CONFIG_H

#include <vector>
#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/THROperation.h"
#include "simulation/dynamics/_GeneralModuleFiles/THRTimePair.h"


//! @brief Container for overall thruster configuration data for single thruster
/*! This structure is used to define the overall configuration of an entire
 thruster.  It holds the current operational data for the thruster, the
 ramp/max/min configuration data, and the physical location/orientation data for
 a thruster. This structure is managed using a `std::shared_ptr` to avoid duplication of configuration data across the simulation and to enable access and updates to the parameters during simulation.*/
typedef struct
//@cond DOXYGEN_IGNORE
THRSimConfig
//@endcond
{
    Eigen::Vector3d thrLoc_B;                       //!< [m] Thruster location expressed in body
    Eigen::Vector3d thrDir_B;                       //!< Thruster force direction unit vector in body
    std::vector<THRTimePair> ThrusterOnRamp;        //!< Percentage of max thrust for ramp up
    std::vector<THRTimePair> ThrusterOffRamp;       //!< Percentage of max thrust for ramp down
	double areaNozzle;								//!< [m^2] Area of nozzle
    double MaxThrust;                               //!< [N] Steady state thrust of thruster
    double steadyIsp;                               //!< [s] Steady state specific impulse of thruster
    double MinOnTime;                               //!< [s]  Minimum allowable on-time
    THROperation ThrustOps;                         //!< Thruster operating data
    double thrusterMagDisp;                         //!< Percentage of magnitude dispersion
    std::vector<double> thrusterDirectionDisp;      //!< Unit vector of dispersed thruster pointing
	bool updateOnly = true;                         //!< Use update only calculations
    char label[10];                                 //!< label name of the TH device being simulated
    double cutoffFrequency;                         //!< [rad/s] cutoff frequency for first-order behavior
    double MaxSwirlTorque;                          //!< [Nm] Steady state magnitude of the swirl torque produced by ionic thrusters
    std::vector<double> thrBlowDownCoeff;           //!< Polynomial coefficients for fuel mass to thrust blow down model in descending order
    std::vector<double> ispBlowDownCoeff;           //!< Polynomial coefficients for fuel mass to Isp blow down model in descending order
}THRSimConfig;


#endif
