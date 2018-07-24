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

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include <random>
#include "utilities/gauss_markov.h"
#include "utilities/discretize.h"
#include "utilities/saturate.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simFswInterfaceMessages/imuSensorIntMsg.h"
#include <Eigen/Dense>
#include "utilities/avsEigenMRP.h"

//!@brief Sensor model to simulate an IMU.
/*!
 The module
 [PDF Description](Basilisk-IMU-20170712.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 The corruption types are outlined in this
 [PDF document](BasiliskCorruptions.pdf).
 */
class ImuSensor: public SysModel {
public:
    ImuSensor();
    ~ImuSensor();
    
    void CrossInit();
    void SelfInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);
    void setBodyToPlatformDCM(double yaw, double pitch, double roll);
    void computePlatformDR();
    void computePlatformDV(uint64_t CurrentTime);
    void applySensorErrors(uint64_t CurrentTime);
    void applySensorDiscretization(uint64_t CurrentTime);
	void applySensorSaturation(uint64_t CurrentTime);
	void computeSensorErrors();
    void scaleTruth();
    void setLSBs(double LSBa, double LSBo);
    void setCarryError(bool aCarry, bool oCarry);
    void setRoundDirection(roundDirection_t aRound, roundDirection_t oRound);
    void set_oSatBounds(Eigen::MatrixXd oSatBounds);
    void set_aSatBounds(Eigen::MatrixXd aSatBounds);

public:
    std::string InputStateMsg;          /*!< Message name for spacecraft state */
    std::string OutputDataMsg;          /*!< Message name for CSS output data */
    Eigen::Vector3d sensorPos_B;              /*!< [m] IMU sensor location in body */
    Eigen::Matrix3d dcm_PB;                /// -- Transform from body to platform
    Eigen::Vector3d senRotBias;               /// [r/s] Rotational Sensor bias value
    Eigen::Vector3d senTransBias;             /// [m/s2] Translational acceleration sen bias
	double senRotMax;					/// [r/s] Gyro saturation value
	double senTransMax;					/// [m/s2] Accelerometer saturation value
    uint64_t OutputBufferCount;         /// -- number of output msgs stored
    bool NominalReady;                  /// -- Flag indicating that system is in run
    Eigen::Matrix3d PMatrixAccel;   //!< [-] Covariance matrix used to perturb state
	Eigen::Matrix3d AMatrixAccel;   //!< [-] AMatrix that we use for error propagation
	Eigen::Vector3d walkBoundsAccel;//!< [-] "3-sigma" errors to permit for states
	Eigen::Vector3d navErrorsAccel; //!< [-] Current navigation errors applied to truth
	Eigen::Matrix3d PMatrixGyro;    //!< [-] Covariance matrix used to perturb state
	Eigen::Matrix3d AMatrixGyro;    //!< [-] AMatrix that we use for error propagation
	Eigen::Vector3d walkBoundsGyro; //!< [-] "3-sigma" errors to permit for states
	Eigen::Vector3d navErrorsGyro;  //!< [-] Current navigation errors applied to truth

    IMUSensorIntMsg trueValues;         //!< [-] total measurement without perturbations
    IMUSensorIntMsg sensedValues;       //!< [-] total measurement including perturbations
    
    Eigen::Vector3d accelScale;         //! (-) scale factor for acceleration axes
    Eigen::Vector3d gyroScale;          //! (-) scale factors for acceleration axes
    
    Discretize aDisc;                  //!  (-) instance of discretization utility for linear acceleration
    Discretize oDisc;                  //!  (-) instance of idscretization utility for angular rate
    Saturate aSat;                     //!  (-) instance of saturate utility for linear acceleration
    Saturate oSat;                     //!  (-) instance of saturate utility for angular rate
    
private:
    int64_t InputStateID;               /// -- Connect to input time message
    int64_t OutputDataID;               /// -- Connect to output CSS data
    uint64_t PreviousTime;              /// -- Timestamp from previous frame
    uint64_t numStates;                 /// -- Number of States for Gauss Markov Models
    SCPlusStatesSimMsg StatePrevious;   /// -- Previous state to delta in IMU
    SCPlusStatesSimMsg StateCurrent;    /// -- Current SSBI-relative state
    GaussMarkov errorModelAccel;       ///!< [-] Gauss-markov error states
    GaussMarkov errorModelGyro;        ///!< [-] Gauss-markov error states
    
    Eigen::MRPd previous_sigma_BN;              /// -- sigma_BN from the previous spacecraft message
    Eigen::MRPd current_sigma_BN;               /// -- sigma_BN from the most recent spacecraft message
    Eigen::Vector3d previous_omega_BN_B;        /// -- omega_BN_B from the previous spacecraft message
    Eigen::Vector3d current_omega_BN_B;         /// -- omega_BN_B from the current spacecraft message
    Eigen::Vector3d current_nonConservativeAccelpntB_B; /// -- nonConservativeAccelpntB_B from the current message
    Eigen::Vector3d current_omegaDot_BN_B;      /// -- omegaDot_BN_B from the curret spacecraft message
    Eigen::Vector3d previous_TotalAccumDV_BN_B; /// -- TotalAccumDV_BN_B from the previous spacecraft message
    Eigen::Vector3d current_TotalAccumDV_BN_B; /// -- TotalAccumDV_BN_B from the current spacecraft message
    
    Eigen::Vector3d accel_SN_P_out;             /// -- rDotDot_SN_P for either next method or output messages
    Eigen::Vector3d DV_SN_P_out;                /// -- time step deltaV for either next method or output messages
    Eigen::Vector3d omega_PN_P_out;             /// -- omega_PN_P for either next method or output messages
    Eigen::Vector3d prv_PN_out;                 /// -- time step PRV_PN for either next method or output messages
};

#endif
