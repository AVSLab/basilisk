/*
 ISC License

 Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H
#include <vector>
#include <random>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/MagneticFieldMsgPayload.h"
#include "architecture/msgPayloadDefC/TAMSensorMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/gauss_markov.h"
#include "architecture/utilities/saturate.h"
#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>

/*! @class Magnetometer
 * @brief Magnetometer sensor model that simulates magnetic field measurements with configurable noise
 *
 * The magnetometer supports noise configuration through:
 * - Walk bounds: Maximum allowed deviations from truth [T]
 * - Noise std: Standard deviation of measurement noise [T]
 * - AMatrix: Propagation matrix for error model (defaults to identity)
 * - Bias: Static measurement bias [T]
 *
 * Example Python usage:
 * @code
 *     magSensor = Magnetometer()
 *
 *     # Configure noise (Tesla)
 *     magSensor.senNoiseStd = [0.001, 0.001, 0.001]  # Standard deviation per axis
 *     magSensor.setWalkBounds([0.01, 0.01, 0.01])    # Maximum error bounds
 *
 *     # Optional: Set static bias (Tesla)
 *     magSensor.senBias = [0.0001, 0.0001, 0.0001]
 *
 *     # Optional: Configure error propagation (default is identity)
 *     magSensor.setAMatrix([[1,0,0], [0,1,0], [0,0,1]])
 * @endcode
 */
class Magnetometer : public SysModel {
public:
    Magnetometer();
    ~Magnetometer();
    void Reset(uint64_t CurrentClock);          //!< Method for reseting the module
    void UpdateState(uint64_t CurrentSimNanos); //!< Method to update state for runtime
    void readInputMessages();                   //!< Method to read the input messages
    void computeTrueOutput();                   //!< Method to compute the true magnetic field vector
    void computeMagData();                      //!< Method to get the magnetic field vector information
    void applySensorErrors();                   //!< Method to set the actual output of the sensor with errors
    void applySaturation();                     //!< Apply saturation effects to sensed output (floor and ceiling)
    void writeOutputMessages(uint64_t Clock);   //!< Method to write the output message to the system
    Eigen::Matrix3d setBodyToSensorDCM(double yaw, double pitch, double roll); //!< Utility method to configure the sensor DCM

    /*! Sets propagation matrix for error model */
    void setAMatrix(const Eigen::Matrix3d& matrix);

    /*! Gets current propagation matrix */
    Eigen::Matrix3d getAMatrix() const;

public:
    ReadFunctor<SCStatesMsgPayload> stateInMsg;        //!< [-] input message for spacecraft states
    ReadFunctor<MagneticFieldMsgPayload> magInMsg;          //!< [-] input message for magnetic field data in inertial frame N
    Message<TAMSensorMsgPayload> tamDataOutMsg;   //!< [-] Message for TAM output data in sensor frame S
    Eigen::Matrix3d     dcm_SB;                 //!< [-] DCM from body frame to sensor frame
    Eigen::Vector3d     tam_S;                  //!< [T] Magnetic field vector in sensor frame
    Eigen::Vector3d     tamSensed_S;            //!< [T] Measurement including perturbations
    Eigen::Vector3d     tamTrue_S;              //!< [T] Measurement without perturbations
    double              scaleFactor;            //!< [-] Scale factor applied to sensor
    Eigen::Vector3d     senBias;                //!< [T] Sensor bias vector
    Eigen::Vector3d     senNoiseStd;            //!< [T] Sensor noise standard deviation vector

    Eigen::Vector3d     walkBounds;             //!< [T] "3-sigma" errors to permit for states
    double              maxOutput;              //!< [T] Maximum output for saturation application
    double              minOutput;              //!< [T] Minimum output for saturation application
    BSKLogger bskLogger;                          //!< -- BSK Logging

private:
    MagneticFieldMsgPayload magData;             //!< [-] Magnetic field in inertial N frame
    SCStatesMsgPayload stateCurrent;         //!< [-] Current spacecraft state
    uint64_t numStates;                          //!< [-] Number of States for Gauss Markov Models
    GaussMarkov noiseModel;                      //!< [-] Gauss Markov noise generation model
    Saturate saturateUtility;                    //!< [-] Saturation utility
    Eigen::Matrix3d AMatrix;      //!< [-] Error propagation matrix
};


#endif
