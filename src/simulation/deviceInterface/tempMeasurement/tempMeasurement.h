/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef TEMPMEASUREMENT_H
#define TEMPMEASUREMENT_H

#include <Eigen/Dense>
#include <cstdint>
#include <random>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/TemperatureMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/gauss_markov.h"

typedef enum {
    TEMP_FAULT_NOMINAL = 0,
    TEMP_FAULT_STUCK_CURRENT = 1,
    TEMP_FAULT_STUCK_VALUE = 2,
    TEMP_FAULT_SPIKING = 3,
    TEMP_FAULT_BIASED = 4,
    TEMP_FAULT_GAUSS_MARKOV = 5
} TempFaultState_t;

/*! @brief Models a sensor to add noise, bias, and faults to temperature measurements.
 */
class TempMeasurement: public SysModel {
public:
    TempMeasurement();
    ~TempMeasurement();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

    /**
     * @brief Set the sensor-error propagation matrix.
     * @param propMatrix One-by-one propagation matrix.
     */
    void setAMatrix(const Eigen::VectorXd& propMatrix);

    /**
     * @brief Get the sensor-error propagation matrix.
     * @return Current one-by-one propagation matrix.
     */
    Eigen::VectorXd getAMatrix() const;

  private:
    void applySensorErrors();

public:
    ReadFunctor<TemperatureMsgPayload> tempInMsg;   //!< True temperature measurement
    Message<TemperatureMsgPayload> tempOutMsg;      //!< Sensed temperature measurement
    BSKLogger bskLogger;                            //!< -- BSK Logging
    TempFaultState_t faultState;                    //!< [-] Fault status variable

    double senBias{};                             //!< [C] Sensor bias value
    double senNoiseStd{};                         //!< [C] Sensor process-noise standard deviation
    double walkBounds;                            //!< [C] Hard bound on the noise state; non-positive disables clipping
    double stuckValue{};                          //!< [C] Value for temp sensor to get stuck at
    double spikeProbability;                      //!< [-] Probability of spiking at each time step (between 0 and 1)
    double spikeAmount;                           //!< [-] Spike multiplier

  private:
    double trueTemperature{};                     //!< [C] Truth value for the temperature measurement
    double sensedTemperature{};                   //!< [C] Temperature measurement as corrupted by noise and faults
    double pastValue{};                           //!< [C] Measurement from last update (used only for faults)

    std::minstd_rand spikeProbabilityGenerator; //!< [-] Generator for calculating probability of a fault spike
    GaussMarkov noiseModel;                     //!< [-] Gauss-Markov noise generation model
    Eigen::VectorXd propagationMatrix;          //!< [-] Sensor-error propagation matrix; defaults to zero
};


#endif
