/*
 ISC License

 Copyright (c) 2024, University of Colorado at Boulder

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

#include "architecture/utilities/signalProcessing.h"

LowPassFilter::LowPassFilter()=default;

LowPassFilter::~LowPassFilter()=default;

/**
 * Process a measurement into the low pass filter
 * @param measurement Eigen::Vector3d
 * @return void
 */
void LowPassFilter::processMeasurement(const Eigen::Vector3d measurement){

    double omegaStep = this->filterStep*this->filterCutOff;
    this->currentState = 1/(2+omegaStep)*(this->currentState*(2-omegaStep) + omegaStep*(measurement+this->currentMeasurement));
    this->currentMeasurement = measurement;
};

/**
 * Get the filter current state
 * @return measurement Eigen::Vector3d
 */
Eigen::Vector3d LowPassFilter::getCurrentState() const {
    return this->currentState;
};

/**
 * Set the time step used in the low pass filter
 * @param filterStepSeconds double
 */
void LowPassFilter::setFilterStep(const double filterStepSeconds){
    this->filterStep = filterStepSeconds;
};

/**
 * Get the time step used in the low pass filter
 * @return filterStepSeconds double
 */
double LowPassFilter::getFilterStep() const{
    return this->filterStep;
};

/**
 * Set the cut off value (norm of the measurements) in the low pass
 * @param cutOffValue double
 */
void LowPassFilter::setFilterCutoff(const double cutOffValue){
    this->filterCutOff = cutOffValue;
};

/**
 * Get the cut off value (norm of the measurements) in the low pass
 * @return cutOffValue double
 */
double LowPassFilter::getFilterCutoff() const{
    return this->filterCutOff;
};
