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

#ifndef _SIGNAL_PROCESSING_H_
#define _SIGNAL_PROCESSING_H_

#include <Eigen/Core>

class LowPassFilter{
public:
    LowPassFilter();
    ~LowPassFilter();

    void setFilterStep(const double filterStepSeconds);
    double getFilterStep() const;
    void setFilterCutoff(const double cutOffValue);
    double getFilterCutoff() const;

    void processMeasurement(const Eigen::Vector3d measurement);
    Eigen::Vector3d getCurrentState() const;

private:
    double filterStep=0.5;         /*!< [s]      filter time step (assumed to be fixed) */
    double filterCutOff=0.1*22/7*2;    /*!< [rad/s]  Cutoff frequency for the filter        */
    Eigen::Vector3d currentState;  /*!< [-] Current state of the filter                 */
    Eigen::Vector3d currentMeasurement;   /*!< [-] Current measurement that we read            */
};

#endif
