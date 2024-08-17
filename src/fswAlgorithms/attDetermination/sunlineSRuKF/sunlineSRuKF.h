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

#ifndef SUNLINESRUKF_H
#define SUNLINESRUKF_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/orbitalMotion.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSUnitConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSArraySensorMsgPayload.h"
#include "architecture/msgPayloadDefCpp/FilterMsgPayload.h"
#include "architecture/msgPayloadDefCpp/FilterResidualsMsgPayload.h"
#include "cMsgCInterface/NavAttMsg_C.h"

#include "fswAlgorithms/_GeneralModuleFiles/srukfInterface.h"
#include "fswAlgorithms/_GeneralModuleFiles/measurementModels.h"

class SunlineSRuKF: public SRukfInterface {
public:
    SunlineSRuKF();
    ~SunlineSRuKF() override;
    void SelfInit() override;                 //!< Self initialization for C-wrapped messages

private:
    void customReset() override;
    void readCssMeasurements();
    void readGyroMeasurements();
    void readFilterMeasurements() override;
    void customFinalizeUpdate() override;
    void writeOutputMessages(uint64_t CurrentSimNanos) override;
    static StateVector stateDerivative(double t, const StateVector &state);

    int filterMeasurement = 0;   //!< [-] Number of measurements of different types being read
    int numActiveCss = 0;        //!< [-] Number of currently active CSS sensors
    double sensorUseThresh = 0;  //!< Threshold below which we discount sensors
    double cssMeasNoiseStd = 0;  //!< [-] CSS measurement noise std
    double gyroMeasNoiseStd = 0; //!< [rad/s] rate gyro measurement noise std
    CSSConfigMsgPayload cssConfigInputBuffer;

public:
    ReadFunctor<NavAttMsgPayload>         navAttInMsg;
    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg;
    ReadFunctor<CSSConfigMsgPayload>      cssConfigInMsg;
    Message<NavAttMsgPayload>             navAttOutMsg;
    NavAttMsg_C                           navAttOutMsgC = {};
    Message<FilterMsgPayload>             filterOutMsg;
    Message<FilterResidualsMsgPayload>    filterGyroResOutMsg;
    Message<FilterResidualsMsgPayload>    filterCssResOutMsg;

    void setCssMeasurementNoiseStd(double cssMeasurementNoiseStd);
    void setGyroMeasurementNoiseStd(double gyroMeasurementNoiseStd);
    double getCssMeasurementNoiseStd() const;
    double getGyroMeasurementNoiseStd() const;
    void setSensorThreshold(double threshold);
    double getSensorThreshold() const;

};

#endif
