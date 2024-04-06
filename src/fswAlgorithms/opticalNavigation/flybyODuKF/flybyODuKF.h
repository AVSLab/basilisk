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


/*! @brief Top level structure for the flyby OD unscented kalman filter.
 Used to estimate the spacecraft's inertial position relative to a body.
 */

#ifndef FLYBYODUKF_H
#define FLYBYODUKF_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/orbitalMotion.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefCpp/OpNavUnitVecMsgPayload.h"
#include "architecture/msgPayloadDefCpp/FilterMsgPayload.h"
#include "architecture/msgPayloadDefCpp/FilterResidualsMsgPayload.h"

#include "fswAlgorithms/_GeneralModuleFiles/srukfInterface.h"
#include "fswAlgorithms/_GeneralModuleFiles/measurementModels.h"

class FlybyODuKF: public SRukfInterface {
public:
    FlybyODuKF();
    ~FlybyODuKF() override;

private:
    void customReset() override;
    void readFilterMeasurements() override;
    void writeOutputMessages(uint64_t CurrentSimNanos) override;
    Eigen::VectorXd propagate(std::array<double, 2> interval, const Eigen::VectorXd& X0, double dt) override;

public:
    ReadFunctor<OpNavUnitVecMsgPayload> opNavHeadingMsg;
    OpNavUnitVecMsgPayload opNavHeadingBuffer;
    Message<NavTransMsgPayload> navTransOutMsg;
    Message<FilterMsgPayload> opNavFilterMsg;
    Message<FilterResidualsMsgPayload> opNavResidualMsg;

    void setMeasurementNoiseScale(const double measurementNoiseScale);
    double getMeasurementNoiseScale() const ;
    void setCentralBodyGravitationParameter(const double mu);
    double getCentralBodyGravitationParameter() const ;


private:

    double measNoiseScaling = 1; //!< [s] Scale factor that can be applied on the measurement noise to over/under weight
    double muCentral = 1; //!< [GM] gravitation parameter of central body
};

#endif
