/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef TIME_CA_H
#define TIME_CA_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefCpp/FilterMsgPayload.h"
#include "architecture/msgPayloadDefCpp/TimeClosestApproachMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"

#include <Eigen/Core>

 /*! @brief A class to perform time of closest approach estimation during a rectilinear flyby */
class TimeClosestApproach: public SysModel {

public:
    TimeClosestApproach();
    ~TimeClosestApproach() override;
    void UpdateState(uint64_t CurrentSimNanos) override;

    ReadFunctor<FilterMsgPayload>  filterInMsg;  //!< relative state and covariance input msg
    Message<TimeClosestApproachMsgPayload> tcaOutMsg; //!< time of closest approach output message

private:
    void readMessages();
    void computeGeometry();
    double computeTca() const;
    double computeTcaStandardDeviation() const;
    void writeMessages(double tCA, double sigmaTca, uint64_t CurrentSimNanos);

    Eigen::Vector3d     v_BN_N;  //!< spacecraft velocity estimate in inertial coordinates
    Eigen::Vector3d     r_BN_N;  //!< spacecraft position estimate in inertial coordinates
    Eigen::Matrix<double, 6, 6> FilterCovariance; //!< filter covariance
    double    FlightPathAngle=-M_PI/2;  //!< flight path angle of the spacecraft at time of read [rad]
    double    Ratio=0;     //!< ratio between relative velocity and position norms at time of read [Hz]
};

#endif
