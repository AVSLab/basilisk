/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics,  University of Colorado at Boulder

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

#include "timeClosestApproach.h"

/*! Module constructor */
TimeClosestApproach::TimeClosestApproach() = default;

/*! Module destructor */
TimeClosestApproach::~TimeClosestApproach() = default;

/*! Read input messages.
 @return void
 */
void TimeClosestApproach::readMessages()
{
    /*! - Read the input messages */
    FilterMsgPayload filterState = this->filterInMsg();

    this->r_BN_N = cArray2EigenVector3d(&filterState.state[0]);
    this->v_BN_N = cArray2EigenVector3d(&filterState.state[3]);
    this->FilterCovariance = cArray2EigenMatrixXd(filterState.covar,6,6);
}

/*! Write output messages.
* @return void
* @param tCA time Closest Approach
* @param sigmaTca standard deviation of Time closest approach
* @param CurrentSimNanos Current sim nano

*/
void TimeClosestApproach::writeMessages(const double tCA, const double sigmaTca, const uint64_t CurrentSimNanos)
{
    /*! create and zero the output message */
    TimeClosestApproachMsgPayload tcaMsgBuffer = this->tcaOutMsg.zeroMsgPayload;
    tcaMsgBuffer.timeClosestApproach = tCA;
    tcaMsgBuffer.standardDeviation = sigmaTca;

    /*! Write the output messages */
    this->tcaOutMsg.write(&tcaMsgBuffer, this->moduleID, CurrentSimNanos);
}

/*! Compute flyby geometry variables f0 & gamma0.
 @return void
 */
void TimeClosestApproach::computeGeometry()
{
    /*! - compute velocity/radius ratio at time of read */
    this->Ratio = this->v_BN_N.norm() / this->r_BN_N.norm();

    // compute angle at the time of read
    Eigen::Vector3d r_BN_N_hat = this->r_BN_N.normalized();
    Eigen::Vector3d v_BN_N_hat = this->v_BN_N.normalized();

    double product = -r_BN_N_hat.dot(v_BN_N_hat);
    product = std::max(-1.0, std::min(1.0, product));
    const double theta = std::acos(product);

    // compute flight path angle at the time of read
    this->FlightPathAngle = theta - M_PI / 2.0;
}

/*! Compute time of closest approach.
 * @return double time of closest approach in sec
 */
double TimeClosestApproach::computeTca() const
{
    return -std::sin(this->FlightPathAngle) / this->Ratio;
}

/*! Compute standard deviation of time closest approach.
 @return double standard deviation of time closest approach.
 */
double TimeClosestApproach::computeTcaStandardDeviation() const
{
    Eigen::Vector3d r_BN_N_hat = this->r_BN_N.normalized();
    Eigen::Vector3d v_BN_N_hat = this->v_BN_N.normalized();

    // Calculate covariance_map_to_tca
    Eigen::VectorXd covariance_map_to_tca(6);
    covariance_map_to_tca.head(3) = v_BN_N_hat /r_BN_N.norm();
    covariance_map_to_tca.tail(3) = 1.0 / v_BN_N.norm() * (r_BN_N_hat - std::sin(this->FlightPathAngle) * v_BN_N_hat);

    const double mappedCovariance = covariance_map_to_tca.transpose() * this->FilterCovariance * covariance_map_to_tca;
    const double tCA_covariance = (1.0 / std::pow(this->Ratio, 2)) * mappedCovariance;

    return std::sqrt(tCA_covariance);
}

/*! This method is the main carrier for the time of closest approach calculation
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void TimeClosestApproach::UpdateState(uint64_t CurrentSimNanos)
{
    this->readMessages();
    this->computeGeometry();
    const double tCA = this->computeTca();
    const double sigmaTca = this->computeTcaStandardDeviation();
    this->writeMessages(tCA, sigmaTca, CurrentSimNanos);
}
