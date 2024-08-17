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

#include "measurementModels.h"

MeasurementModel::MeasurementModel() = default;

MeasurementModel::~MeasurementModel() = default;

/*! Function to represent measurement model which inputs a stateModel and outputs a matrix
 * @param StateVector
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd MeasurementModel::model(const StateVector& state) const {
    return this->measurementModel(state);
}

/*! Set function to represent measurement model which inputs a stateModel and outputs a matrix
 * @param std::function<const Eigen::MatrixXd(const StateVector&)>
 */
void MeasurementModel::setMeasurementModel(const std::function<const Eigen::MatrixXd(const StateVector&)> &modelCalculator) {
    this->measurementModel = modelCalculator;
}

/*! Function to represent measurement matrix which inputs a stateModel and outputs a matrix (partial of measurement
 * @param StateVector
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd MeasurementModel::computeMeasurementMatrix(const StateVector& state) const {
    return this->measurementPartials(state);
}

/*! Set function to represent measurement matrix which inputs a stateModel and outputs a matrix (partial of measurement
 * model with respect to state)
 * @param std::function<const Eigen::MatrixXd(const StateVector&)>
 */
void MeasurementModel::setMeasurementMatrix(const std::function<const Eigen::MatrixXd(const StateVector&)> &hMatrixCalculator){
    this->measurementPartials = hMatrixCalculator;
}

/*! Return the size of the observation
   @return size_t
*/
size_t MeasurementModel::size() const {
    return this->getObservation().size();
}

/*! Get measurement name
 * @return std::string
 */
std::string MeasurementModel::getMeasurementName() const {
    return this->name;
}

/*! Set measurement name
 * @param std::string
 */
void MeasurementModel::setMeasurementName(std::string_view measurementName){
    this->name = measurementName;
}

/*! Get measurement time tag
 * @return double
 */
double MeasurementModel::getTimeTag() const {
    return this->timeTag;
}

/*! Set measurement time tag
 * @param double
 */
void MeasurementModel::setTimeTag(const double measurementTimeTag){
    this->timeTag = measurementTimeTag;
}

/*! Get measurement validity
 * @return Eigen::VectorXd
 */
bool MeasurementModel::getValidity() const {
    return this->validity;
}

/*! Set measurement validity
 * @param bool
 */
void MeasurementModel::setValidity(const bool measurementValidity){
    this->validity = measurementValidity;
}

/*! Get measurement observation
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::getObservation() const {
    return this->observation;
}

/*! Set measurement observation
 * @param Eigen::VectorXd
 */
void MeasurementModel::setObservation(const Eigen::VectorXd& measurementObserved){
    this->observation = measurementObserved;
}

/*! Get measurement noise
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd MeasurementModel::getMeasurementNoise() const {
    return this->noise;
}

/*! Set measurement noise
 * @param Eigen::MatrixXd
 */
void MeasurementModel::setMeasurementNoise(const Eigen::MatrixXd& measurementNoise){
    this->noise = measurementNoise;
}

/*! Get post fit residuals of observation
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::getPostFitResiduals() const {
    return this->postFitResiduals;
}

/*! Set post fit residuals of observation
 * @param Eigen::VectorXd
 */
void MeasurementModel::setPostFitResiduals(const Eigen::VectorXd& measurementPostFit){
    this->postFitResiduals = measurementPostFit;
}

/*! Get pre fit residuals of observation
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::getPreFitResiduals() const {
    return this->preFitResiduals;
}

/*! Set pre fit residuals of observation
 * @param Eigen::VectorXd
 */
void MeasurementModel::setPreFitResiduals(const Eigen::VectorXd& measurementPreFit){
    this->preFitResiduals = measurementPreFit;
}

/*! Measurement model that returns the position component of the state
 * @param StateVector state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::positionStates(const StateVector &state)
{
    return state.getPositionStates();
}

/*! Measurement model that returns the unit vector of the position state
 * @param StateVector state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::normalizedPositionStates(const StateVector &state)
{
    return state.getPositionStates()/state.getPositionStates().norm();
}

/*! Measurement model that returns the position component of the state, and performs a MRP shadow set check
 * @param StateVector state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::mrpStates(const StateVector &state)
{
    return mrpSwitch(state.getPositionStates(), 1);
}

/*! Measurement model that returns the velocity component of the state
 * @param StateVector state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::velocityStates(const StateVector &state)
{
    return state.getVelocityStates();
}

/*! Measurement model that takes first 3 components of the state vector and interprets them as MRPs
 * @param Eigen::VectorXd state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd mrpFirstThreeStates(Eigen::VectorXd state){
    return mrpSwitch(state.head(3), 1);
}
