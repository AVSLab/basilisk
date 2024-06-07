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

#include "stateModels.h"

State::State() = default;

State::~State() = default;

/*! Set the values of a given state
   @param Eigen::VectorXd
*/
void State::setValues(const Eigen::VectorXd &componentValues){
    this->values = componentValues;
}

/*! Get the values of a given state
   @return Eigen::VectorXd
*/
Eigen::VectorXd State::getValues() const {
    return this->values;
}

StateVector::StateVector() = default;

StateVector::~StateVector() = default;

/*! Set the positional components of your state (cartesian position, attitude, etc)
   @param Eigen::VectorXd positionComponents
*/
size_t StateVector::size() const{
    size_t totalSize = 0;
    totalSize += this->position.getValues().size();
    totalSize += this->velocity.getValues().size();
    totalSize += this->acceleration.getValues().size();
    totalSize += this->bias.getValues().size();
    totalSize += this->considerParameters.getValues().size();
    return totalSize;
}

/*! Add two states together
   @param StateVector vector
   @return StateVector sumVector
*/
StateVector StateVector::add(const StateVector &vector) const {
    StateVector sum;
    sum.setPositionStates(this->position.getValues() + vector.getPositionStates());
    sum.setVelocityStates(this->velocity.getValues() + vector.getVelocityStates());
    sum.setAccelerationStates(this->acceleration.getValues() + vector.getAccelerationStates());
    sum.setBiasStates(this->bias.getValues() + vector.getBiasStates());
    sum.setConsiderStates(this->considerParameters.getValues() + vector.getConsiderStates());
    sum.attachSTM(this->stm + vector.detatchSTM());
    return sum;
}

/*! Scale a state vector by a constant
   @param double scalar
   @return StateVector scaledVector
*/
StateVector StateVector::scale(const double scalar) const {
    StateVector scaledVector;
    scaledVector.setPositionStates(this->position.getValues()*scalar);
    scaledVector.setVelocityStates(this->velocity.getValues()*scalar);
    scaledVector.setAccelerationStates(this->acceleration.getValues()*scalar);
    scaledVector.setBiasStates(this->bias.getValues()*scalar);
    scaledVector.setConsiderStates(this->considerParameters.getValues()*scalar);
    scaledVector.attachSTM(this->stm*scalar);
    return scaledVector;
}

/*! Set the positional components of your state (cartesian position, attitude, etc)
   @param Eigen::VectorXd positionComponents
*/
void StateVector::setPositionStates(const Eigen::VectorXd &positionComponents){
    this->position.setValues(positionComponents);
}

/*! Get the positional components of your state (cartesian position, attitude, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd StateVector::getPositionStates() const {
    return this->position.getValues();
}

/*! Set the velocity components of your state (cartesian velocity, angular rate, etc)
   @param Eigen::VectorXd setVelocityStates
*/
void StateVector::setVelocityStates(const Eigen::VectorXd &velocityComponents){
    this->velocity.setValues(velocityComponents);
}

/*! Get the velocity components of your state (cartesian velocity, angular rate, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd StateVector::getVelocityStates() const {
    return this->velocity.getValues();
}

/*! Set the acceleration components of your state (cartesian acceleration, angular acceleration, etc)
   @param Eigen::VectorXd accelerationComponents
*/
void StateVector::setAccelerationStates(const Eigen::VectorXd &accelerationComponents){
    this->acceleration.setValues(accelerationComponents);
}

/*! Get the acceleration components of your state (cartesian acceleration, angular acceleration, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd StateVector::getAccelerationStates() const {
    return this->acceleration.getValues();
}

/*! Set the bias parameters of your state (gyro bias, scale factors, etc)
   @param Eigen::VectorXd biasComponents
*/
void StateVector::setBiasStates(const Eigen::VectorXd &biasComponents){
    this->bias.setValues(biasComponents);
}

/*! Get the bias parameters of your state (gyro bias, scale factors, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd StateVector::getBiasStates() const {
    return this->bias.getValues();
}

/*! Set the consider parameters of your state (gravitational parameter, etc)
   @param Eigen::VectorXd considerParameters
*/
void StateVector::setConsiderStates(const Eigen::VectorXd &considerComponents){
    this->considerParameters.setValues(considerComponents);
}

/*! Get the consider parameters of your state (gravitational parameter, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd StateVector::getConsiderStates() const {
    return this->considerParameters.getValues();
}

/*! Attach the state transition matrix of your state for simultaneous propagation
   @param Eigen::MatrixXd stm
*/
void StateVector::attachSTM(const Eigen::MatrixXd& stm){
    this->stm = stm;
}

/*! Detatch the state transition matrix of your state for simultaneous propagation
   @return Eigen::MatrixXd stm
*/
Eigen::MatrixXd StateVector::detatchSTM() const{
    return this->stm;
}
