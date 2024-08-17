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
long StateVector::size() const{
    long totalSize = 0;
    if (this->hasPosition()){
        totalSize += this->getPositionStates().size();
    }
    if (this->hasVelocity()){
        totalSize += this->getVelocityStates().size();
    }
    if (this->hasAcceleration()){
        totalSize += this->getAccelerationStates().size();
    }
    if (this->hasBias()){
        totalSize += this->getBiasStates().size();
    }
    if (this->hasConsider()){
        totalSize += this->getConsiderStates().size();
    }
    return totalSize;
}

/*! Add a eigen vector to a state, assuming the order of position, velocity, acceleration, bias, consider
   @param Eigen::VectorXd vector
   @return StateVector sumVector
*/
StateVector StateVector::addVector(const Eigen::VectorXd &vector) const {
    assert(vector.size() == this->size());
    long lastIndex = 0;
    StateVector sum;
    if (this->hasPosition()){
        PositionState positionState;
        positionState.setValues(this->getPositionStates() + vector.segment(0, this->getPositionStates().size()));
        sum.setPosition(positionState);
        lastIndex += this->getPositionStates().size();
    }
    if (this->hasVelocity()){
        VelocityState velocityState;
        velocityState.setValues(this->getVelocityStates() + vector.segment(lastIndex, this->getVelocityStates().size()));
        sum.setVelocity(velocityState);
        lastIndex += this->getVelocityStates().size();
    }
    if (this->hasAcceleration()){
        AccelerationState accelerationState;
        accelerationState.setValues(this->getAccelerationStates() + vector.segment(
                lastIndex,this->getAccelerationStates().size()));
        sum.setAcceleration(accelerationState);
        lastIndex += this->getAccelerationStates().size();
    }
    if (this->hasBias()){
        BiasState biasState;
        biasState.setValues(this->getBiasStates() + vector.segment(lastIndex, this->getBiasStates().size()));
        sum.setBias(biasState);
        lastIndex += this->getBiasStates().size();
    }
    if (this->hasConsider()){
        ConsiderState considerState;
        considerState.setValues(this->getConsiderStates() + vector.segment(lastIndex, this->getConsiderStates().size()));
        sum.setConsider(considerState);
    }
    return sum;
}

/*! Add two states together
   @param StateVector vector
   @return StateVector sumVector
*/
StateVector StateVector::add(const StateVector &vector) const {
    StateVector sum;
    if (this->hasPosition() && vector.hasPosition()){
        PositionState sumPosition;
        sumPosition.setValues(this->getPositionStates() + vector.getPositionStates());
        sum.setPosition(sumPosition);
    }
    if (this->hasVelocity() && vector.hasVelocity()){
        VelocityState sumVelocity;
        sumVelocity.setValues(this->getVelocityStates() + vector.getVelocityStates());
        sum.setVelocity(sumVelocity);
    }
    if (this->hasAcceleration() && vector.hasAcceleration()){
        AccelerationState sumAcceleration;
        sumAcceleration.setValues(this->getAccelerationStates() + vector.getAccelerationStates());
        sum.setAcceleration(sumAcceleration);
    }
    if (this->hasBias() && vector.hasBias()){
        BiasState sumBias;
        sumBias.setValues(this->getBiasStates() + vector.getBiasStates());
        sum.setBias(sumBias);
    }
    if (this->hasConsider() && vector.hasConsider()){
        ConsiderState sumConsider;
        sumConsider.setValues(this->getConsiderStates() + vector.getConsiderStates());
        sum.setConsider(sumConsider);
    }
    sum.attachStm(this->detachStm() + vector.detachStm());
    return sum;
}

/*! Scale a state vector by a constant
   @param double scalar
   @return StateVector scaledVector
*/
StateVector StateVector::scale(const double scalar) const {
    StateVector scaledVector;
    if (this->hasPosition()){
        PositionState scaledPosition;
        scaledPosition.setValues(this->getPositionStates()*scalar);
        scaledVector.setPosition(scaledPosition);
    }
    if (this->hasVelocity()){
        VelocityState scaledVelocity;
        scaledVelocity.setValues(this->getVelocityStates()*scalar);
        scaledVector.setVelocity(scaledVelocity);
    }
    if (this->hasAcceleration()){
        AccelerationState scaledAcceleration;
        scaledAcceleration.setValues(this->getAccelerationStates()*scalar);
        scaledVector.setAcceleration(scaledAcceleration);
    }
    if (this->hasBias()){
        BiasState scaledBias;
        scaledBias.setValues(this->getBiasStates()*scalar);
        scaledVector.setBias(scaledBias);
    }
    if (this->hasConsider()){
        ConsiderState scaledConsider;
        scaledConsider.setValues(this->getConsiderStates()*scalar);
        scaledVector.setConsider(scaledConsider);
    }
    scaledVector.attachStm(this->stm*scalar);
    return scaledVector;
}

/*! Return the full state vector
   @return Eigen::VectorXd fullStateVector
*/
Eigen::VectorXd StateVector::returnValues() const {
    long numerOfStates = this->size();
    Eigen::VectorXd stateVectorValues;
    stateVectorValues.setZero(numerOfStates);
    long lastIndex = 0;
    if (this->hasPosition()){
        stateVectorValues.segment(0, this->getPositionStates().size()) = this->getPositionStates();
        lastIndex += this->getPositionStates().size();
    }
    if (this->hasVelocity()){
        stateVectorValues.segment(lastIndex, this->getVelocityStates().size()) = this->getVelocityStates();
        lastIndex += this->getVelocityStates().size();
    }
    if (this->hasAcceleration()){
        stateVectorValues.segment(lastIndex, this->getAccelerationStates().size()) = this->getAccelerationStates();
        lastIndex += this->getAccelerationStates().size();
    }
    if (this->hasBias()){
        stateVectorValues.segment(lastIndex, this->getBiasStates().size()) = this->getBiasStates();
        lastIndex += this->getBiasStates().size();
    }
    if (this->hasConsider()){
        stateVectorValues.segment(lastIndex, this->getConsiderStates().size()) = this->getConsiderStates();
    }
    return stateVectorValues;
}

/*! Check if the state vector has a position state
   @return bool
*/
bool StateVector::hasPosition() const {
    return this->position.has_value();
}

/*! Set the positional components of your state (cartesian position, attitude, etc)
   @param Eigen::VectorXd positionComponents
*/
void StateVector::setPosition(const PositionState &positionState){
    this->position = positionState;
}

/*! Get the positional components of your state (cartesian position, attitude, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd StateVector::getPositionStates() const {
    return this->getPosition().getValues();
}


/*! Get the positional state class of your state vector(cartesian position, attitude, etc)
   @return PositionState
*/
PositionState StateVector::getPosition() const {
    return this->position.value();
}

/*! Check if the state vector has a velocity state
   @return bool
*/
bool StateVector::hasVelocity() const {
    return this->velocity.has_value();
}

/*! Set the velocity components of your state (cartesian velocity, angular rate, etc)
   @param Eigen::VectorXd velocityComponents
*/
void StateVector::setVelocity(const VelocityState &velocityState){
    this->velocity = velocityState;
}

/*! Get the velocity class of your state (cartesian velocity, angular rate, etc)
   @return Eigen::VectorXd
*/
VelocityState StateVector::getVelocity() const {
    return this->velocity.value();
}

/*! Get the velocity components of your state (cartesian velocity, angular rate, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd StateVector::getVelocityStates() const {
    return this->getVelocity().getValues();
}

/*! Check if the state vector has a acceleration state
   @return bool
*/
bool StateVector::hasAcceleration() const {
    return this->acceleration.has_value();
}

/*! Set the acceleration class of your state (cartesian acceleration, angular acceleration, etc)
   @param Eigen::VectorXd velocityComponents
*/
void StateVector::setAcceleration(const AccelerationState &accelerationState){
    this->acceleration = accelerationState;
}

/*! Get the velocity class of your state (cartesian acceleration, angular acceleration, etc)
   @return Eigen::VectorXd
*/
AccelerationState StateVector::getAcceleration() const {
    return this->acceleration.value();
}

/*! Get the acceleration components of your state (cartesian acceleration, angular acceleration, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd StateVector::getAccelerationStates() const {
    return this->getAcceleration().getValues();
}

/*! Check if the state vector has a bias state
   @return bool
*/
bool StateVector::hasBias() const {
    return this->bias.has_value();
}

/*! Set the bias class of your state (cartesian bias, angular bias, etc)
   @param Eigen::VectorXd velocityComponents
*/
void StateVector::setBias(const BiasState &biasState){
    this->bias = biasState;
}

/*! Get the velocity class of your state (cartesian bias, angular bias, etc)
   @return Eigen::VectorXd
*/
BiasState StateVector::getBias() const {
    return this->bias.value();
}

/*! Get the bias components of your state (cartesian bias, angular bias, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd StateVector::getBiasStates() const {
    return this->getBias().getValues();
}

/*! Check if the state vector has a considerParameters state
   @return bool
*/
bool StateVector::hasConsider() const {
    return this->considerParameters.has_value();
}

/*! Set the considerParameters class of your state (cartesian considerParameters, angular considerParameters, etc)
   @param Eigen::VectorXd velocityComponents
*/
void StateVector::setConsider(const ConsiderState &considerParametersState){
    this->considerParameters = considerParametersState;
}

/*! Get the velocity class of your state (cartesian considerParameters, angular considerParameters, etc)
   @return Eigen::VectorXd
*/
ConsiderState StateVector::getConsider() const {
    return this->considerParameters.value();
}

/*! Get the considerParameters components of your state (cartesian considerParameters, angular considerParameters, etc)
   @return Eigen::VectorXd
*/
Eigen::VectorXd StateVector::getConsiderStates() const {
    return this->getConsider().getValues();
}

/*! Attach the state transition matrix of your state for simultaneous propagation
   @param Eigen::MatrixXd stm
*/
void StateVector::attachStm(const Eigen::MatrixXd& stm){
    this->stm = stm;
}

/*! Detach the state transition matrix of your state for simultaneous propagation
   @return Eigen::MatrixXd stm
*/
Eigen::MatrixXd StateVector::detachStm() const{
    return this->stm;
}
