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


#ifndef FILTER_STATE_MODELS_H
#define FILTER_STATE_MODELS_H

#include <Eigen/Core>

/*! @brief State class */
class State{
private:
    Eigen::VectorXd values;
public:
    State();
    ~State();
    void setValues(const Eigen::VectorXd& componentValues);
    Eigen::VectorXd getValues() const;
};

class PositionState : public State{};
class VelocityState : public State{};
class AccelerationState : public State{};
class BiasState : public State{};
class ConsiderState : public State{};


/*! @brief State models used to map a state vector to a measurement */
class StateVector{
private:
    std::optional<PositionState> position;
    std::optional<VelocityState> velocity;
    std::optional<AccelerationState> acceleration;
    std::optional<BiasState> bias;
    std::optional<ConsiderState> considerParameters;
    Eigen::MatrixXd stm;

public:
    StateVector();
    ~StateVector();

    long size() const;
    StateVector add(const StateVector &vector) const;
    StateVector addVector(const Eigen::VectorXd &vector) const;
    StateVector scale(const double scalar) const;
    Eigen::VectorXd returnValues() const;

    void setPosition(const PositionState &position);
    PositionState getPosition() const;
    Eigen::VectorXd getPositionStates() const;
    bool hasPosition() const;
    void setVelocity(const VelocityState &velocity);
    VelocityState getVelocity() const;
    Eigen::VectorXd getVelocityStates() const;
    bool hasVelocity() const;
    void setAcceleration(const AccelerationState &acceleration);
    AccelerationState getAcceleration() const;
    Eigen::VectorXd getAccelerationStates() const;
    bool hasAcceleration() const;
    void setBias(const BiasState &bias);
    BiasState getBias() const;
    Eigen::VectorXd getBiasStates() const;
    bool hasBias() const;
    void setConsider(const ConsiderState &consider);
    ConsiderState getConsider() const;
    Eigen::VectorXd getConsiderStates() const;
    bool hasConsider() const;

    void attachStm(const Eigen::MatrixXd& stm);
    Eigen::MatrixXd detachStm() const;



};

#endif
