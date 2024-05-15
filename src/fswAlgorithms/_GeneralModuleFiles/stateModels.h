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
    PositionState position;
    VelocityState velocity;
    AccelerationState acceleration;
    BiasState bias;
    ConsiderState considerParameters;
    Eigen::MatrixXd stm;

public:
    StateVector();
    ~StateVector();

    size_t size() const;
    StateVector add(const StateVector &vector) const;
    StateVector scale(const double scalar) const;

    void setPositionStates(const Eigen::VectorXd& positionComponents);
    Eigen::VectorXd getPositionStates() const;
    void setVelocityStates(const Eigen::VectorXd& velocityComponents);
    Eigen::VectorXd getVelocityStates() const;
    void setAccelerationStates(const Eigen::VectorXd& accelerationComponents);
    Eigen::VectorXd getAccelerationStates() const;
    void setBiasStates(const Eigen::VectorXd& biasComponents);
    Eigen::VectorXd getBiasStates() const;
    void setConsiderStates(const Eigen::VectorXd& considerComponents);
    Eigen::VectorXd getConsiderStates() const;

    void attachSTM(const Eigen::MatrixXd& stm);
    Eigen::MatrixXd detatchSTM() const;
};

#endif
