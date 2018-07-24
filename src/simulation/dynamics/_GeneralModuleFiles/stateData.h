/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef STATE_DATA_H
#define STATE_DATA_H
#include <Eigen/Dense>
#include <stdint.h>

/*! @brief Object that is to be used by an integrator. It's basically an interface with only one method: the F function describing a dynamic model X_dot = F(X,t)
 */
class StateData {
public:
    Eigen::MatrixXd state;                        //! [-] State value storage
    Eigen::MatrixXd stateDeriv;                   //! [-] State derivative value storage
    std::string stateName;                        //! [-] Name of the state
    bool stateEnabled;                            //! [-] Flag indicating state is enabled

public:
    StateData();
    StateData(std::string inName, const Eigen::MatrixXd & newState);
    StateData(const StateData &inState);
    ~StateData();
    void setState(const Eigen::MatrixXd & newState);
    void propagateState(double dt);
    void setDerivative(const Eigen::MatrixXd & newDeriv);
    Eigen::MatrixXd getState() const {return state;}
    Eigen::MatrixXd getStateDeriv() const {return stateDeriv;}
    std::string getName() const {return stateName;}
    uint32_t getRowSize() const {return((uint32_t)state.innerSize());}
    uint32_t getColumnSize() const {return((uint32_t)state.outerSize());}
    bool isStateActive() {return stateEnabled;}
    void disable() {stateEnabled = false;}
    void enable() {stateEnabled = true;}
    void scaleState(double scaleFactor);

    StateData operator+ (const StateData & operand);
    StateData operator* (double scaleFactor);
    
};

#endif /* STATE_DATA_H */
