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
#include "architecture/utilities/bskLogging.h"

/*! @brief state data class*/
class StateData {
public:
    Eigen::MatrixXd state;                        //!< [-] State value storage
    Eigen::MatrixXd stateDeriv;                   //!< [-] State derivative value storage
    std::string stateName;                        //!< [-] Name of the state
    bool stateEnabled;                            //!< [-] Flag indicating state is enabled
    BSKLogger bskLogger;                          //!< -- BSK Logging

public:
    StateData();
    StateData(std::string inName, const Eigen::MatrixXd & newState);  //!< class method
    StateData(const StateData &inState);                //!< class method
    ~StateData();
    void setState(const Eigen::MatrixXd & newState);    //!< class method
    void propagateState(double dt);                     //!< class method
    void setDerivative(const Eigen::MatrixXd & newDeriv);   //!< class method
    Eigen::MatrixXd getState() const {return state;}    //!< class method
    Eigen::MatrixXd getStateDeriv() const {return stateDeriv;}  //!< class method
    std::string getName() const {return stateName;}     //!< class method
    uint32_t getRowSize() const {return((uint32_t)state.innerSize());}  //!< class method
    uint32_t getColumnSize() const {return((uint32_t)state.outerSize());}   //!< class method
    bool isStateActive() {return stateEnabled;}         //!< class method
    void disable() {stateEnabled = false;}              //!< class method
    void enable() {stateEnabled = true;}                //!< class method
    void scaleState(double scaleFactor);                //!< class method

    StateData operator+ (const StateData & operand);    //!< class method
    StateData operator* (double scaleFactor);           //!< class method

};


#endif /* STATE_DATA_H */
