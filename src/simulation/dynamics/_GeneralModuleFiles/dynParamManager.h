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


#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <stdint.h>
#include <map>
#include <vector>
#include <Eigen/Dense>
#include "stateData.h"

class StateVector {
public:
    std::map<std::string, StateData> stateMap;
public:
    StateVector operator+(const StateVector& operand);
    StateVector operator*(double scaleFactor);
};

/*! Manager of states for Basilisk dynamical systems.  Allows the state-
           effector models of a dynamic object to create, get, and update states 
           present in the model.*/
class DynParamManager {
public:
    std::map<std::string, Eigen::MatrixXd> dynProperties;
    StateVector stateContainer;
public:
    DynParamManager();
    ~DynParamManager();
    StateData* registerState(uint32_t nRow, uint32_t nCol, std::string stateName);
    StateData* getStateObject(std::string stateName);
    StateVector getStateVector();
    void updateStateVector(const StateVector & newState);
    void propagateStateVector(double dt);
    Eigen::MatrixXd* createProperty(std::string propName,
                                    const Eigen::MatrixXd & propValue);
    Eigen::MatrixXd* getPropertyReference(std::string propName);
    void setPropertyValue(const std::string propName,
                          const Eigen::MatrixXd & propValue);
    
};

#endif /* STATE_MANAGER_H */
