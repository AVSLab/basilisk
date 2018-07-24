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

#ifndef _Saturate_HH_
#define _Saturate_HH_

#include <stdint.h>
#include <Eigen/Dense>

/*! \addtogroup Sim Utility Group
 *  This group contains the simulation utilities that are used globally on the 
 *  simulation side of the software.  Note that FSW should not generally use  
 *  these utilities once we reach a CDR level of maturity on a project.
 * @{
 */

/*! This module is used to apply a second-order bounded Gauss-Markov random walk 
    on top of an upper level process.  The intent is that the caller will perform 
    the set methods (setUpperBounds, setNoiseMatrix, setPropMatrix) as often as 
    they need to, call computeNextState, and then call getCurrentState cyclically
*/
class Saturate
{
    
public:
    Saturate();
    Saturate(uint64_t size);
    ~Saturate();
    void setBounds(Eigen::MatrixXd bounds) {this->stateBounds = bounds;}
    /*!@brief sets upper and lower bounds for each state
       @param bounds. one row for each state. lower bounds in left column, upper in right column
       @return void*/
    Eigen::VectorXd saturate(Eigen::VectorXd unsaturatedStates);
    /*!@brief Saturates the given unsaturated states
       @param unsaturated States, a vector of the unsaturated states
       @return saturatedStates*/
    
private:
    uint64_t numStates;             //!< -- Number of states to generate noise for
    Eigen::MatrixXd stateBounds;    //!< -- one row for each state. lower bounds in left column, upper in right column
};

/*! @} */

#endif /* _saturate_HH_ */
