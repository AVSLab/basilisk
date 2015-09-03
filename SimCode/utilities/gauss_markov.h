
#ifndef _GaussMarkov_HH_
#define _GaussMarkov_HH_

#include <string>
#include <stdint.h>
#include <vector>
#include <random>
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
class GaussMarkov
{
    
public:
    GaussMarkov();
    ~GaussMarkov();
    void computeNextState();
    /*!@brief Method does just what it says, seeds the random number generator
       @param newSeed The seed to use in the random number generator
       @return void*/
    void setRNGSeed(uint64_t newSeed) {rGen.seed(newSeed); RNGSeed = newSeed;}
    /*!@brief Method returns the current random walk state from model
       @return The private currentState which is the vector of random walk values*/
    std::vector<double> getCurrentState() {return(currentState);}
    /*!@brief Set the upper bounds on the random walk to newBounds
       @param newBounds the bounds to put on the random walk states
       @return void*/
    void setUpperBounds(std::vector<double> newBounds){stateBounds = newBounds;}
    /*!@brief Set the noiseMatrix that is used to define error sigmas
       @param noise The new value to use for the noiseMatrix variable (error sigmas)
       @return void*/
    void setNoiseMatrix(std::vector<double> noise){noiseMatrix = noise;}
    /*!@brief Set the propagation matrix that is used to propagate the state.
       @param prop The new value for the state propagation matrix
       @return void*/
    void setPropMatrix(std::vector<double> prop){propMatrix = prop;}
    
private:
    uint64_t RNGSeed;                 //!< -- Seed for random number generator
    std::vector<double> stateBounds;  //!< -- Upper bounds to use for markov
    std::vector<double> currentState;  //!< -- State of the markov model
    std::vector<double> propMatrix;    //!< -- Matrix to propagate error state with
    std::vector<double> noiseMatrix;   //!< -- covariance matrix to apply errors with
    std::minstd_rand rGen; //!< -- Random number generator for model
    std::normal_distribution<double> rNum;  //!< -- Random number distribution for model
};

/*! @} */

#endif /* _GaussMarkov_HH_ */
