
#ifndef _GaussMarkov_HH_
#define _GaussMarkov_HH_

#include <string>
#include <stdint.h>
#include <vector>
#include <random>

class GaussMarkov
{
    
public:
    GaussMarkov();
    ~GaussMarkov();
    void computeNextState();
    void setRNGSeed(uint64_t newSeed) {rGen.seed(newSeed); RNGSeed = newSeed;}
    std::vector<double> getCurrentState() {return(currentState);}
    void setUpperBounds(std::vector<double> newBounds){stateBounds = newBounds;}
    void setNoiseMatrix(std::vector<double> noise){noiseMatrix = noise;}
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


#endif /* _GaussMarkov_HH_ */
