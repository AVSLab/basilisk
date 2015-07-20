
#ifndef STAR_TRACKER_H
#define STAR_TRACKER_H

#include <vector>
#include "utilities/sys_model.h"

class StarTracker: public SysModel {
public:
   StarTracker();
   ~StarTracker();
 
   bool LinkMessages();
   void UpdateState(uint64_t CurrentSimNanos);
       
public:
   std::vector<double> NoiseSigma;   // r  Standard deviation on noise (3vec)
   std::vector<double> WalkBound;    // r  Random walk bound (3vec) 
   std::vector<double> WalkTau;      // s  Time constant for random walk (3vec)
   std::vector<double> QStr2STCase;  // -- Quaternion from struct to case (4vec)
   std::vector<double> QMisalign;    // -- Quaternion for misalignment

   std::vector<double> ErrorVec;     // r  Current error vector to append

   std::vector<double> QuatOut;      // -- Quaternion output for tracker
   double SensorTimeTag;             // s Current time tag for sensor out
   std::string InputTimeMessage;     // -- String for time input msg
   bool MessagesLinked;              // -- Indicator for whether inputs bound
 private:
   int64_t InputTimeID;              // -- Connect to input time message
};

#endif
