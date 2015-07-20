
#ifndef ORB_ELEM_CONVERT_H
#define ORB_ELEM_CONVERT_H

#include <vector>
#include "utilities/sys_model.h"
#include "utilities/orbitalMotion.h"

class OrbElemConvert: public SysModel {
public:
   OrbElemConvert();
   ~OrbElemConvert();
 
   void SelfInit();
   void CrossInit();
   void UpdateState(uint64_t CurrentSimNanos);
   void WriteOutputMessages(uint64_t CurrentClock);
   void Elements2Cartesian();
   void Cartesian2Elements();
   void ReadInputs();
       
public:
   double r_N[3];                    // m  Current position vector (inertial)
   double v_N[3];                    // m/s Current velocity vector (inertial)
   double mu;                        // -- Current grav param (inertial)
   classicElements CurrentElem; // -- Current orbital elements 
   std::string StateString;          // -- port to use for conversion
   std::string OutputDataString;     // -- port to use for output data
   uint64_t OutputBufferCount;       // -- Count on number of buffers to output
   bool ReinitSelf;                  // -- Indicator to reset conversion type
   bool Elements2Cart;               // -- Flag saying which direction to go

private:
   int64_t StateInMsgID;              // -- MEssage ID for incoming data
   int64_t StateOutMsgID;             // -- Message ID for outgoing data
};

#endif
