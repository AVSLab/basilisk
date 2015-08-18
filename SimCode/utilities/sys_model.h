
#ifndef _SysModel_HH_
#define _SysModel_HH_

#include <string>
#include <stdint.h>

class SysModel
{

 public:
   SysModel();
   virtual ~SysModel();
   virtual void SelfInit();
   virtual void CrossInit();
   virtual void IntegratedInit();
   virtual void UpdateState(uint64_t CurrentSimNanos);

 public:
   std::string ModelTag;        /// -- name for the algorithm to base off of
   uint64_t CallCounts=0;       /// -- Counts on the model being called
   uint32_t RNGSeed;            /// -- Giving everyone a random seed for ease of MC
};


#endif /* _SYS_MODEL_H_ */
