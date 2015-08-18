
#ifndef ALG_CONTAIN_H
#define ALG_CONTAIN_H

#include "utilities/sys_model.h"

typedef void (*AlgPtr)(void*);
typedef void (*AlgUpdatePtr)(void*, uint64_t);

class AlgContain: public SysModel {
public:
   AlgContain();
   ~AlgContain();
   AlgContain(void *DataIn, void(*UpPtr) (void*, uint64_t), void (*SelfPtr)(void*)=NULL, 
      void (*CrossPtr)(void*)=NULL);
   
   void UseData(void *IncomingData) {DataPtr = IncomingData;}
   void UseUpdate(void (*LocPtr)(void*, uint64_t)) {AlgUpdate = LocPtr;}
   void UseSelfInit(void (*LocPtr)(void*)) {AlgSelfInit = LocPtr;}
   void UseCrossInit(void (*LocPtr)(void*)) {AlgCrossInit = LocPtr;}
   void CrossInit();
   void SelfInit();
   void UpdateState(uint64_t CurrentSimNanos);
       
private:
   void *DataPtr;
   AlgPtr AlgSelfInit;
   AlgPtr AlgCrossInit;
   AlgUpdatePtr AlgUpdate;
};

#endif
