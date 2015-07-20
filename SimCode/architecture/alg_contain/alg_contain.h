
#ifndef ALG_CONTAIN_H
#define ALG_CONTAIN_H

#include "utilities/sys_model.h"

typedef void (*AlgPtr)(void*);

class AlgContain: public SysModel {
public:
   AlgContain();
   ~AlgContain();
   AlgContain(void *DataIn, AlgPtr UpdateIn, AlgPtr SelfInitIn=NULL, 
      AlgPtr CrossInitIn=NULL);
   
   void UseData(void *IncomingData) {DataPtr = IncomingData;}
   void UseUpdate(void (*LocPtr)(void*)) {AlgUpdate = LocPtr;}
   void UseSelfInit(void (*LocPtr)(void*)) {AlgSelfInit = LocPtr;}
   void UseCrossInit(void (*LocPtr)(void*)) {AlgCrossInit = LocPtr;}
   void CrossInit();
   void SelfInit();
   void UpdateState(uint64_t CurrentSimNanos);
       
private:
   void *DataPtr;
   AlgPtr AlgSelfInit;
   AlgPtr AlgCrossInit;
   AlgPtr AlgUpdate;
};

#endif
