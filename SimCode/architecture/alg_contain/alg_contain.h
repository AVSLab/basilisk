
#ifndef ALG_CONTAIN_H
#define ALG_CONTAIN_H

#include "utilities/sys_model.h"

typedef void (*AlgPtr)(void*, uint64_t);
typedef void (*AlgUpdatePtr)(void*, uint64_t, uint64_t);

class AlgContain: public SysModel {
public:
    AlgContain();
    ~AlgContain();
    AlgContain(void *DataIn, void(*UpPtr) (void*, uint64_t, uint64_t),
        void (*SelfPtr)(void*, uint64_t)=NULL,
        void (*CrossPtr)(void*, uint64_t)=NULL);
    
    void UseData(void *IncomingData) {DataPtr = IncomingData;}
    void UseUpdate(void (*LocPtr)(void*, uint64_t, uint64_t)) {AlgUpdate = LocPtr;}
    void UseSelfInit(void (*LocPtr)(void*, uint64_t)) {AlgSelfInit = LocPtr;}
    void UseCrossInit(void (*LocPtr)(void*, uint64_t)) {AlgCrossInit = LocPtr;}
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
