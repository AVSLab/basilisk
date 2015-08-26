#include "architecture/alg_contain/alg_contain.h"

AlgContain::AlgContain()
{
    AlgSelfInit = NULL;
    AlgCrossInit = NULL;
    AlgUpdate = NULL;
    DataPtr = NULL;
    CallCounts = 0;
    return;
}

AlgContain::AlgContain(void *DataIn, void(*UpPtr) (void*, uint64_t),
                       void (*SelfPtr)(void*), void (*CrossPtr)(void*))
{
    DataPtr = DataIn;
    AlgSelfInit = SelfPtr;
    AlgCrossInit = CrossPtr;
    AlgUpdate = UpPtr;
}

AlgContain::~AlgContain()
{
    return;
}

void AlgContain::SelfInit()
{
    if(AlgSelfInit != NULL)
    {
        AlgSelfInit(DataPtr);
    }
}

void AlgContain::CrossInit()
{
    if(AlgCrossInit != NULL)
    {
        AlgCrossInit(DataPtr);
    }
}

void AlgContain::UpdateState(uint64_t CurrentSimNanos)
{
    if(AlgUpdate != NULL)
    {
        AlgUpdate(DataPtr, CurrentSimNanos);
    }
}
