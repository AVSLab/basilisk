
#include "utilities/sys_model.h"
SysModel::SysModel()
{
   ModelTag = "";
   RNGSeed = 0x1badcad1;
}

SysModel::~SysModel()
{
}

void SysModel :: SelfInit()
{
  return;
}

void SysModel :: CrossInit()
{
  return;
}

void SysModel :: IntegratedInit()
{
   return;
}

void SysModel :: UpdateState(uint64_t CurrentSimNanos)
{
   return;
}
