
#include "utilities/sys_model.h"
#include "architecture/messaging/system_messaging.h"
SysModel::SysModel()
{
    ModelTag = "";
    RNGSeed = 0x1badcad1;
    moduleID = SystemMessaging::GetInstance()->checkoutModuleID();
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
