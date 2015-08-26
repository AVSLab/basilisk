#include "sensors/star_tracker/star_tracker.h"
#include "architecture/messaging/system_messaging.h"
#include "environment/spice/spice_interface.h"

StarTracker::StarTracker()
{
    CallCounts = 0;
    MessagesLinked = false;
    InputTimeID = -1;
    InputTimeMessage = "spice_time_output_data";
    return;
}

StarTracker::~StarTracker()
{
    return;
}

bool StarTracker::LinkMessages()
{
    int64_t LocalID = SystemMessaging::GetInstance()->FindMessageID(
                                                                    InputTimeMessage);
    if(LocalID >= 0)
    {
        InputTimeID = LocalID;
        return(true);
    }
    return(false);
}

void StarTracker::UpdateState(uint64_t CurrentSimNanos)
{
    std::vector<double>::iterator it;
    
    if(!MessagesLinked)
    {
        MessagesLinked = LinkMessages();
    }
    
    for(it = NoiseSigma.begin(); it != NoiseSigma.end(); it++)
    {
        *it = *it*2.0;
    }
    if(MessagesLinked)
    {
        SpiceTimeOutput TimeMessage;
        SingleMessageHeader LocalHeader;
        SystemMessaging::GetInstance()->ReadMessage(InputTimeID, &LocalHeader,
                                                    sizeof(SpiceTimeOutput), reinterpret_cast<uint8_t *> (&TimeMessage));
        SensorTimeTag = TimeMessage.J2000Current;
    }
}
