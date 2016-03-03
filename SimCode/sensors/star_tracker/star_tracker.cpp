/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/
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
