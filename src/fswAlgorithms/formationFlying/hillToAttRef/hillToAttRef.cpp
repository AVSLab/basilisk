/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

/* modify the path to reflect the new module names */
#include "hillToAttRef.h"


/*! The constructor for the HoughCircles module. It also sets some default values at its creation.  */
HillToAttRef::HillToAttRef()
{
    this->
}

/*! Selfinit performs the first stage of initialization for this module.
 It's primary function is to create messages that will be written to.
 @return void
 */
void HillToAttRef::SelfInit()
{
    /*! - Create output message for module */
    this->attRefOutMsgId= SystemMessaging::GetInstance()->CreateNewMessage(this->attRefOutMsgName,sizeof(AttRefFswMsg),2,"AttRefFswMsg",this->moduleID);
}


/*! CrossInit performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 */
void HillToAttRef::CrossInit()
{
    /*! - Get the image data message ID*/
    this->imageInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->imageInMsgName,sizeof(CameraImageMsg), moduleID);
}

/*! This is the destructor */
HillToAttRef::~HillToAttRef()
{
    return;
}


/*! This method performs a complete reset of the module.  Local module variables that retain time varying states between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void HillToAttRef::Reset(uint64_t CurrentSimNanos)
{
    return;
}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding. 
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void HillToAttRef::UpdateState(uint64_t CurrentSimNanos)
{
    //  Read in the relative state and chief attitude messages


    //  Apply the gain matrix to get a relative attitude

    //  Combine the relative attitude with the chief inertial attitude to get the reference attitude

    //  Write the reference message
    SystemMessaging::GetInstance()->WriteMessage(this->opnavCirclesOutMsgID, CurrentSimNanos, sizeof(CirclesOpNavMsg), reinterpret_cast<uint8_t *>(&circleBuffer), this->moduleID);

    return;
}

