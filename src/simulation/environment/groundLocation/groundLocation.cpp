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

 */

#include "groundLocation.h"

/*! The constructor method initializes the dipole parameters to zero, resuling in a zero magnetic field result by default.
 @return void
 */
GroundLocation::GroundLocation()
{
    //! - Set the default atmospheric properties to yield a zero response


    return;
}

/*! Empty destructor method.
 @return void
 */
GroundLocation::~GroundLocation()
{
    return;
}

/*! Adds a scState message name to the vector of names to be subscribed to. Also creates a corresponding access message output name.
*/
GroundLocation::addSpacecraftToModel()
{
    std::string tmpAccessMsgName;
    this->scStateInMsgNames.push_back(tmpScMsgName);
        tmpAccessMsgName = this->ModelTag + "_" + std::to_string(this->scStateInMsgNames.size()-1) + "_access";
    this->accessOutMsgNames.push_back(tmpAccessMsgName);
    return;
}

GroundLocation::ReadMessages()
{

    return;
}

GroundLocation::WriteMessages()
{

    return;
}

GroundLocation::SelfInit()
{
    return;
}

GroundLocation::CrossInit()
{
    return;
}

GroundLocation::updateInertialPosition()
{

    return;
}

GroundLocation::computeRelativePosition()
{

    return;
}

GroundLocation::computeAccess()
{
    return
}

GroundLocation::UpdateState()
{
    this->ReadMessages();

    this->computeAccess();

    this->WriteMessages();

    return

}