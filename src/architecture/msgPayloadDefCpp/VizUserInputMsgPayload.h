/*
 ISC License

 Copyright (c) 2013, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include <string>
#include <vector>

#ifndef VIZ_USER_INPUT_H
#define VIZ_USER_INPUT_H

/*! @brief Structure used to contain a single EventDialog panel response */
typedef struct
//@cond DOXYGEN_IGNORE
EventReply
//@endcond
{
    std::string eventHandlerID;    //!< Name provided when setting up the EventDialog object
    std::string reply;             //!< Option selection
    bool eventHandlerDestroyed;    //!< Was the panel closed and destroyed?

}EventReply;

/*! @brief Structure used to contain all keyboard inputs and EventReply objects recorded during the last timestep */
typedef struct
//@cond DOXYGEN_IGNORE
VizUserInputMsgPayload
//@endcond
{
    int frameNumber;               //!< Vizard frame number
    std::string keyboardInput;     //!< String containing all keyboard inputs since last update.
    std::vector<EventReply> eventReplies;  //!< Contains all panel inputs since last update

}VizUserInputMsgPayload;

#endif
