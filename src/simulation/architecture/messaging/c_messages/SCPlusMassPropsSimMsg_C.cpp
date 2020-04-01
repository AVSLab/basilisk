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

/* All of the files in this folder (c_messages) are autocoded by the script GenCMessages.py.
The script checks for the line "INSTANTIATE_TEMPLATES" in the file message.i. This
ensures that if a c++ message is instantiated that we also have a C equivalent of that message.

If you need to edit the way that these types/functions are written,
edit the templates in /templates and run GenCMessages.py.
*/

#include "SCPlusMassPropsSimMsg_C.h"
#include "message.h"

void SCPlusMassPropsSimMsg_C_subscribe(SCPlusMassPropsSimMsg_C *subscriber, SCPlusMassPropsSimMsg_C *source) {
    subscriber->payloadPointer = &(source->payload);
};

void SCPlusMassPropsSimMsg_C_claim(SCPlusMassPropsSimMsg_C *coowner, SCPlusMassPropsSimMsg_C *data) {
    coowner->payloadPointer = &data->payload;
};

void SCPlusMassPropsSimMsg_C_write(SCPlusMassPropsSimMsg *data, SCPlusMassPropsSimMsg_C *destination) {
    *destination->payloadPointer = *data;
    return;
};

SCPlusMassPropsSimMsg SCPlusMassPropsSimMsg_C_read(SCPlusMassPropsSimMsg_C *source) {
    return *source->payloadPointer;
};

void SCPlusMassPropsSimMsg_cpp_subscribe(SCPlusMassPropsSimMsg_C *subscriber, void* source){
    SimMessage<SCPlusMassPropsSimMsg>* source_t = (SimMessage<SCPlusMassPropsSimMsg>*) source;
    subscriber->payloadPointer = source_t->subscribeRaw();
};