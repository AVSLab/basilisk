/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU, Norway

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

// AntennaDefinitions.h (new file in same directory)
#ifndef ANTENNA_DEFINITIONS_H
#define ANTENNA_DEFINITIONS_H

namespace AntennaTypes {
    enum EnvironmentType {
        ENVIRONMENT_SPACE    = 0,
        ENVIRONMENT_EARTH    = 1,
        _ENVIRONMENT_UNKNOWN = -1
    };
    enum AntennaStateEnum {
        ANTENNA_OFF            = 0,
        ANTENNA_RX             = 1,
        ANTENNA_TX             = 2,
        ANTENNA_RXTX           = 3,
        _ANTENNA_STATE_UNKNOWN = -1
    };
} // namespace AntennaTypes
#endif /* ANTENNA_DEFINITIONS_H */
