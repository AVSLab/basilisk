/*
 ISC License

 Copyright (c) 2024, Astroscale Japan

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

#ifndef TYPES_TEST_MESSAGE_H
#define TYPES_TEST_MESSAGE_H

#include <stdint.h>

#define TYPES_TEST_ARRAY_SIZE 3

/*! @brief Structure used to test the handling of various built-in types in message payloads across SWIG C/Python bridge */
typedef struct {
    // Scalars
    int8_t i8Test;
    uint8_t ui8Test;
    int16_t i16Test;
    uint16_t ui16Test;
    int32_t i32Test;
    uint32_t ui32Test;
    int64_t i64Test;
    uint64_t ui64Test;
    float f32Test;
    double f64Test;

    // 1D Arrays, integer
    int16_t i16TestArray[TYPES_TEST_ARRAY_SIZE];
    uint16_t ui16TestArray[TYPES_TEST_ARRAY_SIZE];
    int32_t i32TestArray[TYPES_TEST_ARRAY_SIZE];
    uint32_t ui32TestArray[TYPES_TEST_ARRAY_SIZE];
    int64_t i64TestArray[TYPES_TEST_ARRAY_SIZE];
    uint64_t ui64TestArray[TYPES_TEST_ARRAY_SIZE];

    // 2D Arrays, integer
    int16_t i16TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];
    uint16_t ui16TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];
    int32_t i32TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];
    uint32_t ui32TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];
    int64_t i64TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];
    uint64_t ui64TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];

    // 1D Arrays, floating point
    float f32TestArray[TYPES_TEST_ARRAY_SIZE];
    double f64TestArray[TYPES_TEST_ARRAY_SIZE];

    // 2D Arrays, floating point
    float f32TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];
    double f64TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];
} TypesTestMsgPayload;

#endif
