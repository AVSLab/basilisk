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
    int8_t i8Test;      //!< Test variable of int8_t
    uint8_t ui8Test;    //!< Test variable of uint8_t
    int16_t i16Test;    //!< Test variable of int16_t
    uint16_t ui16Test;  //!< Test variable of uint16_t
    int32_t i32Test;    //!< Test variable of int32_t
    uint32_t ui32Test;  //!< Test variable of uint32_t
    int64_t i64Test;    //!< Test variable of int64_t
    uint64_t ui64Test;  //!< Test variable of uint64_t
    float f32Test;      //!< Test variable of float
    double f64Test;     //!< Test variable of double
    uint8_t uint8Test;      //!< Test variable of uint8_t (alias)
    unsigned char ucharTest; //!< Test variable of unsigned char
    int8_t int8Test;        //!< Test variable of int8_t (alias)
    signed char scharTest;   //!< Test variable of signed char
    bool boolTest;          //!< Test variable of bool

    // 1D Arrays, integer
    int16_t i16TestArray[TYPES_TEST_ARRAY_SIZE];    //!< Test variable of array of int16_t
    uint16_t ui16TestArray[TYPES_TEST_ARRAY_SIZE];  //!< Test variable of array of uint16_t
    int32_t i32TestArray[TYPES_TEST_ARRAY_SIZE];    //!< Test variable of array of int32_t
    uint32_t ui32TestArray[TYPES_TEST_ARRAY_SIZE];  //!< Test variable of array of uint32_t
    int64_t i64TestArray[TYPES_TEST_ARRAY_SIZE];    //!< Test variable of array of int64_t
    uint64_t ui64TestArray[TYPES_TEST_ARRAY_SIZE];  //!< Test variable of array of uint64_t
    uint8_t uint8TestArray[TYPES_TEST_ARRAY_SIZE];      //!< Array test of uint8_t
    unsigned char ucharTestArray[TYPES_TEST_ARRAY_SIZE]; //!< Array test of unsigned char
    int8_t int8TestArray[TYPES_TEST_ARRAY_SIZE];        //!< Array test of int8_t
    signed char scharTestArray[TYPES_TEST_ARRAY_SIZE];   //!< Array test of signed char
    bool boolTestArray[TYPES_TEST_ARRAY_SIZE];          //!< Array test of bool

    // 2D Arrays, integer
    int16_t i16TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];    //!< Test variable of 2D array of int16_t
    uint16_t ui16TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];  //!< Test variable of 2D array of uint16_t
    int32_t i32TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];    //!< Test variable of 2D array of int32_t
    uint32_t ui32TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];  //!< Test variable of 2D array of uint32_t
    int64_t i64TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];    //!< Test variable of 2D array of int64_t
    uint64_t ui64TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];  //!< Test variable of 2D array of uint64_t
    uint8_t uint8TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];      //!< 2D Array test of uint8_t
    unsigned char ucharTestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE]; //!< 2D Array test of unsigned char
    int8_t int8TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];        //!< 2D Array test of int8_t
    signed char scharTestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];   //!< 2D Array test of signed char
    bool boolTestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];          //!< 2D Array test of bool

    // 1D Arrays, floating point
    float f32TestArray[TYPES_TEST_ARRAY_SIZE];   //!< Test variable of array of float
    double f64TestArray[TYPES_TEST_ARRAY_SIZE];  //!< Test variable of array of double

    // 2D Arrays, floating point
    float f32TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];   //!< Test variable of 2D array of float
    double f64TestArray2[TYPES_TEST_ARRAY_SIZE][TYPES_TEST_ARRAY_SIZE];  //!< Test variable of 2D array of double
} TypesTestMsgPayload;

#endif
