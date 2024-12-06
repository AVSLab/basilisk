#
#  ISC License
#
#  Copyright (c) 2024, Astroscale Japan
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

#
# Purpose:  Test the handling of built in types for a C message payload
# Author:   Sasawat Prankprakma (Astroscale Japan)
# Creation Date: 2024-04-17
#


from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging


def test_cMsgTypes():
    """
    Test the Python-side types of various message payload C types
    """

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    test = messaging.TypesTestMsgPayload()

    assert type(test.i8Test) is int
    assert type(test.ui8Test) is int
    assert type(test.i16Test) is int
    assert type(test.ui16Test) is int
    assert type(test.i32Test) is int
    assert type(test.ui32Test) is int
    assert type(test.i64Test) is int
    assert type(test.ui64Test) is int
    assert type(test.f32Test) is float
    assert type(test.f64Test) is float
    assert type(test.uint8Test) is int
    assert type(test.ucharTest) is int
    assert type(test.int8Test) is int
    assert type(test.scharTest) is int
    assert type(test.boolTest) is bool

    assert type(test.i16TestArray[0]) is int
    assert type(test.ui16TestArray[0]) is int
    assert type(test.i32TestArray[0]) is int
    assert type(test.ui32TestArray[0]) is int
    assert type(test.i64TestArray[0]) is int
    assert type(test.ui64TestArray[0]) is int
    assert type(test.f32TestArray[0]) is float
    assert type(test.f64TestArray[0]) is float

    assert type(test.i16TestArray2[0][0]) is int
    assert type(test.ui16TestArray2[0][0]) is int
    assert type(test.i32TestArray2[0][0]) is int
    assert type(test.ui32TestArray2[0][0]) is int
    assert type(test.i64TestArray2[0][0]) is int
    assert type(test.ui64TestArray2[0][0]) is int
    assert type(test.f32TestArray2[0][0]) is float
    assert type(test.f64TestArray2[0][0]) is float

    assert type(test.uint8TestArray2[0][0]) is int
    assert type(test.ucharTestArray2[0][0]) is int
    assert type(test.int8TestArray2[0][0]) is int
    assert type(test.scharTestArray2[0][0]) is int
    assert type(test.boolTestArray2[0][0]) is bool

    assert len(test.i16TestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.ui16TestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.i32TestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.ui32TestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.i64TestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.ui64TestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.f32TestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.f64TestArray) == messaging.TYPES_TEST_ARRAY_SIZE

    assert len(test.i16TestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.ui16TestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.i32TestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.ui32TestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.i64TestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.ui64TestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.f32TestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.f64TestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE

    assert len(test.uint8TestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.ucharTestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.int8TestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.scharTestArray) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.boolTestArray) == messaging.TYPES_TEST_ARRAY_SIZE

    assert len(test.uint8TestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.ucharTestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.int8TestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.scharTestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE
    assert len(test.boolTestArray2[0]) == messaging.TYPES_TEST_ARRAY_SIZE


if __name__ == "__main__":
    test_cMsgTypes()
