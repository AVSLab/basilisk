#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import sys

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import thrForceMapping
from Basilisk.simulation import starTracker


ARRAY_FIELDS = [
    "i16TestArray2", "ui16TestArray2", "i32TestArray2", "ui32TestArray2",
    "i64TestArray2", "ui64TestArray2", "f32TestArray2", "f64TestArray2",
    "uint8TestArray2", "ucharTestArray2", "int8TestArray2", "scharTestArray2",
    "boolTestArray2",
]
ARRAY1_FIELDS = [field[:-1] for field in ARRAY_FIELDS]


class ConversionFailure:
    """Raise from conversions that support Python numeric protocols."""

    def __float__(self):
        raise RuntimeError("conversion failed")

    def __index__(self):
        raise RuntimeError("conversion failed")

    def __bool__(self):
        raise RuntimeError("conversion failed")


class SequenceFailure:
    """Raise while obtaining a sequence length or one of its items."""

    def __init__(self, operation):
        self.operation = operation

    def __len__(self):
        if self.operation == "length":
            raise RuntimeError("length failed")
        return 2

    def __getitem__(self, index):
        raise IndexError("item failed")


class ResizingList(list):
    """Change length on the first item access to exercise cached bounds."""

    def __init__(self, values, grow):
        super().__init__(values)
        self.grow = grow

    def __getitem__(self, index):
        value = super().__getitem__(index)
        if index == 0:
            if self.grow:
                self.append(value)
            else:
                self.pop()
        return value


@pytest.mark.parametrize("field", ARRAY1_FIELDS)
def test_array1_scalar_types(field):
    """Supported scalar arrays preserve full inputs and zero-pad short ones."""
    payload = messaging.TypesTestMsgPayload()
    values = [1, 0, 3]
    expected = list(map(bool, values)) if field == "boolTestArray" else values
    setattr(payload, field, values)
    assert getattr(payload, field) == expected
    setattr(payload, field, [1])
    assert getattr(payload, field) == [1, 0, 0]
    setattr(payload, field, [])
    assert getattr(payload, field) == [0, 0, 0]


@pytest.mark.parametrize("container", [list, tuple, np.array])
def test_array1_sequences(container):
    """Lists, tuples and NumPy arrays remain valid inputs."""
    payload = messaging.TypesTestMsgPayload()
    payload.f64TestArray = container([-1, 2])
    assert payload.f64TestArray == [-1, 2, 0]


@pytest.mark.parametrize("field", ARRAY1_FIELDS)
def test_array1_conversion_failure(field):
    """Numeric conversion errors preserve the destination and release items."""
    payload = messaging.TypesTestMsgPayload()
    setattr(payload, field, [1, 1, 1])
    value = ConversionFailure()
    values = [0, value]
    count = sys.getrefcount(value)
    error = TypeError if field.startswith(("ui", "uchar")) else RuntimeError
    with pytest.raises(error):
        setattr(payload, field, values)
    assert getattr(payload, field) == [1, 1, 1]
    assert sys.getrefcount(value) == count


@pytest.mark.parametrize("field,value,error", [
    ("f64TestArray", 10**1000, OverflowError),
    ("i64TestArray", 2**100, OverflowError),
    ("ui64TestArray", -1, OverflowError),
    ("f64TestArray", None, TypeError),
])
def test_array1_invalid_elements(field, value, error):
    """Overflow and invalid pointer elements cannot partially update arrays."""
    payload = messaging.TypesTestMsgPayload()
    setattr(payload, field, [9, 9, 9])
    with pytest.raises(error):
        setattr(payload, field, [0, value])
    assert getattr(payload, field) == [9, 9, 9]


def test_array1_invalid_pointer_reference():
    """Invalid pointer elements cannot modify arrays or leak references."""
    payload = messaging.TypesTestMsgPayload()
    payload.f64TestArray = [9, 9, 9]
    value = object()
    count = sys.getrefcount(value)
    with pytest.raises(TypeError):
        payload.f64TestArray = [0, value]
    assert payload.f64TestArray == [9, 9, 9]
    assert sys.getrefcount(value) == count


@pytest.mark.parametrize("values,error", [
    (None, ValueError),
    ([0] * 4, ValueError),
    (SequenceFailure("length"), RuntimeError),
    (SequenceFailure("item"), IndexError),
])
def test_array1_sequence_failure(values, error):
    """Sequence validation and access errors leave the destination intact."""
    payload = messaging.TypesTestMsgPayload()
    payload.f64TestArray = [9, 9, 9]
    with pytest.raises(error):
        payload.f64TestArray = values
    assert payload.f64TestArray == [9, 9, 9]


def test_array1_pointer_proxy():
    """Valid numeric pointer proxies work; null proxies are rejected."""
    class PointerProxy:
        def __init__(self, pointer):
            self.this = pointer

    pointer = messaging.new_doubleArray(1)
    try:
        messaging.doubleArray_setitem(pointer, 0, 3.5)
        payload = messaging.TypesTestMsgPayload()
        proxy = PointerProxy(pointer)
        payload.f64TestArray = [proxy]
        assert payload.f64TestArray == [3.5, 0, 0]
        count = sys.getrefcount(proxy)
        proxy.this = None
        with pytest.raises(TypeError):
            payload.f64TestArray = [0, proxy]
        assert payload.f64TestArray == [3.5, 0, 0]
        assert sys.getrefcount(proxy) == count
    finally:
        messaging.delete_doubleArray(pointer)


@pytest.mark.parametrize("grow", [False, True])
@pytest.mark.parametrize("struct_array", [False, True])
def test_array1_resizing_sequence(grow, struct_array):
    """Sequence resizing cannot bypass bounds or copy partial data."""
    if struct_array:
        payload = messaging.CSSConfigMsgPayload()
        field = "cssVals"
        value = messaging.CSSUnitConfigMsgPayload()
        value.CBias = 0.75  # [-]
        size = len(payload.cssVals)
    else:
        payload = messaging.TypesTestMsgPayload()
        field = "f64TestArray"
        value = 3.5
        size = 3
    values = ResizingList([value] * size, grow)
    if grow:
        setattr(payload, field, values)
    else:
        with pytest.raises(IndexError):
            setattr(payload, field, values)
    actual = getattr(payload, field)
    if struct_array:
        actual = [item.CBias for item in actual]
        value = value.CBias
    assert actual == ([value] * size if grow else [0] * size)


@pytest.mark.parametrize("failure,error", [
    ("length", RuntimeError),
    ("item", IndexError),
    ("invalid", TypeError),
    ("null", TypeError),
    ("oversized", ValueError),
    ("nonsequence", ValueError),
])
def test_struct_array_failure(failure, error):
    """Struct arrays share safe sequence handling and pointer cleanup."""
    payload = messaging.CSSConfigMsgPayload()
    css = messaging.CSSUnitConfigMsgPayload()
    css.CBias = 0.75  # [-]
    payload.cssVals = [css]
    initial = [item.CBias for item in payload.cssVals]
    invalid = object()
    inputs = {
        "length": SequenceFailure("length"),
        "item": SequenceFailure("item"),
        "invalid": [css, invalid],
        "null": [css, None],
        "oversized": [css] * (len(payload.cssVals) + 1),
        "nonsequence": None,
    }
    counts = [sys.getrefcount(item) for item in (css, invalid)]
    with pytest.raises(error):
        payload.cssVals = inputs[failure]
    assert [item.CBias for item in payload.cssVals] == initial
    assert [sys.getrefcount(item) for item in (css, invalid)] == counts


@pytest.mark.parametrize("field", ARRAY_FIELDS)
def test_array2_scalar_types(field):
    """All supported payload scalar types accept full and partial matrices."""
    payload = messaging.TypesTestMsgPayload()
    values = [[1, 2, 3], [4, 0, 6], [7, 8, 9]]
    expected = (values if field != "boolTestArray2"
                else [list(map(bool, row)) for row in values])
    setattr(payload, field, values)
    assert getattr(payload, field) == expected
    setattr(payload, field, [[1]])
    assert getattr(payload, field) == [[1, 0, 0], [0, 0, 0], [0, 0, 0]]


@pytest.mark.parametrize(
    "shape", [(3, 3), (3, 2), (2, 3), (2, 2), (0, 0), (2, 0)]
)
@pytest.mark.parametrize("container", [list, tuple, np.array])
def test_array2_sequences_and_padding(shape, container):
    """Lists, tuples and NumPy arrays preserve values and zero-pad unused cells."""
    rows, columns = shape
    values = np.arange(rows * columns).reshape(shape)
    matrix = container([container(row.tolist()) for row in values])
    payload = messaging.TypesTestMsgPayload()
    payload.f64TestArray2 = [[9] * 3 for _ in range(3)]
    payload.f64TestArray2 = matrix
    expected = np.zeros((3, 3))
    expected[:rows, :columns] = values
    np.testing.assert_array_equal(payload.f64TestArray2, expected)


@pytest.mark.parametrize("matrix", [
    None,
    [1],
    [[1, 2], None],
    [[1, 2, 3], [4]],
    [[1], [2, 3]],
    [[], [1]],
    [[1], []],
    [[1]] * 4,
    [[1, 2, 3, 4]],
    [[1], [2, 3, 4, 5]],
])
def test_array2_invalid_shapes(matrix):
    """Malformed and oversized matrices raise without modifying the payload."""
    payload = messaging.TypesTestMsgPayload()
    initial = [[9] * 3 for _ in range(3)]
    payload.f64TestArray2 = initial
    with pytest.raises(ValueError):
        payload.f64TestArray2 = matrix
    assert payload.f64TestArray2 == initial


@pytest.mark.parametrize("field", ARRAY_FIELDS)
def test_array2_conversion_failure(field):
    """Conversion errors propagate without mutation or leaked references."""
    payload = messaging.TypesTestMsgPayload()
    initial = [[1] * 3 for _ in range(3)]
    setattr(payload, field, initial)
    value = ConversionFailure()
    row = [0, value]
    matrix = [[0, 0], row]
    counts = [sys.getrefcount(obj) for obj in (matrix, row, value)]
    # Unsigned converters require a Python integer and do not call __index__.
    error = TypeError if field.startswith(("ui", "uchar")) else RuntimeError
    with pytest.raises(error):
        setattr(payload, field, matrix)
    assert getattr(payload, field) == initial
    assert [sys.getrefcount(obj) for obj in (matrix, row, value)] == counts


@pytest.mark.parametrize("field,value,error", [
    ("f64TestArray2", None, TypeError),
    ("f64TestArray2", "a", TypeError),
    ("i64TestArray2", None, TypeError),
    ("i64TestArray2", 2**100, OverflowError),
    ("ui64TestArray2", -1, OverflowError),
])
def test_array2_builtin_conversion_errors(field, value, error):
    """Built-in conversion failures leave existing values intact."""
    payload = messaging.TypesTestMsgPayload()
    initial = [[1] * 3 for _ in range(3)]
    setattr(payload, field, initial)
    with pytest.raises(error):
        setattr(payload, field, [[0, 0], [0, value]])
    assert getattr(payload, field) == initial


@pytest.mark.parametrize("field", ["i64TestArray2", "f64TestArray2"])
def test_array2_negative_one(field):
    """A valid converter result of minus one is not mistaken for an error."""
    payload = messaging.TypesTestMsgPayload()
    setattr(payload, field, [[-1]])
    assert getattr(payload, field)[0][0] == -1


@pytest.mark.parametrize(
    "operation,error", [("length", RuntimeError), ("item", IndexError)]
)
@pytest.mark.parametrize("as_row", [False, True])
def test_array2_sequence_failure(operation, error, as_row):
    """Sequence protocol failures propagate and release acquired references."""
    payload = messaging.TypesTestMsgPayload()
    initial = [[9] * 3 for _ in range(3)]
    payload.f64TestArray2 = initial
    sequence = SequenceFailure(operation)
    matrix = [[1, 2], sequence] if as_row else sequence
    count = sys.getrefcount(sequence)
    with pytest.raises(error, match=f"{operation} failed"):
        payload.f64TestArray2 = matrix
    assert payload.f64TestArray2 == initial
    assert sys.getrefcount(sequence) == count


def test_array2_module_members_and_arguments():
    """Check module members and rectangular function arguments too."""
    tracker = starTracker.StarTracker()
    initial = np.eye(3).tolist()
    tracker.dcm_CB = initial
    with pytest.raises(ValueError):
        tracker.dcm_CB = [[1, 0, 0], [0]]
    assert tracker.dcm_CB == initial

    mapping = thrForceMapping.thrForceMapping()
    locations = [[1, 2, 3], [4, 5, 6]]  # [m]
    mapping.rThruster_B = locations
    expected = locations + [[0, 0, 0]] * (len(mapping.rThruster_B) - 2)
    assert mapping.rThruster_B == expected
    with pytest.raises(ValueError):
        mapping.rThruster_B = [[1, 2, 3, 4]]  # [m]
    assert mapping.rThruster_B == expected

    # No requested torque: exercise the 3-by-MAX_EFF_CNT argument conversion.
    assert thrForceMapping.computeTorqueAngErr(
        [[0]], [0, 0, 0], 0, 0.0, [], []
    ) == 0.0
    with pytest.raises(ValueError):
        thrForceMapping.computeTorqueAngErr(
            [[0]] * 4, [0, 0, 0], 0, 0.0, [], []
        )
