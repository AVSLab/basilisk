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

%module swig_common_model

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();


%include "std_vector.i"
%include "std_string.i"
%include "std_set.i"
%include "std_pair.i"
%include "swig_conly_data.i"
%feature("copyctor");
%array_functions(bool, boolArray);

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int, allocator<int> >;
   %template(DoubleVector) vector<double, allocator<double> >;
   %template(StringVector) vector<string, allocator<string> >;
   %template(StringSet) set<string>;
   %template(intSet) set<unsigned long>;
   %template(ConstCharVector) vector<const char*, allocator<const char*> >;
   %template(MultiArray) vector < vector <double> >;
   %template(MultiArray3d) vector < vector < vector <double> > >;
}

%include "swig_eigen.i"

%pythoncode %{
class FixedSizeSequence:
    """Expose indexed native storage without permitting size changes."""

    _index_label = "sequence"
    _slice_size_label = "sequence size"

    def __init__(self, owner, count, get_item, set_item):
        object.__setattr__(self, "_owner", owner)
        object.__setattr__(self, "_count", count)
        object.__setattr__(self, "_get_item", get_item)
        object.__setattr__(self, "_set_item", set_item)

    def __len__(self):
        return self._count()

    def _normalize_index(self, index):
        if not isinstance(index, int):
            raise TypeError(f"{self._index_label} indices must be integers")
        if index < 0:
            index += len(self)
        if index < 0 or index >= len(self):
            raise IndexError(f"{self._index_label} index out of range")
        return index

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[position] for position in range(*index.indices(len(self)))]
        return self._get_item(self._normalize_index(index))

    def __setitem__(self, index, value):
        if isinstance(index, slice):
            positions = list(range(*index.indices(len(self))))
            values = list(value)
            if len(positions) != len(values):
                raise ValueError(
                    f"slice assignment cannot change {self._slice_size_label}"
                )
            for position, item in zip(positions, values):
                self._set_item(position, item)
            return
        self._set_item(self._normalize_index(index), value)

    def __iter__(self):
        for index in range(len(self)):
            yield self[index]

    def replace(self, values):
        self[:] = values


class GuardedConfigSequence(FixedSizeSequence):
    """Expose live indexed configuration access through guarded native mutators."""

    _index_label = "configuration"
    _slice_size_label = "configuration collection size"

    def __init__(self, owner, count, get_item, set_item, append_item):
        super().__init__(owner, count, get_item, set_item)
        object.__setattr__(self, "_append_item", append_item)

    def __getitem__(self, index):
        item = super().__getitem__(index)
        try:
            item._swig_bsk_owner = self._owner
        except AttributeError:
            pass
        return item

    def append(self, value):
        self._append_item(value)

    def extend(self, values):
        for value in values:
            self.append(value)

    def replace(self, values):
        values = list(values)
        if len(values) != len(self):
            raise ValueError(
                "assignment cannot change configuration collection size; use append()"
            )
        for index, value in enumerate(values):
            self._set_item(index, value)
%}
