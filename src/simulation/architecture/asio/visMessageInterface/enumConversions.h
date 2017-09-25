/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#ifndef ENUM_CONVERSIONS_H
#define ENUM_CONVERSIONS_H

#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <map>

template<typename T>
using enumMap = std::map<T, std::string>;

// This is the type that will hold all the strings.
// Each enumerate type will declare its own specialization.
// Any enum that does not have a specialization will generate a compiler error
// indicating that there is no definition of this variable (as there should be
// be no definition of a generic version).
template<typename T>
struct enumStrings
{
    static const enumMap<T> data;
};

// This is a utility type. Created automatically. Should not be used directly.
template<typename T>
struct enumRefHolder
{
    T& enumVal;
    enumRefHolder(T& enumVal): enumVal(enumVal) {}
};
template<typename T>
struct enumConstRefHolder
{
    T const& enumVal;
    enumConstRefHolder(T const& enumVal): enumVal(enumVal) {}
};

// The next two functions do the actual work of reading/writing an enum as a string.
template<typename T>
std::ostream& operator<<(std::ostream &str, const enumConstRefHolder<T> &data)
{
    auto iter = enumStrings<T>::data.begin();
    for(; iter != enumStrings<T>::data.end(); ++iter) {
        if(data.enumVal == iter->first) { // first=T
            return str << iter->second; // second= std::string
        }
    }
    if(iter == enumStrings<T>::data.end()) {
        std::cout << "WARNING: enum not recognized (<<)" << std::endl;
    }
    return str;
}
template<typename T>
std::istream& operator>>(std::istream& str, enumRefHolder<T> const& data)
{
    std::string value;
    str >> value;
    
    auto iter = enumStrings<T>::data.begin();
    for(; iter != enumStrings<T>::data.end(); ++iter) {
        if(value.compare(iter->second) == 0) {
            data.enumVal = iter->first;
            break;
        }
    }
    if(iter == enumStrings<T>::data.end()) {
        std::cout << "WARNING: enum not recognized (>>) " << value << std::endl;
    }
    return str;
}

// This is the public interface:
// Use the ability of function to deduce their template type without
// being explicitly told to create the correct type of enumRefHolder<T>
template<typename T>
enumConstRefHolder<T> enumToString(T const& e)
{
    return enumConstRefHolder<T>(e);
}

template<typename T>
enumRefHolder<T> enumFromString(T& e)
{
    return enumRefHolder<T>(e);
}

#endif