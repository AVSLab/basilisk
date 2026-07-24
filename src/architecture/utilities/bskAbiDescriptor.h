/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef BSK_ABI_DESCRIPTOR_H
#define BSK_ABI_DESCRIPTOR_H

#ifdef __cplusplus
#include <Python.h>

#include <Eigen/Core>
#endif

#include <stddef.h>
#include <stdint.h>

#include "architecture/messaging/msgHeader.h"

#define BSK_ABI_DESCRIPTOR_VERSION 1
#define BSK_EXTENSION_ABI_VERSION 1

#define BSK_ABI_STRINGIFY_IMPL(value) #value
#define BSK_ABI_STRINGIFY(value) BSK_ABI_STRINGIFY_IMPL(value)

/*! @cond BSK_INTERNAL */

#if defined(_MSC_VER) && !defined(__clang__)
#define BSK_ABI_INLINE static __inline
#else
#define BSK_ABI_INLINE static inline
#endif

typedef struct
{
    uint8_t first;
    uint32_t second;
    uint16_t third;
} BskAbiPackingCanary;

typedef struct
{
    double vector[3];
    int32_t status;
} BskAbiPayloadCanary;

typedef struct
{
    char prefix;
    BskAbiPackingCanary value;
} BskAbiPackingCanaryAlignment;

typedef struct
{
    char prefix;
    MsgHeader value;
} BskAbiMsgHeaderAlignment;

typedef struct
{
    const char* compilerId;
    const char* compilerVersion;
    const char* runtimeFamily;
    const char* endianness;
    long long compilerMajorVersion;
    long long compilerMinorVersion;
    long long compilerPatchVersion;
    long long languageStandard;
    long long msvcVersion;
    long long runtimeMajorVersion;
    long long runtimeMinorVersion;
    int charIsSigned;
    size_t shortSize;
    size_t intSize;
    size_t longSize;
    size_t longLongSize;
    size_t sizeTSize;
    size_t pointerSize;
    size_t packingCanarySize;
    size_t packingCanaryAlignment;
    size_t packingCanaryUint32Offset;
    size_t packingCanaryUint16Offset;
    size_t msgHeaderSize;
    size_t msgHeaderAlignment;
    size_t msgHeaderIsWrittenOffset;
    size_t msgHeaderTimeWrittenOffset;
    size_t msgHeaderModuleIdOffset;
} BskCAbiInfo;

BSK_ABI_INLINE BskCAbiInfo
bskCreateCAbiInfo(void)
{
    BskCAbiInfo info = { 0 };
    const uint16_t endianValue = 1;

#if defined(__apple_build_version__) && defined(__clang__)
    info.compilerId = "AppleClang";
    info.compilerVersion = __clang_version__;
    info.compilerMajorVersion = __clang_major__;
    info.compilerMinorVersion = __clang_minor__;
    info.compilerPatchVersion = __clang_patchlevel__;
#elif defined(__clang__)
    info.compilerId = "Clang";
    info.compilerVersion = __clang_version__;
    info.compilerMajorVersion = __clang_major__;
    info.compilerMinorVersion = __clang_minor__;
    info.compilerPatchVersion = __clang_patchlevel__;
#elif defined(_MSC_VER)
    info.compilerId = "MSVC";
    info.compilerVersion = BSK_ABI_STRINGIFY(_MSC_FULL_VER);
    info.compilerMajorVersion = _MSC_VER / 100;
    info.compilerMinorVersion = _MSC_VER % 100;
#if defined(_MSC_BUILD)
    info.compilerPatchVersion = _MSC_BUILD;
#endif
#elif defined(__GNUC__)
    info.compilerId = "GNU";
    info.compilerVersion = __VERSION__;
    info.compilerMajorVersion = __GNUC__;
    info.compilerMinorVersion = __GNUC_MINOR__;
    info.compilerPatchVersion = __GNUC_PATCHLEVEL__;
#else
    info.compilerId = "Unknown";
    info.compilerVersion = "";
#endif

#if defined(__GLIBC__)
    info.runtimeFamily = "glibc";
    info.runtimeMajorVersion = __GLIBC__;
    info.runtimeMinorVersion = __GLIBC_MINOR__;
#elif defined(_WIN32)
    info.runtimeFamily = "windows-crt";
#elif defined(__APPLE__)
    info.runtimeFamily = "libSystem";
#else
    info.runtimeFamily = "unknown";
#endif

#if defined(__STDC_VERSION__)
    info.languageStandard = __STDC_VERSION__;
#endif
#if defined(_MSC_VER)
    info.msvcVersion = _MSC_VER;
#endif

    info.endianness = *((const uint8_t*)&endianValue) == 1 ? "little" : "big";
    info.charIsSigned = ((char)-1) < 0;
    info.shortSize = sizeof(short);
    info.intSize = sizeof(int);
    info.longSize = sizeof(long);
    info.longLongSize = sizeof(long long);
    info.sizeTSize = sizeof(size_t);
    info.pointerSize = sizeof(void*);
    info.packingCanarySize = sizeof(BskAbiPackingCanary);
    info.packingCanaryAlignment = offsetof(BskAbiPackingCanaryAlignment, value);
    info.packingCanaryUint32Offset = offsetof(BskAbiPackingCanary, second);
    info.packingCanaryUint16Offset = offsetof(BskAbiPackingCanary, third);
    info.msgHeaderSize = sizeof(MsgHeader);
    info.msgHeaderAlignment = offsetof(BskAbiMsgHeaderAlignment, value);
    info.msgHeaderIsWrittenOffset = offsetof(MsgHeader, isWritten);
    info.msgHeaderTimeWrittenOffset = offsetof(MsgHeader, timeWritten);
    info.msgHeaderModuleIdOffset = offsetof(MsgHeader, moduleID);
    return info;
}

#ifdef __cplusplus

#include <string>
#include <vector>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"

namespace bsk {
namespace abi {

struct CxxAbiInfo
{
    const char* targetSystem;
    const char* targetArchitecture;
    const char* instructionSet;
    const char* compilerId;
    const char* compilerVersion;
    const char* compilerFrontend;
    const char* compilerAbiFamily;
    long long compilerMajorVersion;
    long long compilerMinorVersion;
    long long compilerPatchVersion;
    long long compilerLanguageStandard;
    long long compilerAbiVersion;
    long long msvcFullVersion;
    const char* standardLibraryFamily;
    long long standardLibraryVersion;
    long long standardLibraryAbiVersion;
    long long libstdcxxRelease;
    long long msvcStlUpdate;
    long long libstdcxxUseCxx11Abi;
    long long iteratorDebugLevel;
    bool standardLibraryDebugMode;
    bool standardLibraryAssertions;
    const char* runtimeLinkage;
    bool debugRuntime;
    bool assertionsDisabled;
    bool exceptionsEnabled;
    bool rttiEnabled;
    size_t stdStringSize;
    size_t stdStringAlignment;
    size_t stdVectorIntSize;
    size_t stdVectorIntAlignment;
    size_t sysModelSize;
    size_t sysModelAlignment;
    size_t bskLoggerSize;
    size_t bskLoggerAlignment;
    size_t msgHeaderSize;
    size_t msgHeaderAlignment;
    size_t readFunctorSize;
    size_t readFunctorAlignment;
    size_t messageSize;
    size_t messageAlignment;
    const char* eigenVectorization;
    long long eigenMaxAlignBytes;
    long long eigenMaxStaticAlignBytes;
    size_t eigenIndexSize;
    size_t eigenVector3dSize;
    size_t eigenVector3dAlignment;
    size_t eigenVectorXdSize;
    size_t eigenVectorXdAlignment;
    size_t eigenMatrix3dSize;
    size_t eigenMatrix3dAlignment;
    long long pythonLimitedApi;
    bool pythonDebugBuild;
    bool pythonFreeThreadedBuild;
};

inline CxxAbiInfo
createCxxAbiInfo()
{
    CxxAbiInfo info{};

#if defined(_WIN32)
    info.targetSystem = "Windows";
#elif defined(__APPLE__)
    info.targetSystem = "Darwin";
#elif defined(__linux__)
    info.targetSystem = "Linux";
#else
    info.targetSystem = "Unknown";
#endif

#if defined(_M_ARM64) || defined(__aarch64__)
    info.targetArchitecture = "arm64";
#elif defined(_M_X64) || defined(__x86_64__)
    info.targetArchitecture = "x86_64";
#elif defined(_M_IX86) || defined(__i386__)
    info.targetArchitecture = "x86";
#elif defined(_M_ARM) || defined(__arm__)
    info.targetArchitecture = "arm";
#else
    info.targetArchitecture = "unknown";
#endif

#if defined(__AVX512F__)
    info.instructionSet = "AVX512";
#elif defined(__AVX2__)
    info.instructionSet = "AVX2";
#elif defined(__AVX__)
    info.instructionSet = "AVX";
#elif defined(__SSE4_2__)
    info.instructionSet = "SSE4.2";
#elif defined(__SSE2__) || defined(_M_X64)
    info.instructionSet = "SSE2";
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
    info.instructionSet = "NEON";
#else
    info.instructionSet = "baseline";
#endif

#if defined(__apple_build_version__) && defined(__clang__)
    info.compilerId = "AppleClang";
    info.compilerVersion = __clang_version__;
    info.compilerMajorVersion = __clang_major__;
    info.compilerMinorVersion = __clang_minor__;
    info.compilerPatchVersion = __clang_patchlevel__;
#elif defined(__clang__)
    info.compilerId = "Clang";
    info.compilerVersion = __clang_version__;
    info.compilerMajorVersion = __clang_major__;
    info.compilerMinorVersion = __clang_minor__;
    info.compilerPatchVersion = __clang_patchlevel__;
#elif defined(_MSC_VER)
    info.compilerId = "MSVC";
    info.compilerVersion = BSK_ABI_STRINGIFY(_MSC_FULL_VER);
    info.compilerMajorVersion = _MSC_VER / 100;
    info.compilerMinorVersion = _MSC_VER % 100;
#if defined(_MSC_BUILD)
    info.compilerPatchVersion = _MSC_BUILD;
#endif
#elif defined(__GNUC__)
    info.compilerId = "GNU";
    info.compilerVersion = __VERSION__;
    info.compilerMajorVersion = __GNUC__;
    info.compilerMinorVersion = __GNUC_MINOR__;
    info.compilerPatchVersion = __GNUC_PATCHLEVEL__;
#else
    info.compilerId = "Unknown";
    info.compilerVersion = "";
#endif

#if defined(_MSC_VER)
    info.compilerFrontend = "MSVC";
    info.compilerAbiFamily = "msvc";
    info.compilerAbiVersion = _MSC_VER;
    info.msvcFullVersion = _MSC_FULL_VER;
#else
    info.compilerFrontend = "GNU";
    info.compilerAbiFamily = "itanium";
#if defined(__GXX_ABI_VERSION)
    info.compilerAbiVersion = __GXX_ABI_VERSION;
#endif
#endif

#if defined(_MSVC_LANG)
    info.compilerLanguageStandard = _MSVC_LANG;
#else
    info.compilerLanguageStandard = __cplusplus;
#endif

    info.libstdcxxUseCxx11Abi = -1;
    info.iteratorDebugLevel = -1;
#if defined(_MSVC_STL_VERSION)
    info.standardLibraryFamily = "msvc";
    info.standardLibraryVersion = _MSVC_STL_VERSION;
    info.standardLibraryAbiVersion = _MSVC_STL_VERSION;
#elif defined(_LIBCPP_VERSION)
    info.standardLibraryFamily = "libc++";
    info.standardLibraryVersion = _LIBCPP_VERSION;
#if defined(_LIBCPP_ABI_VERSION)
    info.standardLibraryAbiVersion = _LIBCPP_ABI_VERSION;
#endif
#elif defined(__GLIBCXX__)
    info.standardLibraryFamily = "libstdc++";
    info.standardLibraryVersion = __GLIBCXX__;
#else
    info.standardLibraryFamily = "unknown";
#endif
#if defined(_GLIBCXX_RELEASE)
    info.libstdcxxRelease = _GLIBCXX_RELEASE;
#endif
#if defined(_MSVC_STL_UPDATE)
    info.msvcStlUpdate = _MSVC_STL_UPDATE;
#endif
#if defined(_GLIBCXX_USE_CXX11_ABI)
    info.libstdcxxUseCxx11Abi = _GLIBCXX_USE_CXX11_ABI;
#endif
#if defined(_ITERATOR_DEBUG_LEVEL)
    info.iteratorDebugLevel = _ITERATOR_DEBUG_LEVEL;
#endif
#if defined(_GLIBCXX_DEBUG)
    info.standardLibraryDebugMode = true;
#endif
#if defined(_GLIBCXX_ASSERTIONS)
    info.standardLibraryAssertions = true;
#endif

#if defined(_DLL)
    info.runtimeLinkage = "dynamic";
#elif defined(_MT)
    info.runtimeLinkage = "static";
#else
    info.runtimeLinkage = "system";
#endif
#if defined(_DEBUG)
    info.debugRuntime = true;
#endif
#if defined(NDEBUG)
    info.assertionsDisabled = true;
#endif
#if defined(__cpp_exceptions) || defined(__EXCEPTIONS) || defined(_CPPUNWIND)
    info.exceptionsEnabled = true;
#endif
#if defined(__GXX_RTTI) || defined(_CPPRTTI)
    info.rttiEnabled = true;
#endif

    info.stdStringSize = sizeof(std::string);
    info.stdStringAlignment = alignof(std::string);
    info.stdVectorIntSize = sizeof(std::vector<int>);
    info.stdVectorIntAlignment = alignof(std::vector<int>);
    info.sysModelSize = sizeof(SysModel);
    info.sysModelAlignment = alignof(SysModel);
    info.bskLoggerSize = sizeof(BSKLogger);
    info.bskLoggerAlignment = alignof(BSKLogger);
    info.msgHeaderSize = sizeof(MsgHeader);
    info.msgHeaderAlignment = alignof(MsgHeader);
    info.readFunctorSize = sizeof(ReadFunctor<BskAbiPayloadCanary>);
    info.readFunctorAlignment = alignof(ReadFunctor<BskAbiPayloadCanary>);
    info.messageSize = sizeof(Message<BskAbiPayloadCanary>);
    info.messageAlignment = alignof(Message<BskAbiPayloadCanary>);

#if defined(EIGEN_DONT_VECTORIZE)
    info.eigenVectorization = "disabled";
#elif defined(EIGEN_VECTORIZE_AVX512)
    info.eigenVectorization = "AVX512";
#elif defined(EIGEN_VECTORIZE_AVX2)
    info.eigenVectorization = "AVX2";
#elif defined(EIGEN_VECTORIZE_AVX)
    info.eigenVectorization = "AVX";
#elif defined(EIGEN_VECTORIZE_SSE4_2)
    info.eigenVectorization = "SSE4.2";
#elif defined(EIGEN_VECTORIZE_SSE2)
    info.eigenVectorization = "SSE2";
#elif defined(EIGEN_VECTORIZE_NEON)
    info.eigenVectorization = "NEON";
#else
    info.eigenVectorization = "none";
#endif
    info.eigenMaxAlignBytes = EIGEN_MAX_ALIGN_BYTES;
#if defined(EIGEN_MAX_STATIC_ALIGN_BYTES)
    info.eigenMaxStaticAlignBytes = EIGEN_MAX_STATIC_ALIGN_BYTES;
#endif
    info.eigenIndexSize = sizeof(Eigen::Index);
    info.eigenVector3dSize = sizeof(Eigen::Vector3d);
    info.eigenVector3dAlignment = alignof(Eigen::Vector3d);
    info.eigenVectorXdSize = sizeof(Eigen::VectorXd);
    info.eigenVectorXdAlignment = alignof(Eigen::VectorXd);
    info.eigenMatrix3dSize = sizeof(Eigen::Matrix3d);
    info.eigenMatrix3dAlignment = alignof(Eigen::Matrix3d);

#if defined(Py_LIMITED_API)
    info.pythonLimitedApi = Py_LIMITED_API;
#endif
#if defined(Py_DEBUG)
    info.pythonDebugBuild = true;
#endif
#if defined(Py_GIL_DISABLED) && Py_GIL_DISABLED
    info.pythonFreeThreadedBuild = true;
#endif
    return info;
}

} // namespace abi
} // namespace bsk

#endif

/*! @endcond */

#undef BSK_ABI_INLINE
#undef BSK_ABI_STRINGIFY
#undef BSK_ABI_STRINGIFY_IMPL

#endif
