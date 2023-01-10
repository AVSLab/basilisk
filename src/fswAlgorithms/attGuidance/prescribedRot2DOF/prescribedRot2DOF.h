/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _PRESCRIBEDROT2DOF_
#define _PRESCRIBEDROT2DOF_

/*! Include the required files. */
#include <stdint.h>
#include "architecture/utilities/bskLogging.h"

/*! @brief Top level structure for the sub-module routines. */
typedef struct
{

    BSKLogger *bskLogger;                                      //!< BSK Logging

}PrescribedRot2DOFConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_prescribedRot2DOF(PrescribedRot2DOFConfig *configData, int64_t moduleID);                         //<! Method for initializing the module
    void Reset_prescribedRot2DOF(PrescribedRot2DOFConfig *configData, uint64_t callTime, int64_t moduleID);         //<! Method for resetting the module
    void Update_prescribedRot2DOF(PrescribedRot2DOFConfig *configData, uint64_t callTime, int64_t moduleID);        //<! Method for the updating the module
#ifdef __cplusplus
}
#endif

#endif
