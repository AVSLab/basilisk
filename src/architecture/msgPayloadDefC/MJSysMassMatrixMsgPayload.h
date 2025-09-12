/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef MJ_SYS_MASS_MATRIX_MESSAGE_H
#define MJ_SYS_MASS_MATRIX_MESSAGE_H

#include "architecture/utilities/macroDefinitions.h"

#define MJ_MASSMATRIX_DIM (6 + MAX_EFF_CNT)                        // row/col size

/*! @brief This structure is used in the messaging system to communicate what the mass matrix for
the full spacecraft system is currently from Mujoco.*/
typedef struct {
    int nbase;                                               //!< [-] number of base DOFs (6 if free base, else 0)
    int nj;                                                  //!< [-] number of joint DOFs populated in mass matrix
    double MassMatrix[6 + MAX_EFF_CNT][6 + MAX_EFF_CNT];     //!< [varies by component] system mass matrix in generalized coordinates
}MJSysMassMatrixMsgPayload;


#endif
