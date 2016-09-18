/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

#ifndef _UKF_UTILITIES_H_
#define _UKF_UTILITIES_H_

#include <stdint.h>

#define UKF_MAX_DIM 12

/*! \addtogroup ADCSAlgGroup
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

	void ukfQRDJustR(
		double *sourceMat, uint32_t nRow, uint32_t nCol, double *destMat);
	void ukfLInv(
		double *sourceMat, uint32_t nRow, uint32_t nCol, double *destMat);
	void ukfUInv(
		double *sourceMat, uint32_t nRow, uint32_t nCol, double *destMat);
	int32_t ukfLUD(double *sourceMat, uint32_t nRow, uint32_t nCol,
		double *destMat, int32_t *indx);
	void ukfLUBckSlv(double *sourceMat, uint32_t nRow, uint32_t nCol,
		int32_t *indx, double *bmat, double *destMat);
	void ukfMatInv(double *sourceMat, uint32_t nRow, uint32_t nCol,
		double *destMat);
	void ukfCholDecomp(double *sourceMat, uint32_t nRow, uint32_t nCol,
		double *destMat);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
