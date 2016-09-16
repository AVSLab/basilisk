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

#include "attDetermination/_GeneralModuleFiles/UKFUtilities.h"
#include "../SimCode/utilities/linearAlgebra.h"

void ukfQRDJustR(
	double *inMat, uint32_t nRow, uint32_t nCol, double *destMat)
{
	uint32_t i, j, k, dimi, dimj;
	double qMat[UKF_MAX_DIM*UKF_MAX_DIM];
	double sourceMat[UKF_MAX_DIM*UKF_MAX_DIM];
	double sum;

	mSetZero(qMat, UKF_MAX_DIM, UKF_MAX_DIM);
	mSetZero(sourceMat, UKF_MAX_DIM, UKF_MAX_DIM);
	mSetZero(destMat, nRow, nCol);
	mCopy(inMat, nRow, nCol, sourceMat);

	for (i = 0; i<nCol; i++)
	{
		dimi = i*nCol + i;
		for (j = 0; j<nRow; j++)
		{
			dimj = nCol * j;
			destMat[dimi] += sourceMat[dimj + i] * sourceMat[dimj + i];
		}

		destMat[dimi] = sqrt(destMat[dimi]);
		for (j = 0; j<nRow; j++)
		{
			qMat[j*nCol + i] =
				sourceMat[j*nCol + i] / destMat[dimi];
		}

		for (j = i + 1; j<nCol; j++)
		{
			sum = 0;
			dimj = nCol * j;
			for (k = 0; k<nRow; k++)
			{
				sum += sourceMat[k*nCol + j] * qMat[k*nCol + i];
			}
			destMat[i*nCol + j] = sum;
			for (k = 0; k<nRow; k++)
			{
				sourceMat[k*nCol + j] -= sum*qMat[k*nCol + i];
			}
		}
	}

	return;
}
