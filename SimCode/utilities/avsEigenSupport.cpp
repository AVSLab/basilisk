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

#include <iostream>
#include <math.h>
#include <Eigen/Dense>

/*

 Contains various support algorithms related to using the Eigen Library

 */

/*! This function provides a general conversion between an Eigen matrix and
an output C array.  Note that this routine would convert an inbound type
to a MAtrixXd and then transpose the matrix which would be inefficient
in a lot of cases.
@return void
@param inMat The source Eigen matrix that we are converting
@param outArray The destination array (sized by the user!) we copy in to
*/
void eigenMatrixXd2CArray(Eigen::MatrixXd inMat, double *outArray)
{
	Eigen::MatrixXd tempMat = inMat.transpose();
	memcpy(outArray, tempMat.data(), inMat.rows()*inMat.cols()*sizeof(double));
}

/*! This function provides a direct conversion between a 3-vector and an
output C array.  We are providing this function to save on the  inline conversion
and the transpose that would have been performed by the general case.
@return void
@param inMat The source Eigen matrix that we are converting
@param outArray The destination array (sized by the user!) we copy in to
*/
void eigenVector3d2CArray(Eigen::Vector3d & inMat, double *outArray)
{
	memcpy(outArray, inMat.data(), 3 * sizeof(double));
}

/*! This function provides a direct conversion between a 3x3 matrix and an
output C array.  We are providing this function to save on the inline conversion
that would have been performed by the general case.
@return void
@param inMat The source Eigen matrix that we are converting
@param outArray The destination array (sized by the user!) we copy in to
*/
void eigenMatrix3d2CArray(Eigen::Matrix3d & inMat, double *outArray)
{
	Eigen::MatrixXd tempMat = inMat.transpose();
	memcpy(outArray, tempMat.data(), 9 * sizeof(double));
}
/*! This function performs the general conversion between an input C array
and an Eigen matrix.  Note that to use this function the user MUST size
the Eigen matrix ahead of time so that the internal map call has enough
information to ingest the C array.
@return void
@param inArray The input array (row-major)
@param outMat The output Eigen matrix
*/
void cArray2EigenMatrixXd(double *inArray, Eigen::MatrixXd & outMat, int nRows, int nCols)
{
	outMat.resize(nRows, nCols);
	outMat = Eigen::Map<Eigen::MatrixXd>(inArray, outMat.rows(), outMat.cols());
}
/*! This function performs the conversion between an input C array
3-vector and an output Eigen vector3d.  This function is provided
in order to save an unnecessary conversion between types
@return void
@param inArray The input array (row-major)
@param outMat The output Eigen matrix
*/
void cArray2EigenVector3d(double *inArray, Eigen::Vector3d & outMat)
{
	outMat = Eigen::Map<Eigen::Vector3d>(inArray, 3, 1);
}
/*! This function performs the conversion between an input C array
3x3-matrix and an output Eigen vector3d.  This function is provided
in order to save an unnecessary conversion between types
@return void
@param inArray The input array (row-major)
@param outMat The output Eigen matrix
*/
void cArray2EigenMatrix3d(double *inArray, Eigen::Matrix3d & outMat)
{
	outMat = Eigen::Map<Eigen::Matrix3d>(inArray, 3, 3);
}