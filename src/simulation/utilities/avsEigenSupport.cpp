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
@return Eigen::MatrixXd
@param inArray The input array (row-major)
@param outMat The output Eigen matrix
*/
Eigen::MatrixXd cArray2EigenMatrixXd(double *inArray, int nRows, int nCols)
{
    Eigen::MatrixXd outMat;
    outMat.resize(nRows, nCols);
	outMat = Eigen::Map<Eigen::MatrixXd>(inArray, outMat.rows(), outMat.cols());
    return outMat;
}
/*! This function performs the conversion between an input C array
3-vector and an output Eigen vector3d.  This function is provided
in order to save an unnecessary conversion between types
@return Eigen::Vector3d
@param inArray The input array (row-major)
@param outMat The output Eigen matrix
*/
Eigen::Vector3d cArray2EigenVector3d(double *inArray)
{
    return Eigen::Map<Eigen::Vector3d>(inArray, 3, 1);
}
/*! This function performs the conversion between an input C array
3x3-matrix and an output Eigen vector3d.  This function is provided
in order to save an unnecessary conversion between types
@return Eigen::Matrix3d
@param inArray The input array (row-major)
@param outMat The output Eigen matrix
*/
Eigen::Matrix3d cArray2EigenMatrix3d(double *inArray)
{
	return Eigen::Map<Eigen::Matrix3d>(inArray, 3, 3);
}


/*! This function returns the Eigen DCM that corresponds to a 1-axis rotation
 by the angle theta.  The DCM is the positive theta rotation from the original
 frame to the final frame.
 @return Eigen::Matrix3d
 @param angle The input rotation angle
 */
Eigen::Matrix3d eigenM1(double angle)
{
    Eigen::Matrix3d mOut;

    mOut.setIdentity();

    mOut(1,1) = cos(angle);
    mOut(1,2) = sin(angle);
    mOut(2,1) = -mOut(1,2);
    mOut(2,2) = mOut(1,1);

    return mOut;
}


/*! This function returns the Eigen DCM that corresponds to a 2-axis rotation
 by the angle theta.  The DCM is the positive theta rotation from the original
 frame to the final frame.
 @return Eigen::Matrix3d
 @param angle The input rotation angle
 */
Eigen::Matrix3d eigenM2(double angle)
{
    Eigen::Matrix3d mOut;

    mOut.setIdentity();

    mOut(0,0) = cos(angle);
    mOut(0,2) = -sin(angle);
    mOut(2,0) = -mOut(0,2);
    mOut(2,2) = mOut(0,0);

    return mOut;
}


/*! This function returns the Eigen DCM that corresponds to a 3-axis rotation
 by the angle theta.  The DCM is the positive theta rotation from the original
 frame to the final frame.
 @return Eigen::Matrix3d
 @param angle The input rotation angle
 */
Eigen::Matrix3d eigenM3(double angle)
{
    Eigen::Matrix3d mOut;

    mOut.setIdentity();

    mOut(0,0) = cos(angle);
    mOut(0,1) = sin(angle);
    mOut(1,0) = -mOut(0,1);
    mOut(1,1) = mOut(0,0);

    return mOut;
}


/*! This function returns the tilde matrix version of a vector. The tilde
 matrix is the matrixi equivalent of a vector cross product, where
 [tilde_a] b == a x b
 @return Eigen::Matrix3d
 @param vec The input vector
 */
Eigen::Matrix3d eigenTilde(Eigen::Vector3d vec)
{
    Eigen::Matrix3d mOut;

    mOut(0,0) = mOut(1,1) = mOut(2,2) = 0.0;

    mOut(0,1) = -vec(2);
    mOut(1,0) =  vec(2);
    mOut(0,2) =  vec(1);
    mOut(2,0) = -vec(1);
    mOut(1,2) = -vec(0);
    mOut(2,1) =  vec(0);

    return mOut;
}


/*! This function solves for the zero of the passed function using the Newton Raphson Method
@return double
@param initialEstimate The initial value to use for newton-raphson
@param accuracy The desired upper bound for the error
@param f Function to find the zero of
@param fPrime First derivative of the function
*/
double newtonRaphsonSolve(double initialEstimate, double accuracy, std::function< double(double) >& f, std::function<
                          double(double) >& fPrime) {
	double currentEstimate = initialEstimate;
	for (int i = 0; i < 100 && std::abs(f(currentEstimate)) > accuracy; i++) {
		double functionVal = f(currentEstimate);
		double functionDeriv = fPrime(currentEstimate);
		currentEstimate = currentEstimate - functionVal/functionDeriv;
	}
	return currentEstimate;
}

