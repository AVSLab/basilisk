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


#ifndef _AVSEIGENSUPPORT_
#define _AVSEIGENSUPPORT_
#include <Eigen/Dense>

/*! \addtogroup Sim Utility Group
* @{
*/

//!@brief General conversion between any Eigen matrix and output array
void eigenMatrixXd2CArray(Eigen::MatrixXd inMat, double *outArray);
//!@brief Rapid conversion between 3-vector and output array
void eigenVector3d2CArray(Eigen::Vector3d & inMat, double *outArray);
//!@brief Rapid conversion between 3x3 matrix and output array
void eigenMatrix3d2CArray(Eigen::Matrix3d & inMat, double *outArray);
//!@brief General conversion between a C array and an Eigen matrix
Eigen::MatrixXd cArray2EigenMatrixXd(double *inArray, int nRows, int nCols);
//!@brief Specific conversion between a C array and an Eigen 3-vector
Eigen::Vector3d cArray2EigenVector3d(double *inArray);
//!@brief Specfici conversion between a C array and an Eigen 3x3 matrix
Eigen::Matrix3d cArray2EigenMatrix3d(double *inArray);
//!@brief returns the first axis DCM with the input angle 
Eigen::Matrix3d eigenM1(double angle);
//!@brief returns the second axis DCM with the input angle
Eigen::Matrix3d eigenM2(double angle);
//!@brief returns the third axis DCM with the input angle
Eigen::Matrix3d eigenM3(double angle);
//!@brief returns the tilde matrix representation of a vector (equivalent to a vector cross product)
Eigen::Matrix3d eigenTilde(Eigen::Vector3d vec);
//!@brief solves for the zero of the provided function
double newtonRaphsonSolve(double initialEstimate, double accuracy, std::function< double(double) >& f, std::function< double(double) >& fPrime);


#endif /* _GaussMarkov_HH_ */
