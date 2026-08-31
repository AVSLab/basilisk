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
#include "avsEigenMRP.h"


//! General conversion between any Eigen matrix and output array.
void eigenMatrixXd2CArray(const Eigen::MatrixXd& inMat, double *outArray);
//! General conversion between any integer Eigen matrix and output array.
void eigenMatrixXi2CArray(const Eigen::MatrixXi& inMat, int *outArray);
#ifndef SWIG
/*! Copy any double-precision Eigen expression into a row-major C array
    without first converting it to a dynamic matrix.

    @param inMat Source Eigen expression.
    @param outArray Destination array sized by the caller.
*/
template<typename Derived>
void eigenMatrixXd2CArray(const Eigen::MatrixBase<Derived>& inMat, double *outArray)
{
    using RowMajorMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<RowMajorMatrixXd> outMat(outArray, inMat.rows(), inMat.cols());
    outMat = inMat;
}

/*! Copy any integer Eigen expression into a row-major C array without first
    converting it to a dynamic matrix.

    @param inMat Source Eigen expression.
    @param outArray Destination array sized by the caller.
*/
template<typename Derived>
void eigenMatrixXi2CArray(const Eigen::MatrixBase<Derived>& inMat, int *outArray)
{
    using RowMajorMatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<RowMajorMatrixXi> outMat(outArray, inMat.rows(), inMat.cols());
    outMat = inMat;
}
#endif
//! Rapid conversion between 3-vector and output array.
void eigenVector3d2CArray(const Eigen::Vector3d& inMat, double *outArray);
//! Rapid conversion between MRP and output array.
void eigenMRPd2CArray(const Eigen::MRPd& inMat, double* outArray);
//! Rapid conversion between 3x3 matrix and output array.
void eigenMatrix3d2CArray(const Eigen::Matrix3d& inMat, double *outArray);
//! General conversion between a C array and an Eigen matrix.
Eigen::MatrixXd cArray2EigenMatrixXd(double *inArray, int nRows, int nCols);
//! Specific conversion between a C array and an Eigen 3-vector.
Eigen::Vector3d cArray2EigenVector3d(double *inArray);
//! Specific conversion between a C array and an Eigen MRP.
Eigen::MRPd cArray2EigenMRPd(const double* inArray);
//! Specific conversion between a C array and an Eigen 3x3 matrix.
Eigen::Matrix3d cArray2EigenMatrix3d(double *inArray);
//! Specific conversion between a C 2D array and an Eigen 3x3 matrix.
Eigen::Matrix3d c2DArray2EigenMatrix3d(double in2DArray[3][3]);
//!@brief returns the first axis DCM with the input angle
Eigen::Matrix3d eigenM1(double angle);
//!@brief returns the second axis DCM with the input angle
Eigen::Matrix3d eigenM2(double angle);
//!@brief returns the third axis DCM with the input angle
Eigen::Matrix3d eigenM3(double angle);
//!@brief returns the tilde matrix representation of a vector (equivalent to a vector cross product)
Eigen::Matrix3d eigenTilde(Eigen::Vector3d vec);
//!@brief converts MRPd to an Vector3d variable
Eigen::Vector3d eigenMRPd2Vector3d(Eigen::MRPd vec);
//!@brief maps the DCM to MRPs using Eigen variables
Eigen::MRPd eigenC2MRP(Eigen::Matrix3d);

//!@brief returns true if the 3x3 matrix is a proper rotation matrix (orthonormal with determinant +1, i.e. orthogonal and right-handed) within the given tolerance
bool eigenIsRotationMatrix(const Eigen::Matrix3d& dcm, double tolerance = 1e-9);
//!@brief returns true if the 3-vector has unit norm within the given tolerance
bool eigenIsUnitVector(const Eigen::Vector3d& vec, double tolerance = 1e-9);
//!@brief returns true if the 3x3 matrix is a valid inertia tensor within the given tolerance: finite, symmetric,
//! positive semi-definite with at most one zero principal inertia, and satisfying the triangle inequality
bool eigenIsValidInertiaMatrix(const Eigen::Matrix3d& inertia, double tolerance = 1e-9);
//!@brief returns true if the finite 3x3 matrix is symmetric positive semidefinite within the given tolerance
bool eigenIsPositiveSemidefiniteMatrix(const Eigen::Matrix3d& matrix, double tolerance = 1e-9);

//!@brief solves for the zero of the provided function
double newtonRaphsonSolve(const double& initialEstimate, const double& accuracy, const std::function<double(double)>& f, const std::function<double(double)>& fPrime);

//!@brief solves for the zero of the provided function using Bisection
double bisectionSolve(double *interval, double accuracy, std::function< double(double) >& f);

#endif /* _AVSEIGENSUPPORT_ */
