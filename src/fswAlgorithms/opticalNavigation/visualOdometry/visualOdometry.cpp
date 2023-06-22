/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric Space Physics, University of Colorado Boulder

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

#include "fswAlgorithms/opticalNavigation/visualOdometry/visualOdometry.h"
#include <cmath>
#include <array>

VisualOdometry::VisualOdometry() = default;

VisualOdometry::~VisualOdometry() = default;

/*! Reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void VisualOdometry::Reset(uint64_t currentSimNanos)
{
    // check that required input messages are connected
    if (!this->keyPointPairInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VisialOdometry.keyPointPairInMsg was not linked.");
    }
    // check that required input messages are connected
    if (!this->cameraConfigInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VisialOdometry.cameraInputMsg was not linked.");
    }
}

/*! Main method that gets called every time the module is updated.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void VisualOdometry::UpdateState(uint64_t currentSimNanos) {
    Eigen::Vector3d sPrime;
    Eigen::Matrix3d covar;
    int sign = 1;
    /*! -read messages */
    if (this->readMessages()) {
        /*! -compute partials of linear system and matrices in cost function */
        for (int i = 0; i < this->numberFeatures; i++) {
            this->computeSampsonPartials(i);
            this->computeGammas(i);
            this->computeKsis(i);
        }
        /*! -build measured linear system */
        this->computeHTH();
        Eigen::Vector3d sPrime_mMin1;
        /*! -get initial direction of motion sPrime */
        sPrime_mMin1 = this->svdLastColumn(this->HtransposeH);

        /*! -begin loop to find cost function minima */
        for (int m = 0; m < this->m_max; m++) {
            /*! compute cost function matrices */
            this->computeRinv(sPrime_mMin1);
            this->computeX();
            Eigen::Vector3d sPrime_m;
            /*! svd and test for convergence */
            sPrime_m = this->svdLastColumn(this->X);
            double deltaS = (sPrime_m - sPrime_mMin1).norm();
            if (deltaS < this->errorTolerance) {
                sPrime = sPrime_m;
                break;
            }
            else if (m == this->m_max - 1){
                sPrime = sPrime_m;
            }
            else{
                sPrime_mMin1 = sPrime_m;
            }
        }
        /*! -perform cheirality test (to discern between mirror solutions which are both mathematically valid) */
        sign = this->cheiralityTest(sPrime);
        covar = this->computeCovariance();
    } else {
        covar.setZero();
        sPrime.setZero();
    }
    /*! -write output messages*/
    this->writeMessages(sign * sPrime, covar, currentSimNanos);
}

/*! Compute the camera matrix inverse using the camera message, equation 23 in the reference document
    @return void
*/
void VisualOdometry::computeCameraMatrix(){
    // camera parameters
    double alpha = 0;
    double fieldOfView = this->cameraBuffer.fieldOfView;
    double resolutionX = this->cameraBuffer.resolution[0];
    double resolutionY = this->cameraBuffer.resolution[1];
    double pX = 2.*tan(fieldOfView/2.0);
    double pY = 2.*tan(fieldOfView*resolutionY/resolutionX/2.0);
    double dX = resolutionX/pX;
    double dY = resolutionY/pY;
    double up = resolutionX/2;
    double vp = resolutionY/2;
    // build inverse K^-1 of camera calibration matrix K
    this->cameraCalibrationMatrixInverse << 1./dX, -alpha/(dX*dY), (alpha*vp - dY*up)/(dX*dY),
                                            0., 1./dY, -vp/dY,
                                            0., 0., 1.;
}

/*! Compute the asteroids-cameras frame DCM between images (equation 10 in document)
    @return void
*/
void VisualOdometry::computeAframeDCM()
{
    double CB_temp[3][3];
    MRP2C(this->cameraBuffer.sigma_CB, CB_temp);
    Eigen::Matrix3d CB = cArray2EigenMatrix3d(*CB_temp);

    double Bkmin1N_temp[3][3];
    MRP2C(this->keyPointBuffer.sigma_BN_firstImage, Bkmin1N_temp);
    Eigen::Matrix3d Bkmin1N = cArray2EigenMatrix3d(*Bkmin1N_temp);

    double NTkmin1_temp[3][3];
    MRP2C(this->keyPointBuffer.sigma_TN_firstImage, NTkmin1_temp);
    Eigen::Matrix3d NTkmin1 = cArray2EigenMatrix3d(*NTkmin1_temp).transpose();

    double BkmN_temp[3][3];
    MRP2C(this->keyPointBuffer.sigma_BN_secondImage, BkmN_temp);
    Eigen::Matrix3d BkN = cArray2EigenMatrix3d(*BkmN_temp);

    double NTk_temp[3][3];
    MRP2C(this->keyPointBuffer.sigma_TN_secondImage, NTk_temp);
    Eigen::Matrix3d NTk = cArray2EigenMatrix3d(*NTk_temp).transpose();

    this->CkCkmin1 = CB*BkN*NTk*(CB*Bkmin1N*NTkmin1).transpose();
}

/*! Compute the Sampson partials for future computations, equations 58 and 59 in the reference document
       @param const int: iterator through all of the features pairs
       @return void
*/
void VisualOdometry::computeSampsonPartials(int i)
{
    this->dhdu_firstImage.push_back(
            -eigenTilde(this->cameraCalibrationMatrixInverse*this->pixelCoord_secondImage.col(i))
            *this->CkCkmin1*this->cameraCalibrationMatrixInverse);
    this->dhdu_secondImage.push_back(
            eigenTilde(this->CkCkmin1*this->cameraCalibrationMatrixInverse*this->pixelCoord_firstImage.col(i))
            *this->cameraCalibrationMatrixInverse);
}

/*! Compute the Gamma for each feature pair, equation 69 in the reference document
      @param const int: iterator through all of the features pairs
      @return void
*/
void VisualOdometry::computeGammas(int i)
{
    Eigen::Matrix3d temp1 = this->cameraCalibrationMatrixInverse.transpose()*this->CkCkmin1.transpose();
    Eigen::Matrix3d temp2 = eigenTilde(this->cameraCalibrationMatrixInverse*this->pixelCoord_secondImage.col(i));
    Eigen::Matrix3d hi_partial = (temp1*temp2).transpose();
    Eigen::Vector3d h_i = hi_partial*this->pixelCoord_firstImage.col(i);

    Eigen::MatrixXd dhdksi(3,6);
    dhdksi.block(0,0,3,3) = this->dhdu_firstImage[i];
    dhdksi.block(0,3,3,3) = this->dhdu_secondImage[i];

    Eigen::VectorXd ksi_tilde(6);
    ksi_tilde.setOnes();
    Eigen::Vector3d h_tilde_i = h_i + dhdksi*ksi_tilde*this->deltaKsi_tilde;
    this->gammas.push_back(h_tilde_i*h_tilde_i.transpose());
}

/*! Compute the Ksis for each feature pair, equation 70 in the reference document
      @param const int:  iterator through all of the features pairs
      @return void
*/
void VisualOdometry::computeKsis(int i)
{
    this->ksis.push_back(this->dhdu_firstImage[i]*this->R_uv*this->dhdu_firstImage[i].transpose() +
            this->dhdu_secondImage[i]*this->R_uv*this->dhdu_secondImage[i].transpose());
}

/*! Compute the H^T H as the sum of gammas
      @return void
*/
void VisualOdometry::computeHTH(){
    this->HtransposeH.setZero();
    for (int i=0; i<this->numberFeatures; i++){
        this->HtransposeH += this->gammas[i];
    }
}

/*! Compute extract the last column of the V matrix from an SVD decomposition
      @param Eigen::Matrix3d : Matrix that needs to be svd'ed
      @return void
*/
Eigen::Vector3d VisualOdometry::svdLastColumn(Eigen::Matrix3d& A) const{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d Vmat = svd.matrixV();
    Eigen::Matrix3d Umat = svd.matrixU();
    Eigen::Vector3d singular = svd.singularValues();
    Eigen::DiagonalMatrix<double, 3> Dmat(singular(0), singular(1), singular(2));

    assert((Umat*Dmat*Vmat.transpose() - A).norm());
    assert(singular(0) >= singular(1) && singular(1) >= singular(2));
    return Vmat.col(2);
}

/*! Compute the R inverse, equation 74 from the reference document
      @param Eigen::Vector3d : current direction of motion estimate
      @return void
*/
void VisualOdometry::computeRinv(Eigen::Vector3d& sPrime){
    this->Rinv.setZero();
    for (int i=0; i<this->numberFeatures; i++){
        Eigen::Vector3d ksiS = this->ksis[i]*sPrime;
        this->sTKsiS.push_back(sPrime.dot(ksiS));
        Eigen::Vector3d gammaS = this->gammas[i]*sPrime;
        this->sTGammaS.push_back(sPrime.dot(gammaS));
        this->Rinv += this->gammas[i]/(this->sTKsiS[i]);
    }
}

/*! Compute X, equation 77 from the reference document
     @return void
*/
void VisualOdometry::computeX(){
    this->X = this->Rinv;
    for (int i=0; i<this->numberFeatures;i++){
        this->X -= this->sTGammaS[i]/pow(this->sTKsiS[i], 2)*this->ksis[i];
    }
}

/*! Perform cheirality test for sign of sprime, equation A8 from the reference document
      @param Eigen::Vector3d : current direction of motion estimate
      @return void
*/
int VisualOdometry::cheiralityTest(Eigen::Vector3d sPrime){
    Eigen::VectorXd y(6,1);
    y.head(3) << 0,0,0;
    y.tail(3) = -eigenTilde(this->CkCkmin1*this->cameraCalibrationMatrixInverse*this->pixelCoord_firstImage.col(0))*sPrime;

    Eigen::MatrixXd A(6, 3);
    A.block(0, 0, 3, 3) =
            eigenTilde(this->cameraCalibrationMatrixInverse*this->pixelCoord_secondImage.col(0));
    A.block(3, 0, 3, 3) =
            eigenTilde(this->CkCkmin1*this->cameraCalibrationMatrixInverse*this->pixelCoord_firstImage.col(0));
    Eigen::Vector3d l_prime_0 = A.colPivHouseholderQr().solve(y);

    return (l_prime_0(2) > 0) - (l_prime_0(2) < 0);
}

/*! Compute covariance using an SVD decomposition of Rinv, equation 82 from the reference document
     @return void
*/
Eigen::Matrix3d VisualOdometry::computeCovariance() const{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->Rinv, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d singularValues = svd.singularValues();
    Eigen::MatrixXd Umat = svd.matrixU();
    Eigen::MatrixXd Vmat = svd.matrixV();

    assert(singularValues(0) > this->errorTolerance && singularValues(1) > this->errorTolerance);
    assert(singularValues(0) >= singularValues(1) && singularValues(1) >= singularValues(2));

    Eigen::Matrix3d covar;
    covar << 1/singularValues(0), 0, 0,
                    0, 1/singularValues(1), 0,
                    0, 0, 0;
    return Vmat*covar*Umat.transpose();
}

/*! Read the input messages each call of updateState. Check if the message contents are valid for this module.
    @return void
*/
bool VisualOdometry::readMessages()
{
    this->keyPointBuffer = this->keyPointPairInMsg();
    this->cameraBuffer = this->cameraConfigInMsg();

    if (this->keyPointBuffer.valid) {
        this->numberFeatures = this->keyPointBuffer.keyPointsFound;
        this->gammas.reserve(this->numberFeatures);
        this->ksis.reserve(this->numberFeatures);
        this->sTKsiS.reserve(this->numberFeatures);
        this->sTGammaS.reserve(this->numberFeatures);
        this->dhdu_firstImage.reserve(this->numberFeatures);
        this->dhdu_secondImage.reserve(this->numberFeatures);

        this->pixelCoord_firstImage.resize(3, this->numberFeatures);
        this->pixelCoord_secondImage.resize(3, this->numberFeatures);
        for (int i = 0; i < this->numberFeatures; i++) {
            this->pixelCoord_firstImage.col(i) << this->keyPointBuffer.keyPoints_firstImage[2 * i],
                    this->keyPointBuffer.keyPoints_firstImage[2 * i + 1],
                    1;
            this->pixelCoord_secondImage.col(i) << this->keyPointBuffer.keyPoints_secondImage[2 * i],
                    this->keyPointBuffer.keyPoints_secondImage[2 * i + 1],
                    1;
        }

        this->computeCameraMatrix();
        this->computeAframeDCM();

        this->R_uv << this->sigma_uv * this->sigma_uv, 0, 0,
                0, this->sigma_uv * this->sigma_uv, 0,
                0, 0, 0;
    }
    return this->keyPointBuffer.valid;
}

/*! This method writes the output messages each call of updateState
    @param Eigen::Vector3d final direction of motion estimate
    @param Eigen::Matrix3d final covriance of direction of motion estimate
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void VisualOdometry::writeMessages(Eigen::Vector3d sprime, Eigen::Matrix3d covar, uint64_t currentSimNanos)
{
    this->dirMotionBuffer.valid = this->keyPointBuffer.valid;
    this->dirMotionBuffer.cameraID = this->cameraBuffer.cameraID;
    this->dirMotionBuffer.timeOfDirectionEstimate = this->keyPointBuffer.timeTag_secondImage;
    eigenVector3d2CArray(sprime, this->dirMotionBuffer.v_C_hat );
    eigenMatrix3d2CArray(covar, this->dirMotionBuffer.covar_C);
    this->dirOfMotionMsgOutput.write(&this->dirMotionBuffer, this->moduleID, currentSimNanos);
}
