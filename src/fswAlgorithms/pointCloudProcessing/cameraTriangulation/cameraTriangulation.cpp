/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#include "fswAlgorithms/pointCloudProcessing/cameraTriangulation/cameraTriangulation.h"
#include <cmath>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
CameraTriangulation::CameraTriangulation() = default;

/*! Module Destructor */
CameraTriangulation::~CameraTriangulation() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void CameraTriangulation::Reset(uint64_t currentSimNanos)
{
    // check that required input messages are connected
    if (!this->pointCloudInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "cameraTriangulation.pointCloudInMsg was not linked.");
    }
    if (!this->keyPointsInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "cameraTriangulation.keyPointsInMsg was not linked.");
    }
    if (!this->cameraConfigInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "cameraTriangulation.cameraConfigInMsg was not linked.");
    }

    this->Ru = pow(this->uncertaintyImageMeasurement,2) * Eigen::Matrix2d::Identity();
}

/*! This is the main method that gets called every time the module is updated.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void CameraTriangulation::UpdateState(uint64_t currentSimNanos)
{
    // read messages
    this->readMessages();

    if (this->validInputs == 1) {
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> triangulationResult = this->triangulation(
                this->pointCloud,
                this->keyPoints,
                this->cameraCalibrationMatrixInverse,
                std::vector<Eigen::Matrix3d>{this->dcm_CN}
        );
        this->estimatedCameraLocation = triangulationResult.first;
        this->triangulationCovariance = triangulationResult.second;
    }

    // write messages
    this->writeMessages(currentSimNanos);
}

/*! This method reads the input messages each call of updateState.
    It also checks if the message contents are valid for this module.
    @return void
*/
void CameraTriangulation::readMessages()
{
    PointCloudMsgPayload pointCloudInMsgBuffer = this->pointCloudInMsg();
    PairedKeyPointsMsgPayload keyPointsInMsgBuffer = this->keyPointsInMsg();
    CameraConfigMsgPayload cameraConfigInMsgBuffer = this->cameraConfigInMsg();

    /* point cloud message */
    bool validPointCloud = pointCloudInMsgBuffer.valid;
    int pointCloudSize = pointCloudInMsgBuffer.numberOfPoints;
    this->pointCloud = cArray2EigenMatrixXd(pointCloudInMsgBuffer.points, POINT_DIM, pointCloudSize);
    uint64_t timeTagPointCloud = pointCloudInMsgBuffer.timeTag;

    /* key points message */
    bool validKeyPoints = keyPointsInMsgBuffer.valid;
    int numberKeyPoints = keyPointsInMsgBuffer.keyPointsFound;
    int cameraIDkeyPoints = keyPointsInMsgBuffer.cameraID;
    // stacked vector with all pixel locations of key points
    Eigen::VectorXd keyPointsStacked(2*numberKeyPoints);
    keyPointsStacked = cArray2EigenMatrixXd(keyPointsInMsgBuffer.keyPoints_secondImage, 2*numberKeyPoints, 1);
    // convert to std vector that includes all key point pixel locations
    this->keyPoints.clear();
    for (int c=0; c < numberKeyPoints; c++) {
        // 2D pixel location
        this->keyPoints.emplace_back(keyPointsStacked.segment(2*c, 2));
    }
    uint64_t timeTagKeyPoints = keyPointsInMsgBuffer.timeTag_secondImage;
    // dcm from inertial frame N to body frame B
    this->sigma_BN = cArray2EigenMRPd(keyPointsInMsgBuffer.sigma_BN_secondImage);
    Eigen::Matrix3d dcm_BN = this->sigma_BN.toRotationMatrix().transpose();

    /* camera config message */
    int cameraIDconfig = cameraConfigInMsgBuffer.cameraID;
    // dcm from body frame B to camera frame C
    Eigen::MRPd sigma_CB = cArray2EigenMRPd(cameraConfigInMsgBuffer.sigma_CB);
    Eigen::Matrix3d dcm_CB = sigma_CB.toRotationMatrix().transpose();
    // camera parameters
    double alpha = 0;
    double fieldOfView = cameraConfigInMsgBuffer.fieldOfView;
    double resolutionX = cameraConfigInMsgBuffer.resolution[0];
    double resolutionY = cameraConfigInMsgBuffer.resolution[1];
    double pX = 2.*tan(fieldOfView*resolutionX/resolutionY/2.0);
    double pY = 2.*tan(fieldOfView/2.0);
    double dX = resolutionX/pX;
    double dY = resolutionY/pY;
    double up = resolutionX/2;
    double vp = resolutionY/2;
    // build inverse K^-1 of camera calibration matrix K
    Eigen::Matrix3d Kinv;
    Kinv << 1./dX, -alpha/(dX*dY), (alpha*vp - dY*up)/(dX*dY),
                                            0., 1./dY, -vp/dY,
                                            0., 0., 1.;
    this->cameraCalibrationMatrixInverse = Kinv;

    Eigen::MatrixXd S(2,3);
    S << Eigen::Matrix2d::Identity(), Eigen::MatrixXd::Zero(2, 1);
    this->Rx = Kinv*S.transpose()*this->Ru*S*Kinv.transpose();

    // dcm from inertial frame N to camera frame C
    this->dcm_CN = dcm_CB*dcm_BN;

    this->validInputs = false;
    if (validPointCloud && validKeyPoints) {
        this->validInputs = true;
    }

    // check if cameraIDs, time tags, and number of features are equal
    if (cameraIDkeyPoints == cameraIDconfig) {
        this->cameraID = cameraIDkeyPoints;
    } else if(this->validInputs) {
        bskLogger.bskLog(BSK_ERROR, "cameraTriangulation: camera IDs from keyPointsInMsg and "
                                    "cameraConfigInMsg are different, but should be equal.");
        this->validInputs = false;
    }
    if (timeTagPointCloud == timeTagKeyPoints) {
        this->timeTag = timeTagPointCloud;
    }  else if(this->validInputs) {
        bskLogger.bskLog(BSK_ERROR, "cameraTriangulation: time tags from pointCloudInMsg and "
                                    "keyPointsInMsg (timeTag_secondImage) are different, but should be equal.");
        this->validInputs = false;
    }
    if (pointCloudSize != numberKeyPoints && this->validInputs) {
        bskLogger.bskLog(BSK_ERROR, "cameraTriangulation: number of features from pointCloudInMsg "
                                    "(pointCloudSize) and keyPointsInMsg (numberKeyPoints) are different, "
                                    "but should be equal.");
        this->validInputs = false;
    }
}

/*! This method writes the output messages each call of updateState
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void CameraTriangulation::writeMessages(uint64_t currentSimNanos)
{
    CameraLocalizationMsgPayload cameraLocationOutMsgBuffer;
    cameraLocationOutMsgBuffer = this->cameraLocationOutMsg.zeroMsgPayload;

    cameraLocationOutMsgBuffer.valid = this->validInputs;
    cameraLocationOutMsgBuffer.cameraID = this->cameraID;
    cameraLocationOutMsgBuffer.timeTag = this->timeTag;
    Eigen::Vector3d sig_BN = eigenMRPd2Vector3d(this->sigma_BN);
    eigenVector3d2CArray(sig_BN, cameraLocationOutMsgBuffer.sigma_BN);
    eigenVector3d2CArray(this->estimatedCameraLocation, cameraLocationOutMsgBuffer.cameraPos_N);
    eigenMatrix3d2CArray(this->triangulationCovariance, cameraLocationOutMsgBuffer.covariance_N);

    this->cameraLocationOutMsg.write(&cameraLocationOutMsgBuffer, this->moduleID, currentSimNanos);
}

/*! This method performs the triangulation to estimate the unknown location given the known locations and images.
    The covariance matrix of the estimated unknown location is also computed.
    @param knownLocations known positions (point cloud points)
    @param imagePoints location of key points in pixel space
    @param cameraCalibrationInverse inverse of camera calibration matrix K
    @param dcmCamera dcm from frame of interest F to camera frame C
    @return std::pair<Eigen::Vector3d, Eigen::Matrix3d>
*/
std::pair<Eigen::Vector3d, Eigen::Matrix3d> CameraTriangulation::triangulation(
                                                                        Eigen::MatrixXd knownLocations,
                                                                        std::vector<Eigen::Vector2d> imagePoints,
                                                                        const Eigen::Matrix3d& cameraCalibrationInverse,
                                                                        std::vector<Eigen::Matrix3d> dcmCamera) const
{
    unsigned long numLocations = knownLocations.cols();
    unsigned long numImagePoints = imagePoints.size();
    unsigned long numDCM = dcmCamera.size();

    // make sure number of provided locations is equal to number of image points.
    // number of DCMs must be either 1 or also equal to number of image points
    assert(numLocations == numImagePoints && (numDCM == 1 || numDCM == numImagePoints));

    Eigen::Matrix3d dcm_CF = dcmCamera.at(0); // dcm from frame of interest F to camera frame C
    Eigen::Matrix3d dcm_FC = dcm_CF.transpose(); // dcm from camera frame C to frame of interest F

    Eigen::MatrixXd A(3*numLocations, 3);
    Eigen::VectorXd y(3*numLocations);
    Eigen::Matrix3d covarianceSumTerm = Eigen::Matrix3d::Zero();

    for (int c = 0; c < numLocations; ++c) {
        // update dcm in case they are different for each image point
        if (dcmCamera.size() != 1) {
            dcm_CF = dcmCamera.at(c);
            dcm_FC = dcm_CF.transpose();
        }

        // 2D pixel location
        Eigen::Vector2d u = imagePoints.at(c);
        // 3D pixel location
        Eigen::Vector3d uBar;
        uBar << u,
                1.;
        // transform from pixel space to [m] space
        Eigen::Vector3d xBar = cameraCalibrationInverse*uBar;
        // known point
        Eigen::Vector3d p = knownLocations.col(c);
        // fill in A matrix and measurements y
        A.block(3*c, 0, 3, 3) = eigenTilde(xBar)*dcm_CF;
        y.segment(3*c, 3) = eigenTilde(xBar)*dcm_CF*p;

        // covariance computation (perform only if image noise is non-zero for more efficiency)
        if (this->uncertaintyImageMeasurement != 0.) {
            // Choose index for "companion measurement"
            // According to https://doi.org/10.2514/1.G006989, this can be done arbitrarily, so simply choose next index
            int c2 = c + 1;
            if (c2 > numLocations - 1) {
                // if c is already the last measurement, choose c2 = 0
                c2 = 0;
            }

            // dcm for measurement j
            // (only equal to the dcm for measurement i if only a single dcm is provided to function)
            Eigen::Matrix3d dcm_FCj = dcm_FC;
            if (dcmCamera.size() != 1) {
                dcm_FCj = dcmCamera.at(c2).transpose();
            }
            Eigen::Vector2d uj = imagePoints.at(c2);
            Eigen::Vector3d ujBar;
            ujBar << uj,
                    1.;
            Eigen::Vector3d pj = knownLocations.col(c2);
            Eigen::Vector3d d = pj - p;
            Eigen::Vector3d a = dcm_FC * cameraCalibrationInverse * uBar;
            Eigen::Vector3d aj = dcm_FCj * cameraCalibrationInverse * ujBar;
            double gamma = (d.cross(aj)).norm() / (a.cross(aj)).norm();
            if ((a.cross(aj)).norm() < 1e-10) {
                // if keypoint i and j are almost identical, gamma could be inf, resulting in a NaN covariance matrix
                // set gamma = 0 if divisor is close to 0
                gamma = 0.;
            }
            Eigen::Matrix3d Re = -pow(gamma, 2) * eigenTilde(xBar) * this->Rx * eigenTilde(xBar);
            covarianceSumTerm += dcm_FC * eigenTilde(xBar) * Re * eigenTilde(xBar) * dcm_CF;
        }
    }
    // solve linear least squares to find unknown location r
    Eigen::Vector3d r = A.colPivHouseholderQr().solve(y);
    // compute corresponding covariance matrix
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    if (this->uncertaintyImageMeasurement != 0.) {
        Eigen::Matrix3d AAinv = (A.transpose()*A).inverse();
        P = -AAinv*covarianceSumTerm*AAinv;
    }

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> triangulationResult = {r, P};

    return triangulationResult;
}
