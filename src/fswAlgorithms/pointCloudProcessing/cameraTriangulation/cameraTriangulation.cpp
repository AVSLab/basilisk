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
        this->estimatedCameraLocation = this->triangulation(
                this->pointCloud,
                this->keyPoints,
                this->cameraCalibrationMatrixInverse,
                std::vector<Eigen::Matrix3d>{this->dcm_CN}
        );
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
    // stacked vector with all feature locations of point cloud
    Eigen::VectorXd pointCloudStacked(POINT_DIM*pointCloudSize);
    pointCloudStacked = cArray2EigenMatrixXd(pointCloudInMsgBuffer.points, POINT_DIM*pointCloudSize, 1);
    // convert to std vector that includes all point cloud feature locations
    this->pointCloud.clear();
    for (int c=0; c < pointCloudSize; c++) {
        // point of point cloud
        this->pointCloud.emplace_back(pointCloudStacked.segment(POINT_DIM*c, POINT_DIM));
    }
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
    this->cameraCalibrationMatrixInverse << 1./dX, -alpha/(dX*dY), (alpha*vp - dY*up)/(dX*dY),
                                            0., 1./dY, -vp/dY,
                                            0., 0., 1.;

    // dcm from inertial frame N to camera frame C
    this->dcm_CN = dcm_CB*dcm_BN;

    this->validInputs = false;
    if (validPointCloud && validKeyPoints) {
        this->validInputs = true;
    }

    // check if cameraIDs, time tags, and number of features are equal
    if (cameraIDkeyPoints == cameraIDconfig) {
        this->cameraID = cameraIDkeyPoints;
    } else {
        bskLogger.bskLog(BSK_ERROR, "cameraTriangulation: camera IDs from keyPointsInMsg and "
                                    "cameraConfigInMsg are different, but should be equal.");
        this->validInputs = false;
    }
    if (timeTagPointCloud == timeTagKeyPoints) {
        this->timeTag = timeTagPointCloud;
    } else {
        bskLogger.bskLog(BSK_ERROR, "cameraTriangulation: time tags from pointCloudInMsg and "
                                    "keyPointsInMsg (timeTag_secondImage) are different, but should be equal.");
        this->validInputs = false;
    }
    if (pointCloudSize != numberKeyPoints) {
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

    this->cameraLocationOutMsg.write(&cameraLocationOutMsgBuffer, this->moduleID, currentSimNanos);
}

/*! This method performs the triangulation to estimate the unknown location given the known locations and images
    @param knownLocations known positions (point cloud points)
    @param imagePoints location of key points in pixel space
    @param cameraCalibrationInverse inverse of camera calibration matrix K
    @param dcmCamera dcm from frame of interest F to camera frame C
    @return Eigen::Vector3d
*/
Eigen::Vector3d CameraTriangulation::triangulation(std::vector<Eigen::Vector3d> knownLocations,
                                                       std::vector<Eigen::Vector2d> imagePoints,
                                                       const Eigen::Matrix3d& cameraCalibrationInverse,
                                                       std::vector<Eigen::Matrix3d> dcmCamera) const
{
    unsigned long numLocations = knownLocations.size();
    unsigned long numImagePoints = imagePoints.size();
    unsigned long numDCM = dcmCamera.size();

    // make sure number of provided locations is equal to number of image points.
    // number of DCMs must be either 1 or also equal to number of image points
    assert(numLocations == numImagePoints && (numDCM == 1 || numDCM == numImagePoints));

    Eigen::Matrix3d dcm_CF = dcmCamera.at(0); // dcm from frame of interest F to camera frame C

    Eigen::MatrixXd A(3*numLocations, 3);
    Eigen::VectorXd y(3*numLocations);

    for (int c = 0; c < numLocations; ++c) {
        // update dcm in case they are different for each image point
        if (dcmCamera.size() != 1) {
            dcm_CF = dcmCamera.at(c);
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
        Eigen::Vector3d p = knownLocations.at(c);
        // fill in A matrix and measurements y
        A.block(3*c, 0, 3, 3) = eigenTilde(xBar)*dcm_CF;
        y.segment(3*c, 3) = eigenTilde(xBar)*dcm_CF*p;
    }
    // solve linear least squares to find unknown location r
    Eigen::Vector3d r = A.colPivHouseholderQr().solve(y);

    return r;
}
