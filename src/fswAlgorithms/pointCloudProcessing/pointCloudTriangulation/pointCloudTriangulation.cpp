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

#include "fswAlgorithms/pointCloudProcessing/pointCloudTriangulation/pointCloudTriangulation.h"
#include <cmath>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
PointCloudTriangulation::PointCloudTriangulation()
{
    this->numberTimesCalled = 0;
    this->initialPhase = true;
}

/*! Module Destructor */
PointCloudTriangulation::~PointCloudTriangulation() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void PointCloudTriangulation::Reset(uint64_t currentSimNanos)
{
    // check that required input messages are connected
    if (!this->directionOfMotionInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "pointCloudTriangulation.directionOfMotionInMsg was not linked.");
    }
    if (!this->keyPointsInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "pointCloudTriangulation.keyPointsInMsg was not linked.");
    }
    if (!this->cameraConfigInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "pointCloudTriangulation.cameraConfigInMsg was not linked.");
    }
}

/*! This is the main method that gets called every time the module is updated.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void PointCloudTriangulation::UpdateState(uint64_t currentSimNanos)
{
    if (this->numberTimesCalled >= this->numberTimeStepsInitialPhase) {
        this->initialPhase = false;
    }

    // read messages
    this->readMessages();

    if (this->valid) {
        Eigen::Vector3d p1_C1 = Eigen::Vector3d::Zero();
        double dt = (this->timeTag2 - this->timeTag1)*NANO2SEC;
        Eigen::Vector3d p2_C1 = this->vScaleFactor*dt*this->v_C1_hat;

        std::vector<Eigen::Vector3d> cameraLocations = {p1_C1, p2_C1};
        std::vector<Eigen::Matrix3d> dcmCamera = {Eigen::Matrix3d::Identity(), this->dcm_C2C1};

        this->measuredPointCloud.clear();
        for (int c = 0; c < this->numberKeyPoints; ++c) {
            std::vector<Eigen::Vector2d> imagePoints = {this->keyPoints1.at(c), this->keyPoints2.at(c)};
            Eigen::Vector3d featureLocation = this->triangulation(
                    cameraLocations,
                    imagePoints,
                    this->cameraCalibrationMatrixInverse,
                    dcmCamera
            );
            this->measuredPointCloud.emplace_back(featureLocation);
        }
        this->pointCloudSize = this->numberKeyPoints;
        this->numberTimesCalled += 1;
    }

    // write messages
    this->writeMessages(currentSimNanos);
}

/*! This method reads the input messages each call of updateState.
    It also checks if the message contents are valid for this module.
    @return void
*/
void PointCloudTriangulation::readMessages()
{
    EphemerisMsgPayload ephemerisInMsgBuffer = this->ephemerisInMsg();
    NavTransMsgPayload navTransInMsgBuffer = this->navTransInMsg();
    DirectionOfMotionMsgPayload directionOfMotionInMsgBuffer = this->directionOfMotionInMsg();
    PairedKeyPointsMsgPayload keyPointsInMsgBuffer = this->keyPointsInMsg();
    CameraConfigMsgPayload cameraConfigInMsgBuffer = this->cameraConfigInMsg();

    /* velocity information either from ephemeris message or nav message */
    uint64_t timeTagVelocityInfo{};
    if (this->initialPhase) {
        this->vScaleFactor = cArray2EigenVector3d(ephemerisInMsgBuffer.v_BdyZero_N).norm();
        timeTagVelocityInfo = ephemerisInMsgBuffer.timeTag * SEC2NANO;
    } else {
        this->vScaleFactor = cArray2EigenVector3d(navTransInMsgBuffer.v_BN_N).norm();
        timeTagVelocityInfo = navTransInMsgBuffer.timeTag * SEC2NANO;
    }

    /* direction of motion message */
    bool validDOM = directionOfMotionInMsgBuffer.valid;
    uint64_t timeTagDOM = directionOfMotionInMsgBuffer.timeOfDirectionEstimate;
    this->v_C1_hat = cArray2EigenVector3d(directionOfMotionInMsgBuffer.v_C_hat);

    /* key points message */
    bool validKeyPoints = keyPointsInMsgBuffer.valid;
    this->numberKeyPoints = keyPointsInMsgBuffer.keyPointsFound;
    int64_t cameraIDkeyPoints = keyPointsInMsgBuffer.cameraID;

    // stacked vector with all pixel locations of key points for 1st camera location
    Eigen::VectorXd keyPointsStacked1(2*this->numberKeyPoints);
    keyPointsStacked1 = cArray2EigenMatrixXd(keyPointsInMsgBuffer.keyPoints_firstImage,
                                             2*this->numberKeyPoints,
                                             1);
    // convert to std vector that includes all key point pixel locations
    this->keyPoints1.clear();
    for (int c=0; c < this->numberKeyPoints; c++) {
        // 2D pixel location
        this->keyPoints1.emplace_back(keyPointsStacked1.segment(2*c, 2));
    }
    this->timeTag1 = keyPointsInMsgBuffer.timeTag_firstImage;
    // dcm from inertial frame N to body frame B
    Eigen::MRPd sigma_B1N = cArray2EigenMRPd(keyPointsInMsgBuffer.sigma_BN_firstImage);
    Eigen::Matrix3d dcm_B1N = sigma_B1N.toRotationMatrix().transpose();

    // stacked vector with all pixel locations of key points for 2nd camera location
    Eigen::VectorXd keyPointsStacked2(2*this->numberKeyPoints);
    keyPointsStacked2 = cArray2EigenMatrixXd(keyPointsInMsgBuffer.keyPoints_secondImage,
                                             2*this->numberKeyPoints,
                                             1);
    // convert to std vector that includes all key point pixel locations
    this->keyPoints2.clear();
    for (int c=0; c < this->numberKeyPoints; c++) {
        // 2D pixel location
        this->keyPoints2.emplace_back(keyPointsStacked2.segment(2*c, 2));
    }
    this->timeTag2 = keyPointsInMsgBuffer.timeTag_secondImage;
    // dcm from inertial frame N to body frame B
    Eigen::MRPd sigma_B2N = cArray2EigenMRPd(keyPointsInMsgBuffer.sigma_BN_secondImage);
    Eigen::Matrix3d dcm_B2N = sigma_B2N.toRotationMatrix().transpose();

    /* camera config message */
    int64_t cameraIDconfig = cameraConfigInMsgBuffer.cameraID;
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

    // dcm from first image camera frame C1 to second image camera frame C2
    Eigen::Matrix3d dcm_C1N = dcm_CB*dcm_B1N;
    Eigen::Matrix3d dcm_C2N= dcm_CB*dcm_B2N;
    this->dcm_C2C1 = dcm_C2N*dcm_C1N.transpose();

    this->valid = false;
    if (validDOM && validKeyPoints) {
        this->valid = true;
    }

    // check if cameraIDs and time tags are equal
    if (cameraIDkeyPoints != cameraIDconfig && this->valid) {
        bskLogger.bskLog(BSK_ERROR, "pointCloudTriangulation: camera IDs from keyPointsInMsg and "
                                    "cameraConfigInMsg are different, but should be equal.");
        this->valid = false;
    }
    if (!(timeTagDOM == this->timeTag2) && this->valid) {
        bskLogger.bskLog(BSK_ERROR, "pointCloudTriangulation: time tags from ephemerisInMsg, "
                                    "navTransInMsg, directionOfMotionInMsg and the time tag of the first image from "
                                    "keyPointsInMsg (timeTag_firstImage) are different, but should be equal.");
        this->valid = false;
    }
}

/*! This method writes the output messages each call of updateState
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void PointCloudTriangulation::writeMessages(uint64_t currentSimNanos)
{
    PointCloudMsgPayload pointCloudOutMsgBuffer;
    pointCloudOutMsgBuffer = this->pointCloudOutMsg.zeroMsgPayload;

    pointCloudOutMsgBuffer.timeTag = this->timeTag2;
    pointCloudOutMsgBuffer.valid = this->valid;
    pointCloudOutMsgBuffer.numberOfPoints = this->pointCloudSize;

    // stacked vector with all feature locations of point cloud
    Eigen::VectorXd pointCloudStacked(POINT_DIM*this->pointCloudSize);
    for (int c=0; c < this->pointCloudSize; c++) {
        pointCloudStacked.segment(POINT_DIM*c, POINT_DIM) = this->measuredPointCloud.at(c);
    }
    eigenMatrixXd2CArray(pointCloudStacked, pointCloudOutMsgBuffer.points);

    this->pointCloudOutMsg.write(&pointCloudOutMsgBuffer, this->moduleID, currentSimNanos);
}

/*! This method performs the triangulation to estimate the unknown location given the known locations and images
    @param knownLocations known positions (point cloud points)
    @param imagePoints location of key points in pixel space
    @param cameraCalibrationInverse inverse of camera calibration matrix K
    @param dcmCamera dcm from frame of interest F to camera frame C
    @return Eigen::Vector3d
*/
Eigen::Vector3d PointCloudTriangulation::triangulation(std::vector<Eigen::Vector3d> knownLocations,
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
