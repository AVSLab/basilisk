/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#ifndef PINHOLE_CAMERA_H
#define PINHOLE_CAMERA_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/LandmarkMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief Pinhole camera class */
class PinholeCamera:  public SysModel {
public:
    PinholeCamera();
    ~PinholeCamera();
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentClock);

    void addLandmark(Eigen::Vector3d& pos, Eigen::Vector3d& normal);
    void processBatch(Eigen::MatrixXd rBatch_BP_P, Eigen::MatrixXd mrpBatch_BP, Eigen::MatrixXd eBatch_SP_P, bool show_progress);

private:
    void processInputs();
    void loopLandmarks();
    bool checkLighting(Eigen::Vector3d nLmk);
    bool checkFOV(Eigen::Vector2i pLmk, Eigen::Vector3d rLmk, Eigen::Vector3d nLmk);
    Eigen::Vector2i computePixel(Eigen::Vector3d rLmk);

public:
    /* Pinhole camera properties */
    double f; //!< [m] camera focal length
    double FOVx; //!< [rad] horizontal field of view
    double FOVy; //!< [rad] vertical field of view
    double nxPixel; //!< [-] number of horizontal pixels
    double nyPixel; //!< [-] number of vertical pixels
    double wPixel; //!< [m] pixel width
    Eigen::Matrix3d dcm_CB; //!< [-] dcm from body to camera

    /* Module outputs */
    Eigen::VectorXi isvisibleLmk; //!< [-] flag telling if a landmark is visible
    Eigen::MatrixXi pixelLmk; //!< [-] pixels for landmarks

    /* Landmark distribution */
    std::vector<Eigen::Vector3d> r_LP_P; //!< [m] landmark positions in planet frame
    std::vector<Eigen::Vector3d> nL_P; //!< [-] landmark normals in planet frame

    double maskSun; //!< [-] minimum slant range for Sun lighting

    /* Messages definition */
    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg; //!< planet ephemeris input message
    ReadFunctor<SCStatesMsgPayload> scStateInMsg; //!< spacecraft state input msg

    SCStatesMsgPayload spacecraftState; //!< input spacecraft state
    EphemerisMsgPayload ephemerisPlanet;  //!< input planet ephemeris
    std::vector<Message<LandmarkMsgPayload>*> landmarkOutMsgs; //!< vector of landmark messages
    std::vector<LandmarkMsgPayload> landmarkMsgBuffer; //!< buffer of landmark output data

    BSKLogger bskLogger;         //!< -- BSK Logging

    /* Batch variables */
    Eigen::MatrixXi pixelBatchLmk; //!< [-] batch of landmark pixels
    Eigen::MatrixXi isvisibleBatchLmk; //!< [-] batch of landmark visibility stauts

private:
    int n; //!< [-] number of landmarks

    /* Positions */
    Eigen::Vector3d r_PN_N; //!< [m] current planet position in inertial frame
    Eigen::Vector3d r_BP_P; //!< [m] current spacecraft position w.r.t. planet in planet frame

    /* Direction cosine matrices */
    Eigen::Matrix3d dcm_BP; //!< [-] current direction cosine matrix from planet to spacecraft body frame
    Eigen::Matrix3d dcm_PN; //!< [-] current direction cosine matrix from inertial to planet frame

    /* Unit-vectors */
    Eigen::Vector3d e_SP_P; //!< [-] current unit-vector pointing from planet to Sun in planet frame
    Eigen::Vector3d eC_C; //!< [-] focal direction unit-vector in camera frame
    Eigen::Vector3d eC_P; //!< [-] focal direction unit-vector in planet frame
};

#endif
