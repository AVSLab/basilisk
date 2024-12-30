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

#include "pinholeCamera.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <iostream>
#include <math.h>


/*! @brief Creates an instance of the PinholeCamera class with a prescribed focal direction in camera frame and -90º of Sun's mask angle (that is, no lighting constraint).

 */
PinholeCamera::PinholeCamera()
{
    /* Set focal direction to be +z axis in camera frame */
    this->eC_C << 0, 0, 1;

    /* Set no lighting conditions by default  */
    this->maskSun = -M_PI_2;
}

/*! Empty destructor method.

 */
PinholeCamera::~PinholeCamera()
{
    for (long unsigned int i = 0; i < this->landmarkOutMsgs.size(); i++){
        delete this->landmarkOutMsgs.at(i);
    }
    return;
}


/*! Resets the module.*/
void PinholeCamera::Reset(uint64_t CurrentSimNanos)
{
    /* Get number of landmarks */
    this->n = int(this->r_LP_P.size());

    /* Check that required input messages are connected */
    if (!this->scStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "PinholeCamera.scStateInMsg was not linked.");
    }
    if (!this->ephemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "PinholeCamera.ephemerisInMsg was not linked.");
    }

    /* Compute field of view */
    this->FOVx = 2 * atan2(this->nxPixel*this->wPixel, 2*this->f);
    this->FOVy = 2 * atan2(this->nyPixel*this->wPixel, 2*this->f);
}

/*! Method to add landmarks
 * @param pos: landmark position in planet-rotating frame
 * @param normal: landmark surface normal in planet-rotating frame
 */
void PinholeCamera::addLandmark(Eigen::Vector3d& pos, Eigen::Vector3d& normal){
    /* Add the landmark position and surface normal */
    this->r_LP_P.push_back(pos);
    this->nL_P.push_back(normal);

    /* Create buffer output messages */
    Message<LandmarkMsgPayload> *msg;
    msg = new Message<LandmarkMsgPayload>;
    this->landmarkOutMsgs.push_back(msg);

    /* Expand the landmark buffer vectors */
    LandmarkMsgPayload lmkMsg;
    this->landmarkMsgBuffer.push_back(lmkMsg);
}

/*! Loop through landmarks
*/
void PinholeCamera::loopLandmarks()
{
    /* Define ith landmark position, surface normal, camera pixel
     and field of view/lighting flags */
    Eigen::Vector3d rLmk, nLmk;
    Eigen::Vector2i pLmk;
    bool flagFOV, flagLighting;

    /* Preallocate visibility status and landmark pixels */
    this->isvisibleLmk.setZero(this->n);
    this->pixelLmk.setZero(this->n, 2);

    /* Loop through landmarks */
    for (int i = 0; i < this->n; i++){
        /* Extract ith landmark position and its normal */
        rLmk = this->r_LP_P[i];
        nLmk = this->nL_P[i];

        /* Compute pixel and check visbility/lighting conditions */
        pLmk = this->computePixel(rLmk);
        flagFOV = this->checkFOV(pLmk, rLmk, nLmk);
        flagLighting = this->checkLighting(nLmk);

        /* Add pixel */
        if (flagFOV && flagLighting){
            this->pixelLmk.row(i) = pLmk;
            this->isvisibleLmk(i) = 1;
        }
    }
}


/*! Check lighting condition of a landmark
 * @param nLmk: landmark surface normal in planet-rotating frame
*/
bool PinholeCamera::checkLighting(Eigen::Vector3d nLmk)
{
    /* Compute Sun's incident angle */
    double angleSun;
    angleSun = M_PI_2 - safeAcos(nLmk.dot(this->e_SP_P) / (this->e_SP_P.norm()*nLmk.norm()));

    /* Check if the landmark is illuminated */
    bool flagLighting;
    flagLighting = false;
    if (angleSun > this->maskSun){
        /* Set lighting condition to true */
        flagLighting = true;
    }

    return flagLighting;
}

/*! Check if a landmark is within camera field of view
 * @param pLmk: landmark pixel
 * @param rLmk: landmark position in planet-rotating frame
 * @param nLmk: landmark surface normal in planet-rotating frame
*/
bool PinholeCamera::checkFOV(Eigen::Vector2i pLmk, Eigen::Vector3d rLmk, Eigen::Vector3d nLmk){
    /* Define relative position from landmark to spacecraft and its unit-vector */
    Eigen::Vector3d dr, eLmk_P;
    dr = this->r_BP_P - rLmk;
    eLmk_P = dr / dr.norm();

    /* Define output and angles to check
     angleFOV1 checks if landmarks are in front of the camera
     angleFOV2 checks if the reflected light (landmark normal) is captured by camera
     The focal direction is taken negative so positive angles equal the truth condition */
    bool flagFOV;
    flagFOV = false;
    double angleFOV1, angleFOV2;
    angleFOV1 = M_PI_2 - safeAcos(-this->eC_P.dot(eLmk_P) / (this->eC_P.norm()*eLmk_P.norm()));
    angleFOV2 = M_PI_2 - safeAcos(-this->eC_P.dot(nLmk) / (this->eC_P.norm()*nLmk.norm()));

    /* If both angles are positive */
    if (angleFOV1 > 0 && angleFOV2 > 0){
        /* Check if landmark actually falls within camera field of view*/
        if ((abs(2*pLmk(0)) <= this->nxPixel) && (abs(2*pLmk(1)) <= this->nyPixel)){
            /* Set field of view flag to true */
            flagFOV = true;
        }
    }

    return flagFOV;
}

/*! Compute pixel from 3D coordinates
 * @param rLmk: landmark position in planet-rotating frame
*/
Eigen::Vector2i PinholeCamera::computePixel(Eigen::Vector3d rLmk)
{
    /* Compute relative distance between landmark and spacecraft
     in camera frame */
    Eigen::Vector3d dr;
    dr = this->dcm_CB * this->dcm_BP * (rLmk - this->r_BP_P);

    /* Compute pixel location */
    double u, v;
    Eigen::Vector2i p;
    u = this->f * dr(0) / (dr(2) * this->wPixel);
    v = this->f * dr(1) / (dr(2) * this->wPixel);

    /* Apply pixelization */
    if (u > 0){
        p(0) = int(ceil(u));
    }
    else if (u < 0){
        p(0) = int(floor(u));
    }
    else{
        p(0) = 1;
    }
    if (v > 0){
        p(1) = int(ceil(v));
    }
    else if (v < 0){
        p(1) = int(floor(v));
    }
    else{
        p(1) = 1;
    }

    return p;
}

/*! Process input messages to module variables
*/
void PinholeCamera::processInputs()
{
    /* Extract planet dcm */
    double dcm_PN_array[3][3];
    MRP2C(this->ephemerisPlanet.sigma_BN, dcm_PN_array);
    this->dcm_PN = cArray2EigenMatrix3d(*dcm_PN_array);

    /* Compute planet to Sun line e_SP on the planet rotating frame P */
    Eigen::Vector3d r_PS_N;
    r_PS_N = cArray2EigenVector3d(this->ephemerisPlanet.r_BdyZero_N);
    this->e_SP_P = this->dcm_PN * (-r_PS_N / r_PS_N.norm());

    /* Compute spacecraft position in the planet rotating frame P */
    this->r_BP_P = this->dcm_PN * (cArray2EigenVector3d(this->spacecraftState.r_BN_N) -  cArray2EigenVector3d(this->ephemerisPlanet.r_BdyZero_N));

    /* Compute spacecraft dcm with respect to planet rotating frame */
    double dcm_BN_array[3][3];
    MRP2C(this->spacecraftState.sigma_BN, dcm_BN_array);
    this->dcm_BP = cArray2EigenMatrix3d(*dcm_BN_array) * this->dcm_PN.transpose();

    /* Camera focal direction in planet frame */
    this->eC_P = (this->dcm_CB * this->dcm_BP).transpose()*this->eC_C;
}

/*! Read module input messages
*/
void PinholeCamera::readInputMessages()
{
    /* Read planet ephemeris and spacecraft state messages */
    this->ephemerisPlanet = this->ephemerisInMsg();
    this->spacecraftState = this->scStateInMsg();
}

/*! Write module output messages
*/
void PinholeCamera::writeOutputMessages(uint64_t CurrentClock)
{
    /* Loop through landmarks */
    for (int i = 0; i < this->n; i++){
        /* Zero the output message buffers */
        this->landmarkMsgBuffer.at(i) = this->landmarkOutMsgs.at(i)->zeroMsgPayload;

        /* Fill landmark output messages */
        this->landmarkMsgBuffer.at(i).isVisible = this->isvisibleLmk(i);
        eigenMatrixXi2CArray(this->pixelLmk.row(i), this->landmarkMsgBuffer.at(i).pL);
        this->landmarkOutMsgs.at(i)->write(&this->landmarkMsgBuffer.at(i), this->moduleID, CurrentClock);
    }
}


/*!
 Update module
 @param CurrentSimNanos
 */
void PinholeCamera::UpdateState(uint64_t CurrentSimNanos)
{
    /* Read messages */
    this->readInputMessages();

    /* Process planet ephemeris, spacecraft position
     and orientation */
    this->processInputs();

    /* Loop through landmarks */
    this->loopLandmarks();

    /* Write message */
    this->writeOutputMessages(CurrentSimNanos);
}

/*! Process a batch of inputs: spacecraft positions, orientation and unit-vector from planet to Sun.
 * @param rBatch_BP_P: batch of spacecraft position w.r.t. planet in planet-rotating frame
 * @param mrpBatch_BP: batch of spacecraft MRPs w.r.t. planet-rotating frame
 * @param eBatch_SP_P: batch unit-vectors from planet to Sun in planet-rotating frame
 * @param show_progress: boolean variable that prints batch processing status when true
*/
void PinholeCamera::processBatch(Eigen::MatrixXd rBatch_BP_P, Eigen::MatrixXd mrpBatch_BP, Eigen::MatrixXd eBatch_SP_P, bool show_progress){
    /* Obtain number of samples in the batch to be processed */
    int nBatch;
    nBatch = int(rBatch_BP_P.rows());

    /* Preallocate batches of pixel landmarks and visibility status. These
     are the variables the user should extract */
    this->pixelBatchLmk.setZero(nBatch, 2*this->n);
    this->isvisibleBatchLmk.setZero(nBatch, this->n);

    /* Define direction cosine matrix and MRP of the spacecraft body
     frame w.r.t. the planet frame */
    double dcm_BP_array[3][3];
    double mrp_BP[3];

    /* Print initialization */
    if (show_progress){
        std::cout << "--- Initialization of loop through " << nBatch << " samples ---" << '\n';
    }

    /* Loop through batch */
    for (int i = 0; i < nBatch; i++){
        /* Print progress after each 5% of the batch is completed */
        if (show_progress){
            if (i % (int(nBatch/20)) == 0){
                std::cout << i / (int(nBatch/100)) << "% completed" << '\n';
            }
        }

        /* Extract spacecraft position and Sun's unit-vector in
         planet frame */
        this->r_BP_P = rBatch_BP_P.row(i).transpose();
        this->e_SP_P = eBatch_SP_P.row(i).transpose();

        /* Compute spacecraft dcm w.r.t. planet frame */
        eigenMatrixXd2CArray(mrpBatch_BP.row(i).transpose(), mrp_BP);
        MRP2C(mrp_BP, dcm_BP_array);
        this->dcm_BP = cArray2EigenMatrix3d(*dcm_BP_array);

        /* Compute camera focal direction in planet frame */
        this->eC_P = (this->dcm_CB * this->dcm_BP).transpose()*this->eC_C;

        /* Loop through landmarks */
        this->loopLandmarks();

        /* Fill batches of pixel landmarks and visibility status */
        this->pixelBatchLmk.block(i, 0, 1, this->n) = this->pixelLmk.col(0).transpose();
        this->pixelBatchLmk.block(i, this->n, 1, this->n) = this->pixelLmk.col(1).transpose();
        this->isvisibleBatchLmk.row(i) = this->isvisibleLmk;
    }

    /* Declare initialization */
    if (show_progress){
        std::cout << "--- Batch has been processed ---" << '\n';
    }
}
