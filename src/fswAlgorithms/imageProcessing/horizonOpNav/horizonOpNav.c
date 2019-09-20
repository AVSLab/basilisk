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

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "horizonOpNav.h"



/*! This method transforms pixel, line, and diameter data into heading data for orbit determination or heading determination.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void SelfInit_horizonOpNav(HorizonOpNavData *configData, uint64_t moduleID)
{
    configData->stateOutMsgID = CreateNewMessage(configData->opNavOutMsgName,
                                                 sizeof(OpNavFswMsg),
                                                 "OpNavFswMsg",
                                                 moduleID);
    configData->noiseSF = 4;
}

/*! This method subscribes to the camera and circle messages
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void CrossInit_horizonOpNav(HorizonOpNavData *configData, uint64_t moduleID)
{
    configData->cameraConfigMsgID = subscribeToMessage(configData->cameraConfigMsgName,sizeof(CameraConfigMsg),moduleID);
    configData->limbInMsgID = subscribeToMessage(configData->limbInMsgName, sizeof(LimbOpNavMsg),moduleID);
    configData->attInMsgID = subscribeToMessage(configData->attInMsgName, sizeof(NavAttIntMsg),moduleID);
}

/*! This resets the module to original states.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Reset_horizonOpNav(HorizonOpNavData *configData, uint64_t callTime, uint64_t moduleID)
{

}

/*! This method reads in the camera and circle messages and extracts navigation data from them. It outputs the heading (norm and direction) to the celestial body identified in the inertial frame. It provides the heading to the most robust circle identified by the image processing algorithm.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Update_horizonOpNav(HorizonOpNavData *configData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    double dcm_NC[3][3], dcm_CB[3][3], dcm_BN[3][3], Q[3][3], B[3][3];
    double planetRad;
    double covar_In_C[3][3], covar_In_B[3][3], covar_In_N[3][3];
    CameraConfigMsg cameraSpecs;
    LimbOpNavMsg limbIn;
    OpNavFswMsg opNavMsgOut;
    NavAttIntMsg attInfo;
    memset(&cameraSpecs, 0x0, sizeof(CameraConfigMsg));
    memset(&attInfo, 0x0, sizeof(NavAttIntMsg));
    memset(&limbIn, 0x0, sizeof(LimbOpNavMsg));
    memset(&opNavMsgOut, 0x0, sizeof(OpNavFswMsg));

    /*! - read input messages */
    ReadMessage(configData->cameraConfigMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CameraConfigMsg), &cameraSpecs, moduleID);
    ReadMessage(configData->limbInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(LimbOpNavMsg), &limbIn, moduleID);
    ReadMessage(configData->attInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(NavAttIntMsg), &attInfo, moduleID);
    
    /*! Check the validity of the image*/
    if (limbIn.valid == 0){
        opNavMsgOut.valid = 0;
        WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                     &opNavMsgOut, moduleID);
        return;
    }
    /*! Create Q matrix, the square root inverse of the A matrix, eq (6) in Engineering Note*/
    if(configData->planetTarget ==1){
        planetRad = REQ_EARTH*1E3;//in m
        opNavMsgOut.planetID = configData->planetTarget;
    }
    if(configData->planetTarget ==2){
        planetRad = REQ_MARS*1E3;//in m
        opNavMsgOut.planetID = configData->planetTarget;
    }
    if(configData->planetTarget ==3){
        planetRad = REQ_JUPITER*1E3;//in m
        opNavMsgOut.planetID = configData->planetTarget;
    }
    mSetIdentity(Q, 3, 3);
    mScale(planetRad, Q, 3, 3, Q);
    /* Set the number of limb points for ease of use*/
    int32_t numPoints;
    double sigma_pix2, R_s[3][3];
    numPoints = limbIn.numLimbPoints;
    sigma_pix2 = cameraSpecs.resolution[0]/(configData->noiseSF*numPoints);
    
    /*! Build DCMs */
    configData->planetTarget = limbIn.planetIds;
    MRP2C(cameraSpecs.sigma_CB, dcm_CB);
    MRP2C(attInfo.sigma_BN, dcm_BN);
    m33MultM33(dcm_CB, dcm_BN, dcm_NC);
    m33Transpose(dcm_NC, dcm_NC);
    
    /*! - Find pixel size using camera specs */
    double d_x, d_y, u_p, v_p, tranf[3][3], alpha;
    double H[numPoints*3], s_bar[numPoints*2], s[3], R_y[numPoints*numPoints], J[3];
    int i;
    /* To do: replace alpha by a skew read from the camera message */
    alpha = 0;
    d_x = cameraSpecs.sensorSize[0]/cameraSpecs.resolution[0];
    d_y = cameraSpecs.sensorSize[1]/cameraSpecs.resolution[1];
    u_p = cameraSpecs.resolution[0]/2 - 0.5;
    v_p = cameraSpecs.resolution[1]/2 - 0.5;
    m33SetZero(tranf);
    /*! Set the noise matrix in pix eq (53) in Engineering Note*/
    m33Set(sigma_pix2/(d_x*d_x), 0, 0, 0, sigma_pix2/(d_x*d_x), 0, 0, 0, 0, R_s);
    /*! Rotate R_s with B eq (52) in Journal*/
    m33MultM33(B, R_s, R_s);
    m33MultM33t(R_s, B, R_s);
    mSetZero(R_y, numPoints, numPoints);


    /*! Set the map from pixel to position eq (8) in Journal*/
    m33Set(1/d_x, -alpha/(d_x*d_y), (alpha*v_p - d_y*u_p)/(d_x*d_y), 0, 1/d_y, -v_p/d_y, 0, 0, 1, tranf);

    /*! Create the H matrix. This is the stacked vector of all the limb points eq (33) in Engineering Note attached*/
    for (i=0; i<numPoints;i++){
        v3SetZero(s);
        /*! - Put the pixel data in s (not s currently)*/
        s[0] = limbIn.limbPoints[0];
        s[1] = limbIn.limbPoints[1];
        s[2] = 1;
        /*! - Apply the trasnformation computed previously from pixel to position*/
        m33MultV3(tranf, s, s);
        /*! - Rotate the Vector in the inertial frame*/
        m33MultM33(Q, dcm_NC, B);
        m33MultV3(B, s, s);
        /*! - We now have s_bar in the Journal Paper, store to later compute J for uncertainty*/
        v3Copy(s, &s_bar[3*i]);
        v3Normalize(s, s);
        v3Copy(s, &H[i*3]);
    }

    /*! Need to solve Hn = 1, for n. If we performa  QR decomp on H, the problem becomes:
     Rn = Q^T.1*/
    /*! Perform the QR decompostion of H, this will */
    double Q_decomp[numPoints*numPoints], R_decomp[3*3], jTemp[3];
    double RHS_vec[3], ones[numPoints], n[3], IminusOuter[3][3], outer[3][3], sNorm;
    double scaleFactor, nNorm2;
    nNorm2 = v3Dot(n, n);
    scaleFactor = -1./sqrt(nNorm2-1);
    
    QRDecomp(H, numPoints, 3, Q_decomp, R_decomp);
    /*! Backsup to get n */
    v3SetZero(RHS_vec);
    vSetOnes(ones, numPoints);
    mTranspose(Q_decomp, numPoints, numPoints, Q_decomp);
    mMultV(Q_decomp, numPoints, numPoints, ones, RHS_vec);
    BackSub(R_decomp, RHS_vec, 3, n);

    /*! - With all the s_bar terms, and n, we can compute J eq(50) in journal, and get uncertainty */
    for (i=0; i<numPoints;i++){
        v3SetZero(J);
        sNorm = v3Norm(&s_bar[3*i]);
        m33SetIdentity(IminusOuter);
        /*! - Equation 31 in Journal*/
        v3OuterProduct(&s_bar[3*i], &s_bar[3*i], outer);
        m33Subtract(IminusOuter, outer, IminusOuter);
        /*! - Rotate the Vector in the inertial frame*/
        v3tMultM33(n, IminusOuter, J);
        v3Scale(1/sNorm, J, J);
        v3tMultM33(J, R_s, jTemp);
        R_y[numPoints*i+i] = v3Dot(jTemp, J);
    }
    
    /*! - Covar from least squares */
    double Pn[3][3], R_yInv[numPoints][numPoints], Rtemp[numPoints][3];
    double F[3][3];
    mInverse(R_y, numPoints, R_yInv);
    m33SetIdentity(Pn);
    mMultM(R_yInv, numPoints, numPoints, H, numPoints, 3, Rtemp);
    mtMultM(H, numPoints, 3, Rtemp, numPoints, 3, Pn);
    m33Inverse(Pn, Pn);
    
    /*! - Build F from eq (55) of engineering note */
    v3OuterProduct(n, n, outer);
    m33Scale(1/(nNorm2-1), outer, outer);
    m33SetIdentity(IminusOuter);
    m33Subtract(IminusOuter, outer, IminusOuter);
    
    /*! - Get the heading */
    m33Inverse(B, B);
    m33MultV3(B, n, n);
    v3Scale(scaleFactor, n, opNavMsgOut.r_BN_C);
    
    /*! - Build F from eq (55) of engineering note */
    m33MultM33t(B, IminusOuter, F);
    m33Scale(scaleFactor, F, F);
    m33MultM33(F, Pn, covar_In_C);
    m33MultM33t(covar_In_C, F, covar_In_C);
    
    /*! - Transform to desireable frames */
    m33MultV3(dcm_NC, opNavMsgOut.r_BN_C, opNavMsgOut.r_BN_N);
    m33MultV3(dcm_BN, opNavMsgOut.r_BN_N, opNavMsgOut.r_BN_B);
    m33MultM33(dcm_NC, covar_In_C, covar_In_N);
    m33MultM33t(covar_In_N, dcm_NC, covar_In_N);
    m33MultM33(dcm_BN, covar_In_N, covar_In_B);
    m33MultM33t(covar_In_B, dcm_BN, covar_In_B);


    /*! - write output message */
    mCopy(covar_In_N, 3, 3, opNavMsgOut.covar_N);
    mCopy(covar_In_C, 3, 3, opNavMsgOut.covar_C);
    mCopy(covar_In_B, 3, 3, opNavMsgOut.covar_B);
    opNavMsgOut.timeTag = limbIn.timeTag;
    opNavMsgOut.valid =1;
    WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpNavFswMsg),
                 &opNavMsgOut, moduleID);

    return;
}


/*! This performs a QR decomposition on a input matrix. In this method it's used on the H matrix made up of the limb points
 @return void
 @param inMat The input matrix to decompose
 @param nRow  The number of rows
 @param nRow  The number of columns
 @param Q     The output Q matrix
 @param R     The output R matrix
 */
void QRDecomp(double *inMat, int32_t nRow, int32_t nCol, double *Q , double *R)
{
    int32_t i, j, k, dimi, dimj;
    double sourceMat[nRow*nCol];
    double sum;
    
    mSetZero(Q, nRow, nRow);
    mSetZero(sourceMat, nRow, nCol);
    mSetZero(R, nCol, nCol);
    mCopy(inMat, nRow, nCol, sourceMat);
    
    for (i = 0; i<nCol; i++)
    {
        dimi = i*nCol + i;
        for (j = 0; j<nRow; j++)
        {
            dimj = nCol * j;
            R[dimi] += sourceMat[dimj + i] * sourceMat[dimj + i];
        }
        R[dimi] = sqrt(R[dimi]); /* Sum of squares, can't be negative */
        for (j = 0; j<nRow; j++)
        {
            Q[j*nCol + i] =
            sourceMat[j*nCol + i] / R[dimi];
        }
        
        for (j = i + 1; j<nCol; j++)
        {
            sum = 0;
            dimj = nCol * j;
            for (k = 0; k<nRow; k++)
            {
                sum += sourceMat[k*nCol + j] * Q[k*nCol + i];
            }
            R[i*nCol + j] = sum;
            for (k = 0; k<nRow; k++)
            {
                sourceMat[k*nCol + j] -= sum*Q[k*nCol + i];
            }
        }
    }
    
    return;
}

/*! This performs a backsubstitution solve. This methods solves for n given Rn = V with R an upper triangular matrix. 
 @return void
 @param R     The upper triangular matrix for the backsolve
 @param inVec Vector on the Right-Hand-Side of the Rn = V equation
 @param nRow  The number of rows/columns
 @param n     The solution vector
 */
void BackSub(double *R, double *inVec, int32_t nRow, double *n)
{
    int32_t i, j;
    double sum;
    
    vSetZero(n, nRow);
    n[nRow-1] = inVec[nRow-1]/R[nRow*nRow-1];
    for (i = nRow-2; i>=0; i--)
    {
        sum = 0;
        for (j = i + 1; j<nRow; j++)
        {
            sum += R[i*nRow + j] * n[j];
            
        }
        n[i] = (inVec[i] -sum)/R[i*nRow + i];
    }
    
    return;
}
