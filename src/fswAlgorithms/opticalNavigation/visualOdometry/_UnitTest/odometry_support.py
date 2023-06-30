# 
#  ISC License
# 
# Copyright (c) 2023, Laboratory for Atmospheric Space Physics, University of Colorado Boulder
# 
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
# 
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
# 
#

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

def  odometry(keyPointData, cameraData, sigma_uv = 0.01):
    points = []
    time1 = keyPointData.timeTag_firstImage
    time2 = keyPointData.timeTag_secondImage
    numFeatures = keyPointData.keyPointsFound
    points1 = np.array(keyPointData.keyPoints_firstImage[:2*numFeatures]).reshape([numFeatures, 2])
    points2 = np.array(keyPointData.keyPoints_secondImage[:2*numFeatures]).reshape([numFeatures, 2])

    BN1 = rbk.MRP2C(keyPointData.sigma_BN_firstImage)
    BN2 = rbk.MRP2C(keyPointData.sigma_BN_secondImage)
    TN1 = rbk.MRP2C(keyPointData.sigma_TN_firstImage)
    TN2 = rbk.MRP2C(keyPointData.sigma_TN_secondImage)

    fov = cameraData.fieldOfView
    res = cameraData.resolution
    CB = rbk.MRP2C(cameraData.sigma_CB)

    CkCkmin1 = np.dot(np.dot(np.dot(CB, BN2), TN2.T), np.dot(np.dot(CB, BN1), TN1.T).T)

    # Transf camera to meters
    alpha =0
    up = res[0] / 2
    vp = res[1] / 2
    pX = 2. * np.tan(fov / 2.0)
    pY = 2. * np.tan(fov * res[1] / res[0] / 2.0)
    d_x = res[0] / pX
    d_y = res[1] / pY

    CameraInv = np.zeros([3,3])
    CameraInv[0, 0] = 1 / d_x
    CameraInv[1, 1] = 1 / d_y
    CameraInv[2, 2] = 1
    CameraInv[0, 1] = -alpha/(d_x*d_y)
    CameraInv[0, 2] = (alpha*vp - d_y*up)/ (d_x * d_y)
    CameraInv[1, 2] = -vp / (d_y)

    dhdukmin1 = np.zeros([numFeatures, 3, 3])
    dhduk = np.zeros([numFeatures, 3, 3])
    gammas = np.zeros([numFeatures, 3, 3])
    ksis = np.zeros([numFeatures, 3, 3])
    sigmas = np.array([0.25]*6)

    Ruv = np.zeros([3,3])
    Ruv[:2,:2] = sigma_uv*sigma_uv*np.eye(2)
    max_iter = 10

    for i in range(numFeatures):
        ukmin1 = np.array([points1[i, 0], points1[i, 1], 1])
        uk = np.array([points2[i, 0], points2[i, 1], 1])
        dhdukmin1[i, :, :] = - np.dot(np.dot(rbk.v3Tilde(np.dot(CameraInv, uk)) , CkCkmin1) , CameraInv)
        dhduk[i, :, :] = np.dot(rbk.v3Tilde(np.dot(np.dot(CkCkmin1, CameraInv), ukmin1)) , CameraInv)
        dhdksi = np.zeros([3, 6])
        dhdksi[:, :3] = dhdukmin1[i, :, :]
        dhdksi[:, 3:] = dhduk[i, :, :]

        hi = np.dot(np.dot(ukmin1.T, CameraInv.T), np.dot(CkCkmin1.T, rbk.v3Tilde(np.dot(CameraInv, uk)))).T
        hi_tilde = hi + np.dot(dhdksi, sigmas)

        gammas[i, :, :] = np.outer(hi_tilde, hi_tilde)
        ksis[i, :, :] = np.dot(dhdukmin1[i, :, :], np.dot(Ruv, dhdukmin1[i, :, :].T)) + np.dot(dhduk[i, :, :], np.dot(Ruv, dhduk[i, :, :].T))

    HTH = np.zeros([3,3])
    for i in range(numFeatures):
        HTH += gammas[i, :, :]

    U, D, V = np.linalg.svd(HTH)
    indices = np.flip(np.argsort(D))
    sprime_min1 = V.T[indices, -1]
    Rinv = np.zeros([3,3])
    tol = 1E-5

    for m in range(max_iter):
        Rinv = np.zeros([3,3])
        for i in range(numFeatures):
            Rinv += gammas[i, :, :]/np.dot(np.dot(sprime_min1.T, ksis[i,:,:]), sprime_min1)
        X = Rinv
        for i in range(numFeatures):
            X -= np.dot(np.dot(sprime_min1.T, gammas[i, :, :]), sprime_min1)/ \
                  np.dot(np.dot(sprime_min1.T, ksis[i, :, :]), sprime_min1)**2 * ksis[i, :, :]
        U, D, V = np.linalg.svd(X)
        indices = np.flip(np.argsort(D))
        sprime = V.T[indices, -1]
        dS = np.linalg.norm(sprime - sprime_min1)
        if dS < tol:
            break
        else:
            sprime_min1 = sprime

    cheiralityRHS = np.zeros(6)
    cheiralityRHS[3:] = - np.dot(rbk.v3Tilde(np.dot(np.dot(CkCkmin1, CameraInv),
                                                    np.array([points1[i, 0], points1[i, 1], 1]))), sprime)
    cheiralityLHS = np.zeros([6, 3])
    cheiralityLHS[:3, :3] = rbk.v3Tilde(np.dot(CameraInv, np.array([points2[i, 0], points2[i, 1], 1])))
    cheiralityLHS[3:, :3] = rbk.v3Tilde(np.dot(CkCkmin1, np.dot(CameraInv, np.array([points1[i, 0], points1[i, 1], 1]))))
    Q, R = np.linalg.qr(cheiralityLHS)
    lk = np.dot(np.dot(np.linalg.inv(R), Q.T), cheiralityRHS)
    if lk[2] < 0:
        sprime = -sprime
    U, D, V = np.linalg.svd(Rinv)
    indices = np.flip(np.argsort(D))
    R = np.dot(np.dot(V[indices,:].T, np.array([[1/D[indices][0], 0,0],[0,1/D[indices][1], 0],[0,0,0]])), U[indices,:].T)
    return sprime, R
