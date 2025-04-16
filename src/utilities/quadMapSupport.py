
#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

# Import required resources
import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import vizSupport
try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass
from Basilisk.architecture import messaging

def computeRectMesh(parentBody, latBounds, lonBounds, maxAngWidth):
    """
    This method creates a polar/rectangular mesh on the surface of a reference ellipsoid.

    Parameters
    ----------
    parentBody:
        Parent grav body, containing ``radEquator`` and ``radiusRatio`` which define the
        reference ellipsoid.
    latBounds:
        [deg] Latitude bounds
    lonBounds:
        [deg] Longitude bounds
    maxAngWidth:
        [deg] Maximum angular width for sub-regions

    Returns
    -------
    3-element list
        r_GP_P, position vector of the location G relative to parent body frame P in P frame components

    """
    startLat = latBounds[0]
    endLat = latBounds[1]
    startLon = lonBounds[0]
    endLon = lonBounds[1]

    # Scale angular width to produce evenly sized sub-regions
    latList = np.deg2rad(np.linspace(startLat, endLat,
                                     int(np.ceil(abs(endLat - startLat) / maxAngWidth)) + 1))
    lonList = np.deg2rad(np.linspace(startLon, endLon,
                                     int(np.ceil(abs(endLon - startLon) / maxAngWidth)) + 1))

    # Extract planet radius and flattening
    p_RE = parentBody.radEquator
    p_RR = parentBody.radiusRatio

    vertices = []

    for lat1, lat2 in zip(latList, latList[1:]):
        for lon1, lon2 in zip(lonList, lonList[1:]):

            corner1 = vizSupport.lla2fixedframe([lat1, lon1, 0], p_RE, p_RR)
            vertices.extend(corner1)

            corner2 = vizSupport.lla2fixedframe([lat2, lon1, 0], p_RE, p_RR)
            vertices.extend(corner2)

            corner3 = vizSupport.lla2fixedframe([lat2, lon2, 0], p_RE, p_RR)
            vertices.extend(corner3)

            corner4 = vizSupport.lla2fixedframe([lat1, lon2, 0], p_RE, p_RR)
            vertices.extend(corner4)

    return vertices


def computeCamFOVBox(parentBody, spiceObject, scObject, cam):
    """
    This method projects a camera FOV onto the surface of a reference ellipsoid. All rays
    which intersect the ellipsoid are added to the output list, which is formatted with
    the intersection points as: [x1 y1 z1 x2 y2 ... z4]. If all four camera rays intersect,
    this list can be used to generate a QuadMap in Vizard to see the region. Otherwise, the
    partial list is returned for the user to handle.

    Parameters
    ----------
    parentBody:
        Parent grav body, containing ``radEquator`` and ``radiusRatio`` which define the
        reference ellipsoid
    spiceObject:
        SPICE object
    scObject:
        Spacecraft instance
    cam:
        vizInterface.StdCameraSettings or messaging.CameraConfigMsgPayload

    Returns
    -------
    Up to 12-element double-list
        FOVBox, list of camera FOV rays which intersect the reference ellipsoid

    """

    # Extract spacecraft position in inertial frame (N)
    r_BN_N = scObject.scStateOutMsg.read().r_BN_N

    # Find inertial (N) to planet-fixed (P) DCM
    PN = np.array(spiceObject.planetStateOutMsgs[0].read().J20002Pfix).reshape((3, 3))
    r_BN_P = np.matmul(PN, r_BN_N)

    # Find inertial (N) to spacecraft (B) DCM
    sigma_BN = scObject.scStateOutMsg.read().sigma_BN
    BN = rbk.MRP2C(np.array(sigma_BN))

    # Check camera type
    r_CN_P = []
    PC = []
    FOV = 0
    # -- Standard camera
    if isinstance(cam, vizInterface.StdCameraSettings):
        r_CB_B = cam.position_B
        r_CB_N = np.matmul(np.transpose(BN), r_CB_B)
        r_CB_P = np.matmul(PN, r_CB_N)
        r_CN_P = r_BN_P + r_CB_P

        # Get planet-fixed (P) to camera (C) DCM
        CP_1 = -r_CN_P / np.linalg.norm(r_CN_P)  # Camera pointing toward the planet
        CP_2 = -np.cross(CP_1, np.array([0, 0, 1])) / np.linalg.norm(np.cross(CP_1, np.array([0, 0, 1])))  # Define "up"
        CP_3 = np.cross(CP_1, CP_2)
        CP = np.array([CP_1, CP_2, CP_3])
        PC = np.transpose(CP)

        FOV = cam.fieldOfView
    # -- Camera config
    elif isinstance(cam, messaging.CameraConfigMsgPayload):
        r_CB_B = cam.cameraPos_B
        r_CB_N = np.matmul(np.transpose(BN), r_CB_B)
        r_CB_P = np.matmul(PN, r_CB_N)
        r_CN_P = r_BN_P + r_CB_P

        # Get planet-fixed (P) to camera (C) DCM
        CB = rbk.MRP2C(np.array(cam.sigma_CB))
        PB = np.matmul(PN, np.transpose(BN))
        PC = np.matmul(PB, np.transpose(CB))

        FOV = cam.fieldOfView

    # Compute FOV rays in camera frame (C)
    half_fov = np.tan(FOV / 2)
    d1_C = np.array([1, half_fov, half_fov])
    d2_C = np.array([1, -half_fov, half_fov])
    d3_C = np.array([1, -half_fov, -half_fov])
    d4_C = np.array([1, half_fov, -half_fov])

    # Rotate rays from camera frame (C) to planet-fixed frame (P)
    d1_P = np.matmul(PC, d1_C)
    d2_P = np.matmul(PC, d2_C)
    d3_P = np.matmul(PC, d3_C)
    d4_P = np.matmul(PC, d4_C)

    d_P = [d1_P, d2_P, d3_P, d4_P]

    # Extract planet semi-major and semi-minor axes
    a = parentBody.radEquator
    b = parentBody.radiusRatio * a

    # Find intersection of each ray with the planet
    FOVBox = []
    for i in range(0, 4):
        di_P = d_P[i]

        A = (di_P[0] / a)**2 + (di_P[1] / a)**2 + (di_P[2] / b)**2
        B = 2 * (r_CN_P[0] * di_P[0] / (a**2) + r_CN_P[1] * di_P[1] / (a**2) + r_CN_P[2] * di_P[2] / (b**2))
        C = (r_CN_P[0] / a)**2 + (r_CN_P[1] / a)**2 + (r_CN_P[2] / b)**2 - 1

        disc = B**2 - 4*A*C

        if disc < 0:
            # print(f"ERROR: No intersection for ray {i}")
            continue

        tPos = (-B + np.sqrt(disc)) / (2 * A)
        tNeg = (-B - np.sqrt(disc)) / (2 * A)

        # Compute parametrized solution
        potentialSols = [x for x in [tPos, tNeg] if x > 0]
        if potentialSols:
            tSol = min(potentialSols)  # Choose the smallest positive root
            di_P_int = r_CN_P + tSol * di_P
            FOVBox.extend(di_P_int)
        else:
            pass
            # print(f"ERROR: No positive root for ray {i}")

    return FOVBox


def subdivideFOVBox(parentBody, FOVBox, fieldOfViewSubdivs):
    """
    This method subdivides a complete FOVBox into N x N regions. This is helpful for
    wrapping a rectangular region over a convex surface. Uses bi-linear interpolation.

    Parameters
    ----------
    parentBody:
        Parent grav body, containing ``radEquator`` and ``radiusRatio`` which define the
        reference ellipsoid.
    FOVBox:
        FOV corners in fixed-frame, defined as [x1, y1, z1, x2, ... , z4]
    fieldOfViewSubdivs:
        N, number of vertical and horizontal subdivisions to mesh

    Returns
    -------
    N x N x 4  double-list
        vertices, interpolated FOVBox in fixed-frame to include higher-resolution quads

    """
    # Extract planet radius and flattening
    p_RE = parentBody.radEquator
    p_RR = parentBody.radiusRatio
    drawHt = 0.0001 * p_RE

    # Convert 3D points to LLA
    llaList = []
    for i in range(4):
        r_GP_P = FOVBox[i*3:(i+1)*3]
        lla = vizSupport.fixedframe2lla(r_GP_P, p_RE, p_RR)
        llaList.append(lla)  # lla = [lat, lon, alt]

    # Unpack LLA corners
    TL = llaList[0]  # top-left
    TR = llaList[1]  # top-right
    BR = llaList[2]  # bottom-right
    BL = llaList[3]  # bottom-left

    # Generate the mesh of N x N quadrilaterals
    vertices = []
    for i in range(fieldOfViewSubdivs):

        # Create left and right lat/lon strips
        topLeftLon = TL[1] + (TR[1] - TL[1]) * i / fieldOfViewSubdivs
        botLeftLon = BL[1] + (BR[1] - BL[1]) * i / fieldOfViewSubdivs
        leftLon = np.linspace(botLeftLon, topLeftLon, fieldOfViewSubdivs + 1)

        topRightLon = TL[1] + (TR[1] - TL[1]) * (i + 1) / fieldOfViewSubdivs
        botRightLon = BL[1] + (BR[1] - BL[1]) * (i + 1) / fieldOfViewSubdivs
        rightLon = np.linspace(botRightLon, topRightLon, fieldOfViewSubdivs + 1)

        topLeftLat = TL[0] + (TR[0] - TL[0]) * i / fieldOfViewSubdivs
        botLeftLat = BL[0] + (BR[0] - BL[0]) * i / fieldOfViewSubdivs
        leftLat = np.linspace(botLeftLat, topLeftLat, fieldOfViewSubdivs + 1)

        topRightLat = TL[0] + (TR[0] - TL[0]) * (i + 1) / fieldOfViewSubdivs
        botRightLat = BL[0] + (BR[0] - BL[0]) * (i + 1) / fieldOfViewSubdivs
        rightLat = np.linspace(botRightLat, topRightLat, fieldOfViewSubdivs + 1)

        for j in range(fieldOfViewSubdivs):
            vertices.extend(vizSupport.lla2fixedframe([leftLat[j], leftLon[j], drawHt], p_RE, p_RR))
            vertices.extend(vizSupport.lla2fixedframe([rightLat[j], rightLon[j], drawHt], p_RE, p_RR))
            vertices.extend(vizSupport.lla2fixedframe([rightLat[j+1], rightLon[j+1], drawHt], p_RE, p_RR))
            vertices.extend(vizSupport.lla2fixedframe([leftLat[j+1], leftLon[j+1], drawHt], p_RE, p_RR))

    return vertices
