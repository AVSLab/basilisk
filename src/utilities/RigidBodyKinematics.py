
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.




import math

# Import required modules:
import numpy as np
from numpy import linalg as la

M_PI = np.pi
D2R = M_PI / 180.
R2D = 180. / M_PI


def Picheck(x):
    """
        Picheck(x)
        Makes sure that the angle x lies within +/- Pi.
    """
    if x > M_PI:
        return x - 2 * M_PI
    if x < -M_PI:
        return x + 2 * M_PI
    return x


def C2EP(C):
    """
    C2EP
        Q = C2EP(C) translates the 3x3 direction cosine matrix
        C into the corresponding 4x1 euler parameter vector Q,
        where the first component of Q is the non-dimensional
        Euler parameter Beta_0 >= 0. Transformation is done
        using the Stanley method.
    """
    tr = np.trace(C)
    b2 = np.array([(1 + tr) / 4,
                   (1 + 2 * C[0, 0] - tr) / 4,
                   (1 + 2 * C[1, 1] - tr) / 4,
                   (1 + 2 * C[2, 2] - tr) / 4
                   ])
    case = np.argmax(b2)
    b = b2
    if case == 0:
        b[0] = np.sqrt(b2[0])
        b[1] = (C[1, 2] - C[2, 1]) / 4 / b[0]
        b[2] = (C[2, 0] - C[0, 2]) / 4 / b[0]
        b[3] = (C[0, 1] - C[1, 0]) / 4 / b[0]
    elif case == 1:
        b[1] = np.sqrt(b2[1])
        b[0] = (C[1, 2] - C[2, 1]) / 4 / b[1]
        if b[0] < 0:
            b[1] = -b[1]
            b[0] = -b[0]
        b[2] = (C[0, 1] + C[1, 0]) / 4 / b[1]
        b[3] = (C[2, 0] + C[0, 2]) / 4 / b[1]
    elif case == 2:
        b[2] = np.sqrt(b2[2])
        b[0] = (C[2, 0] - C[0, 2]) / 4 / b[2]
        if b[0] < 0:
            b[2] = -b[2]
            b[0] = -b[0]
        b[1] = (C[0, 1] + C[1, 0]) / 4 / b[2]
        b[3] = (C[1, 2] + C[2, 1]) / 4 / b[2]
    elif case == 3:
        b[3] = np.sqrt(b2[3])
        b[0] = (C[0, 1] - C[1, 0]) / 4 / b[3]
        if b[0] < 0:
            b[3] = -b[3]
            b[0] = -b[0]
        b[1] = (C[2, 0] + C[0, 2]) / 4 / b[3]
        b[2] = (C[1, 2] + C[2, 1]) / 4 / b[3]
    return b


def C2Euler121(C):
    """
    C2Euler121

    	Q = C2Euler121(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (1-2-1) euler angle set.
    """
    q0 = math.atan2(C[0, 1], -C[0, 2])
    q1 = math.acos(C[0, 0])
    q2 = math.atan2(C[1, 0], C[2, 0])
    q = np.array([q0, q1, q2])

    return q


def C2Euler123(C):
    """
    C2Euler123

    	Q = C2Euler123(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (1-2-3) euler angle set.
    """
    q0 = np.arctan2(-C[2, 1], C[2, 2])
    q1 = np.arcsin(C[2, 0])
    q2 = np.arctan2(-C[1, 0], C[0, 0])
    q = np.array([q0, q1, q2])
    return q


def C2Euler131(C):
    """
    C2Euler131

    	Q = C2Euler131(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (1-3-1) euler angle set.
    """
    q0 = math.atan2(C[0, 2], C[0, 1])
    q1 = math.acos(C[0, 0])
    q2 = math.atan2(C[2, 0], -C[1, 0])
    q = np.array([q0, q1, q2])

    return q


def C2Euler132(C):
    """
    C2Euler132

    	Q = C2Euler132(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (1-3-2) euler angle set.
    """
    q0 = math.atan2(C[1, 2], C[1, 1])
    q1 = math.asin(-C[1, 0])
    q2 = math.atan2(C[2, 0], C[0, 0])
    q = np.array([q0, q1, q2])

    return q


def C2Euler212(C):
    """
    C2Euler212

    	Q = C2Euler212(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (2-1-2) euler angle set.
    """
    q0 = math.atan2(C[1, 0], C[1, 2])
    q1 = math.acos(C[1, 1])
    q2 = math.atan2(C[0, 1], -C[2, 1])
    q = np.array([q0, q1, q2])

    return q


def C2Euler213(C):
    """
    C2Euler213

        Q = C2Euler213(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (2-1-3) euler angle set.
    """

    q0 = math.atan2(C[2, 0], C[2, 2])
    q1 = math.asin(-C[2, 1])
    q2 = math.atan2(C[0, 1], C[1, 1])
    q = np.array([q0, q1, q2])

    return q


def C2Euler231(C):
    """
    C2Euler231

    	Q = C2Euler231(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (2-3-1) euler angle set.
    """

    q0 = math.atan2(-C[0, 2], C[0, 0])
    q1 = math.asin(C[0, 1])
    q2 = math.atan2(-C[2, 1], C[1, 1])
    q = np.array([q0, q1, q2])
    return q


def C2Euler232(C):
    """
    C2Euler232

    	Q = C2Euler232(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (2-3-2) euler angle set.
    """

    q0 = math.atan2(C[1, 2], -C[1, 0])
    q1 = math.acos(C[1, 1])
    q2 = math.atan2(C[2, 1], C[0, 1])
    q = np.array([q0, q1, q2])
    return q


def C2Euler312(C):
    """
    C2Euler312

    	Q = C2Euler312(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (3-1-2) euler angle set.
    """

    q0 = math.atan2(-C[1, 0], C[1, 1])
    q1 = math.asin(C[1, 2])
    q2 = math.atan2(-C[0, 2], C[2, 2])
    q = np.array([q0, q1, q2])
    return q


def C2Euler313(C):
    """
    C2Euler313

    	Q = C2Euler313(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (3-1-3) euler angle set.
    """

    q0 = math.atan2(C[2, 0], -C[2, 1])
    q1 = math.acos(C[2, 2])
    q2 = math.atan2(C[0, 2], C[1, 2])
    q = np.array([q0, q1, q2])
    return q


def C2Euler321(C):
    """
    C2Euler321

    	Q = C2Euler321(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (3-2-1) euler angle set.
    """

    q0 = math.atan2(C[0, 1], C[0, 0])
    q1 = math.asin(-C[0, 2])
    q2 = math.atan2(C[1, 2], C[2, 2])
    q = np.array([q0, q1, q2])
    return q


def C2Euler323(C):
    """
    C2Euler323

    	Q = C2Euler323(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (3-2-3) euler angle set.
    """

    q0 = math.atan2(C[2, 1], C[2, 0])
    q1 = math.acos(C[2, 2])
    q2 = math.atan2(C[1, 2], -C[0, 2])
    q = np.array([q0, q1, q2])
    return q


def C2Gibbs(C):
    """
    C2Gibbs

    	Q = C2Gibbs(C) translates the 3x3 direction cosine matrix
    	C into the corresponding 3x1 gibbs vector Q.
    """

    b = C2EP(C)

    q0 = b[1] / b[0]
    q1 = b[2] / b[0]
    q2 = b[3] / b[0]
    q = np.array([q0, q1, q2])
    return q


def C2MRP(C):
    """
    C2MRP

    	Q = C2MRP(C) translates the 3x3 direction cosine matrix
    	C into the corresponding 3x1 MRP vector Q where the
    	MRP vector is chosen such that :math:`|Q| <= 1`.
    """

    b = C2EP(C)
    q = np.array([
        b[1] / (1 + b[0]),
        b[2] / (1 + b[0]),
        b[3] / (1 + b[0])
    ])
    return q


def C2PRV(C):
    """
    C2PRV

    	Q = C2PRV(C) translates the 3x3 direction cosine matrix
    	C into the corresponding 3x1 principal rotation vector Q,
    	where the first component of Q is the principal rotation angle
    	phi (0<= phi <= Pi)
    """

    cp = (np.trace(C) - 1) / 2
    p = np.arccos(cp)
    sp = p / 2. / np.sin(p)
    q = np.array([
        (C[1, 2] - C[2, 1]) * sp,
        (C[2, 0] - C[0, 2]) * sp,
        (C[0, 1] - C[1, 0]) * sp
    ])
    return q


def addEP(b1, b2):
    """
    addEP(B1,B2)

    	Q = addEP(B1,B2) provides the euler parameter vector
    	which corresponds to performing to successive
    	rotations B1 and B2.
    """

    q0 = b2[0] * b1[0] - b2[1] * b1[1] - b2[2] * b1[2] - b2[3] * b1[3]
    q1 = b2[1] * b1[0] + b2[0] * b1[1] + b2[3] * b1[2] - b2[2] * b1[3]
    q2 = b2[2] * b1[0] - b2[3] * b1[1] + b2[0] * b1[2] + b2[1] * b1[3]
    q3 = b2[3] * b1[0] + b2[2] * b1[1] - b2[1] * b1[2] + b2[0] * b1[3]
    q = np.array([q0, q1, q2, q3])

    return q


def addEuler121(e1, e2):
    """
    addEuler121(E1,E2)

    	Q = addEuler121(E1,E2) computes the overall (1-2-1) euler
    	angle vector corresponding to two successive
    	(1-2-1) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1])
    cp2 = math.cos(e2[1])
    sp1 = math.sin(e1[1])
    sp2 = math.sin(e2[1])
    dum = e1[2] + e2[0]

    q1 = math.acos(cp1 * cp2 - sp1 * sp2 * math.cos(dum))
    cp3 = math.cos(q1)
    q0 = Picheck(e1[0] + math.atan2(sp1 * sp2 * math.sin(dum), cp2 - cp3 * cp1))
    q2 = Picheck(e2[2] + math.atan2(sp1 * sp2 * math.sin(dum), cp1 - cp3 * cp2))

    q = np.array([q0, q1, q2])

    return q


def addEuler123(e1, e2):
    """
    addEuler123(E1,E2)

    	Q = addEuler123(E1,E2) computes the overall (1-2-3) euler
    	angle vector corresponding to two successive
    	(1-2-3) rotations E1 and E2.
    """

    C1 = euler1232C(e1)
    C2 = euler1232C(e2)
    C = np.dot(C2, C1)
    return C2Euler123(C)


def addEuler131(e1, e2):
    """
    addEuler131(E1,E2)

    	Q = addEuler131(E1,E2) computes the overall (1-3-1) euler
    	angle vector corresponding to two successive
    	(1-3-1) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1])
    cp2 = math.cos(e2[1])
    sp1 = math.sin(e1[1])
    sp2 = math.sin(e2[1])
    dum = e1[2] + e2[0]

    q1 = math.acos(cp1 * cp2 - sp1 * sp2 * math.cos(dum))
    cp3 = math.cos(q1)
    q0 = Picheck(e1[0] + math.atan2(sp1 * sp2 * math.sin(dum), cp2 - cp3 * cp1))
    q2 = Picheck(e2[2] + math.atan2(sp1 * sp2 * math.sin(dum), cp1 - cp3 * cp2))

    q = np.array([q0, q1, q2])
    return q


def addEuler132(e1, e2):
    """
    addEuler132(E1,E2)

    	Q = addEuler132(E1,E2) computes the overall (1-3-2) euler
    	angle vector corresponding to two successive
    	(1-3-2) rotations E1 and E2.
    """

    C1 = euler1322C(e1)
    C2 = euler1322C(e2)
    C = np.dot(C2, C1)
    return C2Euler132(C)


def addEuler212(e1, e2):
    """
    addEuler212(E1,E2)

    	Q = addEuler212(E1,E2) computes the overall (2-1-2) euler
    	angle vector corresponding to two successive
    	(2-1-2) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1])
    cp2 = math.cos(e2[1])
    sp1 = math.sin(e1[1])
    sp2 = math.sin(e2[1])
    dum = e1[2] + e2[0]

    q1 = math.acos(cp1 * cp2 - sp1 * sp2 * math.cos(dum))
    cp3 = math.cos(q1)
    q0 = Picheck(e1[0] + math.atan2(sp1 * sp2 * math.sin(dum), cp2 - cp3 * cp1))
    q2 = Picheck(e2[2] + math.atan2(sp1 * sp2 * math.sin(dum), cp1 - cp3 * cp2))
    q = np.array([q0, q1, q2])
    return q


def addEuler213(e1, e2):
    """
    addEuler213(E1,E2)

    	Q = addEuler213(E1,E2) computes the overall (2-1-3) euler
    	angle vector corresponding to two successive
    	(2-1-3) rotations E1 and E2.
    """

    C1 = euler2132C(e1)
    C2 = euler2132C(e2)
    C = np.dot(C2, C1)
    return C2Euler213(C)


def addEuler231(e1, e2):
    """
    addEuler231(E1,E2)

    	Q = addEuler231(E1,E2) computes the overall (2-3-1) euler
    	angle vector corresponding to two successive
    	(2-3-1) rotations E1 and E2.
    """

    C1 = euler2312C(e1)
    C2 = euler2312C(e2)
    C = np.dot(C2, C1)
    return C2Euler231(C)


def addEuler232(e1, e2):
    """
    addEuler232(E1,E2)

    	Q = addEuler232(E1,E2) computes the overall (2-3-2) euler
    	angle vector corresponding to two successive
    	(2-3-2) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1])
    cp2 = math.cos(e2[1])
    sp1 = math.sin(e1[1])
    sp2 = math.sin(e2[1])
    dum = e1[2] + e2[0]

    q1 = math.acos(cp1 * cp2 - sp1 * sp2 * math.cos(dum))
    cp3 = math.cos(q1)
    q0 = Picheck(e1[0] + math.atan2(sp1 * sp2 * math.sin(dum), cp2 - cp3 * cp1))
    q2 = Picheck(e2[2] + math.atan2(sp1 * sp2 * math.sin(dum), cp1 - cp3 * cp2))
    q = np.array([q0, q1, q2])
    return q


def addEuler312(e1, e2):
    """
    addEuler312(E1,E2)

    	Q = addEuler312(E1,E2) computes the overall (3-1-2) euler
    	angle vector corresponding to two successive
    	(3-1-2) rotations E1 and E2.
    """

    C1 = euler3122C(e1)
    C2 = euler3122C(e2)
    C = np.dot(C2, C1)
    return C2Euler312(C)


def addEuler313(e1, e2):
    """
    addEuler313(E1,E2)

    	Q = addEuler313(E1,E2) computes the overall (3-1-3) euler
    	angle vector corresponding to two successive
    	(3-1-3) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1])
    cp2 = math.cos(e2[1])
    sp1 = math.sin(e1[1])
    sp2 = math.sin(e2[1])
    dum = e1[2] + e2[0]

    q1 = math.acos(cp1 * cp2 - sp1 * sp2 * math.cos(dum))
    cp3 = math.cos(q1)
    q0 = Picheck(e1[0] + math.atan2(sp1 * sp2 * math.sin(dum), cp2 - cp3 * cp1))
    q2 = Picheck(e2[2] + math.atan2(sp1 * sp2 * math.sin(dum), cp1 - cp3 * cp2))
    q = np.array([q0, q1, q2])
    return q


def addEuler321(e1, e2):
    """
    addEuler321(E1,E2)

    	Q = addEuler321(E1,E2) computes the overall (3-2-1) euler
    	angle vector corresponding to two successive
    	(3-2-1) rotations E1 and E2.
    """

    C1 = euler3212C(e1)
    C2 = euler3212C(e2)
    C = np.dot(C2, C1)
    return C2Euler321(C)


def addEuler323(e1, e2):
    """
    addEuler323(E1,E2)

    	Q = addEuler323(E1,E2) computes the overall (3-2-3) euler
    	angle vector corresponding to two successive
    	(3-2-3) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1])
    cp2 = math.cos(e2[1])
    sp1 = math.sin(e1[1])
    sp2 = math.sin(e2[1])
    dum = e1[2] + e2[0]

    q1 = math.acos(cp1 * cp2 - sp1 * sp2 * math.cos(dum))
    cp3 = math.cos(q1)
    q0 = Picheck(e1[0] + math.atan2(sp1 * sp2 * math.sin(dum), cp2 - cp3 * cp1))
    q2 = Picheck(e2[2] + math.atan2(sp1 * sp2 * math.sin(dum), cp1 - cp3 * cp2))
    q = np.array([q0, q1, q2])
    return q


def addGibbs(q1, q2):
    """
    addGibbs(Q1,Q2)

    	Q = addGibbs(Q1,Q2) provides the gibbs vector
    	which corresponds to performing to successive
    	rotations Q1 and Q2.
    """
    result = (q1 + q2 + np.cross(q1, q2)) / (1 - np.dot(q1, q2))
    return result


def addMRP(q1, q2):
    """
     addMRP(Q1,Q2)

    	Q = addMRP(Q1,Q2) provides the MRP vector
    	which corresponds to performing to successive
    	rotations Q1 and Q2.
    """

    den = 1 + np.dot(q1, q1) * np.dot(q2, q2) - 2 * np.dot(q1, q2)

    if np.abs(den) < 1e-5:
        q2 = -q2/np.dot(q2,q2)
        den = 1 + np.dot(q1, q1) * np.dot(q2, q2) - 2 * np.dot(q1, q2)
    num = (1 - np.dot(q1, q1)) * q2 + (1 - np.dot(q2, q2)) * q1 + 2 * np.cross(q1, q2)

    q = num / den

    if np.dot(q,q) > 1:
            q = - q/np.dot(q,q)

    return q


def PRV2elem(r):
    """
    PRV2elem(R)

    	Q = PRV2elem(R) translates a prinicpal rotation vector R
    	into the corresponding principal rotation element set Q.
    """
    q0 = np.linalg.norm(r)
    if q0 < 1e-12:
        return np.zeros(4)
    q1 = r[0] / q0
    q2 = r[1] / q0
    q3 = r[2] / q0
    q = np.array([q0, q1, q2, q3])
    return q


def addPRV(qq1, qq2):
    """
     addPRV(Q1,Q2)

    	Q = addPRV(Q1,Q2) provides the principal rotation vector
    	which corresponds to performing to successive
    	prinicipal rotations Q1 and Q2.
    """

    q1 = PRV2elem(qq1)
    q2 = PRV2elem(qq2)
    cp1 = math.cos(q1[0] / 2.)
    cp2 = math.cos(q2[0] / 2.)
    sp1 = math.sin(q1[0] / 2.)
    sp2 = math.sin(q2[0] / 2.)
    e1 = q1[1:4]
    e2 = q2[1:4]

    p = 2. * math.acos(cp1 * cp2 - sp1 * sp2 * np.dot(e1, e2))
    sp = math.sin(p / 2.)
    e = (cp1 * sp2 * e2 + cp2 * sp1 * e1 + sp1 * sp2 * np.cross(e1, e2))
    q = (p / sp) * e

    return q


def BinvEP(q):
    """
    BinvEP(Q)

    	B = BinvEP(Q) returns the 3x4 matrix which relates
    	the derivative of euler parameter vector Q to the
    	body angular velocity vector w.

    		w = 2 [B(Q)]^(-1) dQ/dt
    """
    B = np.zeros([3, 4])
    B[0, 0] = -q[1]
    B[0, 1] = q[0]
    B[0, 2] = q[3]
    B[0, 3] = -q[2]
    B[1] = -q[2]
    B[1, 1] = -q[3]
    B[1, 2] = q[0]
    B[1, 3] = q[1]
    B[2] = -q[3]
    B[2, 1] = q[2]
    B[2, 2] = -q[1]
    B[2, 3] = q[0]

    return B


def BinvEuler121(q):
    """
    BinvEuler121(Q)

    	B = BinvEuler121(Q) returns the 3x3 matrix which relates
    	the derivative of the (1-2-1) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = c2
    B[0, 1] = 0
    B[0, 2] = 1
    B[1, 0] = s2 * s3
    B[1, 1] = c3
    B[1, 2] = 0
    B[2, 0] = s2 * c3
    B[2, 1] = -s3
    B[2, 2] = 0

    return B


def BinvEuler123(q):
    """
    BinvEuler123(Q)

    	B = BinvEuler123(Q) returns the 3x3 matrix which relates
    	the derivative of the (1-2-3) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = c2 * c3
    B[0, 1] = s3
    B[0, 2] = 0
    B[1, 0] = -c2 * s3
    B[1, 1] = c3
    B[1, 2] = 0
    B[2, 0] = s2
    B[2, 1] = 0
    B[2, 2] = 1

    return B


def BinvEuler131(q):
    """
    BinvEuler131(Q)

    	B = BinvEuler131(Q) returns the 3x3 matrix which relates
    	the derivative of the (1-3-1) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = c2
    B[0, 1] = 0
    B[0, 2] = 1
    B[1, 0] = -s2 * c3
    B[1, 1] = s3
    B[1, 2] = 0
    B[2, 0] = s2 * s3
    B[2, 1] = c3
    B[2, 2] = 0

    return B


def BinvEuler132(q):
    """
    BinvEuler132(Q)

    	B = BinvEuler132(Q) returns the 3x3 matrix which relates
    	the derivative of the (1-3-2) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = c2 * c3
    B[0, 1] = -s3
    B[0, 2] = 0
    B[1, 0] = -s2
    B[1, 1] = 0
    B[1, 2] = 1
    B[2, 0] = c2 * s3
    B[2, 1] = c3
    B[2, 2] = 0

    return B


def BinvEuler212(q):
    """
    BinvEuler212(Q)

    	B = BinvEuler212(Q) returns the 3x3 matrix which relates
    	the derivative of the (2-1-2) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = s2 * s3
    B[0, 1] = c3
    B[0, 2] = 0
    B[1, 0] = c2
    B[1, 1] = 0
    B[1, 2] = 1
    B[2, 0] = -s2 * c3
    B[2, 1] = s3
    B[2, 2] = 0

    return B


def BinvEuler213(q):
    """
    BinvEuler213(Q)

    	B = BinvEuler213(Q) returns the 3x3 matrix which relates
    	the derivative of the (2-1-3) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = c2 * s3
    B[0, 1] = c3
    B[0, 2] = 0
    B[1, 0] = c2 * c3
    B[1, 1] = -s3
    B[1, 2] = 0
    B[2, 0] = -s2
    B[2, 1] = 0
    B[2, 2] = 1

    return B


def BinvEuler231(q):
    """
    BinvEuler231(Q)

    	B = BinvEuler231(Q) returns the 3x3 matrix which relates
    	the derivative of the (2-3-1) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = s2
    B[0, 1] = 0
    B[0, 2] = 1
    B[1, 0] = c2 * c3
    B[1, 1] = s3
    B[1, 2] = 0
    B[2, 0] = -c2 * s3
    B[2, 1] = c3
    B[2, 2] = 0

    return B


def BinvEuler232(q):
    """
    BinvEuler232(Q)

    	B = BinvEuler232(Q) returns the 3x3 matrix which relates
    	the derivative of the (2-3-2) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = s2 * c3
    B[0, 1] = -s3
    B[0, 2] = 0
    B[1, 0] = c2
    B[1, 1] = 0
    B[1, 2] = 1
    B[2, 0] = s2 * s3
    B[2, 1] = c3
    B[2, 2] = 0

    return B


def BinvEuler312(q):
    """
    BinvEuler312(Q)

    	B = BinvEuler312(Q) returns the 3x3 matrix which relates
    	the derivative of the (3-1-2) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = -c2 * s3
    B[0, 1] = c3
    B[0, 2] = 0
    B[1, 0] = s2
    B[1, 1] = 0
    B[1, 2] = 1
    B[2, 0] = c2 * c3
    B[2, 1] = s3
    B[2, 2] = 0

    return B


def BinvEuler313(q):
    """
    BinvEuler313(Q)

    	B = BinvEuler313(Q) returns the 3x3 matrix which relates
    	the derivative of the (3-1-3) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = s2 * s3
    B[0, 1] = c3
    B[0, 2] = 0
    B[1, 0] = s2 * c3
    B[1, 1] = -s3
    B[1, 2] = 0
    B[2, 0] = c2
    B[2, 1] = 0
    B[2, 2] = 1

    return B


def BinvEuler321(q):
    """
    BinvEuler321(Q)

    	B = BinvEuler321(Q) returns the 3x3 matrix which relates
    	the derivative of the (3-2-1) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = -s2
    B[0, 1] = 0
    B[0, 2] = 1
    B[1, 0] = c2 * s3
    B[1, 1] = c3
    B[1, 2] = 0
    B[2, 0] = c2 * c3
    B[2, 1] = -s3
    B[2, 2] = 0

    return B


def BinvEuler323(q):
    """
    BinvEuler323(Q)

    	B = BinvEuler323(Q) returns the 3x3 matrix which relates
    	the derivative of the (3-2-3) euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])

    B = np.zeros([3, 3])
    B[0, 0] = -s2 * c3
    B[0, 1] = s3
    B[0, 2] = 0
    B[1, 0] = s2 * s3
    B[1, 1] = c3
    B[1, 2] = 0
    B[2, 0] = c2
    B[2, 1] = 0
    B[2, 2] = 1

    return B


def BinvGibbs(q):
    """
    BinvGibbs(Q)

    	B = BinvGibbs(Q) returns the 3x3 matrix which relates
    	the derivative of gibbs vector Q to the
    	body angular velocity vector w.

    		w = 2 [B(Q)]^(-1) dQ/dt
    """

    B = np.zeros([3, 3])
    B[0, 0] = 1
    B[0, 1] = q[2]
    B[0, 2] = -q[1]
    B[1, 0] = -q[2]
    B[1, 1] = 1
    B[1, 2] = q[0]
    B[2, 0] = q[1]
    B[2, 1] = -q[0]
    B[2, 2] = 1
    B = B / (1 + np.dot(q, q))

    return B


def BinvMRP(q):
    """
    BinvMRP(Q)

    	B = BinvMRP(Q) returns the 3x3 matrix which relates
    	the derivative of MRP vector Q to the
    	body angular velocity vector w.

    		w = 4 [B(Q)]^(-1) dQ/dt
    """

    s2 = np.dot(q, q)
    B = np.zeros([3, 3])
    B[0, 0] = 1 - s2 + 2 * q[0] * q[0]
    B[0, 1] = 2 * (q[0] * q[1] + q[2])
    B[0, 2] = 2 * (q[0] * q[2] - q[1])
    B[1, 0] = 2 * (q[1] * q[0] - q[2])
    B[1, 1] = 1 - s2 + 2 * q[1] * q[1]
    B[1, 2] = 2 * (q[1] * q[2] + q[0])
    B[2, 0] = 2 * (q[2] * q[0] + q[1])
    B[2, 1] = 2 * (q[2] * q[1] - q[0])
    B[2, 2] = 1 - s2 + 2 * q[2] * q[2]
    B = B / (1 + s2) / (1 + s2)

    return B


def BinvPRV(q):
    """
    BinvPRV(Q)

    	B = BinvPRV(Q) returns the 3x3 matrix which relates
    	the derivative of principal rotation vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    p = la.norm(q)
    c1 = (1 - math.cos(p)) / p / p
    c2 = (p - math.sin(p)) / p / p / p

    B = np.zeros([3, 3])
    B[0, 0] = 1 - c2 * (q[1] * q[1] + q[2] * q[2])
    B[0, 1] = c1 * q[2] + c2 * q[0] * q[1]
    B[0, 2] = -c1 * q[1] + c2 * q[0] * q[2]
    B[1, 0] = -c1 * q[2] + c2 * q[0] * q[1]
    B[1, 1] = 1 - c2 * (q[0] * q[0] + q[2] * q[2])
    B[1, 2] = c1 * q[0] + c2 * q[1] * q[2]
    B[2, 0] = c1 * q[1] + c2 * q[2] * q[0]
    B[2, 1] = -c1 * q[0] + c2 * q[2] * q[1]
    B[2, 2] = 1 - c2 * (q[0] * q[0] + q[1] * q[1])

    return B


def BmatEP(q):
    """
    BmatEP(Q)

    	B = BmatEP(Q) returns the 4x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	Euler parameter vector Q.

    		dQ/dt = 1/2 [B(Q)] w
    """

    B = np.zeros([4, 3])
    B[0, 0] = -q[1]
    B[0, 1] = -q[2]
    B[0, 2] = -q[3]
    B[1, 0] = q[0]
    B[1, 1] = -q[3]
    B[1, 2] = q[2]
    B[2, 0] = q[3]
    B[2, 1] = q[0]
    B[2, 2] = -q[1]
    B[3, 0] = -q[2]
    B[3, 1] = q[1]
    B[3, 2] = q[0]

    return B


def BmatEuler121(q):
    """
    BmatEuler121(Q)

    	B = BmatEuler121(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(1-2-1) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = 0
    B[0, 1] = s3
    B[0, 2] = c3
    B[1, 0] = 0
    B[1, 1] = s2 * c3
    B[1, 2] = -s2 * s3
    B[2, 0] = s2
    B[2, 1] = -c2 * s3
    B[2, 2] = -c2 * c3
    B = B / s2

    return B


def BmatEuler123(q):
    """
    BmatEuler123(Q)

    	B = BmatEuler123(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(1-2-3) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = c3
    B[0, 1] = -s3
    B[0, 2] = 0
    B[1, 0] = c2 * s3
    B[1, 1] = c2 * c3
    B[1, 2] = 0
    B[2, 0] = -s2 * c3
    B[2, 1] = s2 * s3
    B[2, 2] = c2
    B = B / c2

    return B


def BmatEuler131(q):
    """
    BmatEuler131(Q)

    	B = BmatEuler131(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(1-3-1) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = 0
    B[0, 1] = -c3
    B[0, 2] = s3
    B[1, 0] = 0
    B[1, 1] = s2 * s3
    B[1, 2] = s2 * c3
    B[2, 0] = s2
    B[2, 1] = c2 * c3
    B[2, 2] = -c2 * s3
    B = B / s2

    return B


def BmatEuler132(q):
    """
    BmatEuler132(Q)

    	B = BmatEuler132(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(1-3-2) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = c3
    B[0, 1] = 0
    B[0, 2] = s3
    B[1, 0] = -c2 * s3
    B[1, 1] = 0
    B[1, 2] = c2 * c3
    B[2, 0] = s2 * c3
    B[2, 1] = c2
    B[2, 2] = s2 * s3
    B = B / c2

    return B


def BmatEuler212(q):
    """
    BmatEuler212(Q)

    	B = BmatEuler212(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(2-1-2) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = s3
    B[0, 1] = 0
    B[0, 2] = -c3
    B[1, 0] = s2 * c3
    B[1, 1] = 0
    B[1, 2] = s2 * s3
    B[2, 0] = -c2 * s3
    B[2, 1] = s2
    B[2, 2] = c2 * c3
    B = B / s2

    return B


def BmatEuler213(q):
    """
    BmatEuler213(Q)

    	B = BmatEuler213(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(2-1-3) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = s3
    B[0, 1] = c3
    B[0, 2] = 0
    B[1, 0] = c2 * c3
    B[1, 1] = -c2 * s3
    B[1, 2] = 0
    B[2, 0] = s2 * s3
    B[2, 1] = s2 * c3
    B[2, 2] = c2
    B = B / c2

    return B


def BmatEuler231(q):
    """
    BmatEuler231(Q)

    	B = BmatEuler231(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(2-3-1) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = 0
    B[0, 1] = c3
    B[0, 2] = -s3
    B[1, 0] = 0
    B[1, 1] = c2 * s3
    B[1, 2] = c2 * c3
    B[2, 0] = c2
    B[2, 1] = -s2 * c3
    B[2, 2] = s2 * s3
    B = B / c2

    return B


def BmatEuler232(q):
    """
    BmatEuler232(Q)

    	B = BmatEuler232(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(2-3-2) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = c3
    B[0, 1] = 0
    B[0, 2] = s3
    B[1, 0] = -s2 * s3
    B[1, 1] = 0
    B[1, 2] = s2 * c3
    B[2, 0] = -c2 * c3
    B[2, 1] = s2
    B[2, 2] = -c2 * s3
    B = B / s2

    return B


def BmatEuler312(q):
    """
    BmatEuler312(Q)

    	B = BmatEuler312(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(3-1-2) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = -s3
    B[0, 1] = 0
    B[0, 2] = c3
    B[1, 0] = c2 * c3
    B[1, 1] = 0
    B[1, 2] = c2 * s3
    B[2, 0] = s2 * s3
    B[2, 1] = c2
    B[2, 2] = -s2 * c3
    B = B / c2

    return B


def BmatEuler313(q):
    """
    BmatEuler313(Q)

    	B = BmatEuler313(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(3-1-3) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = s3
    B[0, 1] = c3
    B[0, 2] = 0
    B[1, 0] = c3 * s2
    B[1, 1] = -s3 * s2
    B[1, 2] = 0
    B[2, 0] = -s3 * c2
    B[2, 1] = -c3 * c2
    B[2, 2] = s2
    B = B / s2

    return B


def BmatEuler321(q):
    """
    BmatEuler321(Q)

    	B = BmatEuler321(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(3-2-1) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = 0
    B[0, 1] = s3
    B[0, 2] = c3
    B[1, 0] = 0
    B[1, 1] = c2 * c3
    B[1, 2] = -c2 * s3
    B[2, 0] = c2
    B[2, 1] = s2 * s3
    B[2, 2] = s2 * c3
    B = B / c2

    return B


def BmatEuler323(q):
    """
    BmatEuler323(Q)

    	B = BmatEuler323(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(3-2-3) euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    B = np.zeros([3, 3])

    B[0, 0] = -c3
    B[0, 1] = s3
    B[0, 2] = 0
    B[1, 0] = s2 * s3
    B[1, 1] = s2 * c3
    B[1, 2] = 0
    B[2, 0] = c2 * c3
    B[2, 1] = -c2 * s3
    B[2, 2] = s2
    B = B / s2

    return B


def BmatGibbs(q):
    """
    BmatGibbs(Q)

    	B = BmatGibbs(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	Gibbs vector Q.

    		dQ/dt = 1/2 [B(Q)] w
    """

    B = np.zeros([3, 3])
    B[0, 0] = 1 + q[0] * q[0]
    B[0, 1] = q[0] * q[1] - q[2]
    B[0, 2] = q[0] * q[2] + q[1]
    B[1, 0] = q[1] * q[0] + q[2]
    B[1, 1] = 1 + q[1] * q[1]
    B[1, 2] = q[1] * q[2] - q[0]
    B[2, 0] = q[2] * q[0] - q[1]
    B[2, 1] = q[2] * q[1] + q[0]
    B[2, 2] = 1 + q[2] * q[2]

    return B


def BmatMRP(q):
    """
    BmatMRP(Q)

    	B = BmatMRP(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	MRP vector Q.

    		dQ/dt = 1/4 [B(Q)] w
    """

    B = np.zeros([3, 3])
    s2 = np.dot(q, q)
    B[0, 0] = 1 - s2 + 2 * q[0] * q[0]
    B[0, 1] = 2 * (q[0] * q[1] - q[2])
    B[0, 2] = 2 * (q[0] * q[2] + q[1])
    B[1, 0] = 2 * (q[1] * q[0] + q[2])
    B[1, 1] = 1 - s2 + 2 * q[1] * q[1]
    B[1, 2] = 2 * (q[1] * q[2] - q[0])
    B[2, 0] = 2 * (q[2] * q[0] - q[1])
    B[2, 1] = 2 * (q[2] * q[1] + q[0])
    B[2, 2] = 1 - s2 + 2 * q[2] * q[2]

    return B


def BdotmatMRP(q, dq):
    """
    BdotmatMRP(Q, dQ)

    	B = BdotmatMRP(Q, dQ) returns the derivative of the 3x3 BmatMRP
        matrix, which is used to calculate the second order derivative
        of the MRP vector Q.

    	(d^2Q)/(dt^2) = 1/4 ( [B(Q)] dw + [Bdot(Q,dQ)] w )
    """

    Bdot = np.zeros([3, 3])
    s = -2 * np.dot(q, dq)
    Bdot[0, 0] = s + 4 * (q[0] * dq[0])
    Bdot[0, 1] = 2 * (-dq[2] + q[0] * dq[1] + dq[0] * q[1])
    Bdot[0, 2] = 2 * ( dq[1] + q[0] * dq[2] + dq[0] * q[2])
    Bdot[1, 0] = 2 * ( dq[2] + q[0] * dq[1] + dq[0] * q[1])
    Bdot[1, 1] = s + 4 * (q[1] * dq[1])
    Bdot[1, 2] = 2 * (-dq[0] + q[1] * dq[2] + dq[1] * q[2])
    Bdot[2, 0] = 2 * (-dq[1] + q[0] * dq[2] + dq[0] * q[2])
    Bdot[2, 1] = 2 * ( dq[0] + q[1] * dq[2] + dq[1] * q[2])
    Bdot[2, 2] = s + 4 * (q[2] * dq[2])

    return Bdot


def BmatPRV(q):
    """
    BmatPRV(Q)

    	B = BmatPRV(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	principal rotation vector Q.

    		dQ/dt = [B(Q)] w
    """

    p = np.linalg.norm(q)
    c = 1 / p / p * (1 - p / 2 / math.tan(p / 2))
    B = np.zeros([3, 3])
    B[0, 0] = 1 - c * (q[1] * q[1] + q[2] * q[2])
    B[0, 1] = -q[2] / 2 + c * (q[0] * q[1])
    B[0, 2] = q[1] / 2 + c * (q[0] * q[2])
    B[1, 0] = q[2] / 2 + c * (q[0] * q[1])
    B[1, 1] = 1 - c * (q[0] * q[0] + q[2] * q[2])
    B[1, 2] = -q[0] / 2 + c * (q[1] * q[2])
    B[2, 0] = -q[1] / 2 + c * (q[0] * q[2])
    B[2, 1] = q[0] / 2 + c * (q[1] * q[2])
    B[2, 2] = 1 - c * (q[0] * q[0] + q[1] * q[1])

    return B


def dEP(q, w):
    """
    dEP(Q,W)

    	dq = dEP(Q,W) returns the euler parameter derivative
    	for a given euler parameter vector Q and body
    	angular velocity vector w.

    	dQ/dt = 1/2 [B(Q)] w
    """

    return .5 * np.dot(BmatEP(q), w)


def dEuler121(q, w):
    """
    dEuler121(Q,W)

    	dq = dEuler121(Q,W) returns the (1-2-1) euler angle derivative
    	vector for a given (1-2-1) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler121(q), w)


def dEuler123(q, w):
    """
    dEuler123(Q,W)

    	dq = dEuler123(Q,W) returns the (1-2-3) euler angle derivative
    	vector for a given (1-2-3) euler angle vector Q and body
    	angular velocity vector w.

        dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler123(q), w)


def dEuler131(q, w):
    """
    dEuler131(Q,W)

    	dq = dEuler131(Q,W) returns the (1-3-1) euler angle derivative
    	vector for a given (1-3-1) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler131(q), w)


def dEuler132(q, w):
    """
    dEuler132(Q,W)

    	dq = dEuler132(Q,W) returns the (1-3-2) euler angle derivative
    	vector for a given (1-3-2) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler132(q), w)


def dEuler212(q, w):
    """
    dEuler212(Q,W)

    	dq = dEuler212(Q,W) returns the (2-1-2) euler angle derivative
    	vector for a given (2-1-2) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler212(q), w)


def dEuler213(q, w):
    """
    dEuler213(Q,W)

    	dq = dEuler213(Q,W) returns the (2-1-3) euler angle derivative
    	vector for a given (2-1-3) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler213(q), w)


def dEuler231(q, w):
    """
    dEuler231(Q,W)

    	dq = dEuler231(Q,W) returns the (2-3-1) euler angle derivative
    	vector for a given (2-3-1) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler231(q), w)


def dEuler232(q, w):
    """
    dEuler232(Q,W)

    	dq = dEuler232(Q,W) returns the (2-3-2) euler angle derivative
    	vector for a given (2-3-2) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler232(q), w)


def dEuler312(q, w):
    """
    dEuler312(Q,W)

    	dq = dEuler312(Q,W) returns the (3-1-2) euler angle derivative
    	vector for a given (3-1-2) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler312(q), w)


def dEuler313(q, w):
    """
    dEuler313(Q,W)

    	dq = dEuler313(Q,W) returns the (3-1-3) euler angle derivative
    	vector for a given (3-1-3) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler313(q), w)


def dEuler321(q, w):
    """
    dEuler321(Q,W)

    	dq = dEuler321(Q,W) returns the (3-2-1) euler angle derivative
    	vector for a given (3-2-1) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler321(q), w)


def dEuler323(q, w):
    """
    dEuler323(Q,W)

    	dq = dEuler323(Q,W) returns the (3-2-3) euler angle derivative
    	vector for a given (3-2-3) euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatEuler323(q), w)


def dGibbs(q, w):
    """
    dGibbs(Q,W)

    	dq = dGibbs(Q,W) returns the gibbs derivative
    	for a given gibbs vector Q and body
    	angular velocity vector w.

    	dQ/dt = 1/2 [B(Q)] w
    """

    return .5 * np.dot(BmatGibbs(q), w)


def dMRP(q, w):
    """
    dMRP(Q,W)

    	dq = dMRP(Q,W) returns the MRP derivative
    	for a given MRP vector Q and body
    	angular velocity vector w.

    	dQ/dt = 1/4 [B(Q)] w
    """

    return .25 * np.dot(BmatMRP(q), w)


def dMRP2Omega(q, dq):
    """
    dMRP(Q,dQ)

    	W = dMRP(Q,dQ) returns the angular rate
    	for a given MRP set q MRP derivative dq.

        W = 4 [B(Q)]^(-1) dQ
    """

    return 4 * np.matmul(BinvMRP(q), dq)


def ddMRP(q, dq, w, dw):
    """
    dMRP(Q,dQ,W,dW)

    	ddQ = ddMRP(Q,dQ,W,dW) returns the MRP second derivative
    	for a given MRP vector q, MRP derivative dq, body
    	angular velocity vector w and body angulat acceleration
        vector dw.

    	(d^2Q)/(dt^2) = 1/4 ( [B(Q)] dw + [Bdot(Q,dQ)] w )
    """

    return .25 * ( np.dot(BmatMRP(q), dw) + np.dot(BdotmatMRP(q, dq), w) )

def ddMRP2dOmega(q, dq, ddq):
    """
    ddMRP2dOmega(Q,dQ,ddQ)

    	dW = ddMRP2dOmega(Q,dQ,ddQ) returns the body angular acceleration
        dW given the MRP vector Q, the MRP derivative dQ and the MRP
        second order derivative ddQ.

    	dW/dt = 4 [B(Q)]^(-1) ( ddQ - [Bdot(Q,dQ)] [B(Q)]^(-1) dQ )
    """

    Binv = BinvMRP(q)
    Bdot = BdotmatMRP(q, dq)

    return 4 * np.dot(Binv, (ddq - np.dot(Bdot, np.dot(Binv, dq))) )


def dPRV(q, w):
    """
    dPRV(Q,W)

    	dq = dPRV(Q,W) returns the PRV derivative
    	for a given PRV vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return np.dot(BmatPRV(q), w)


def elem2PRV(r):
    """
    elem2PRV(R)

    	Q = elem2PRV(R) translates a prinicpal rotation
    	element set R into the corresponding principal
    	rotation vector Q.
    """

    q0 = r[1] * r[0]
    q1 = r[2] * r[0]
    q2 = r[3] * r[0]
    q = np.array([q0, q1, q2])

    return q


def gibbs2C(q):
    """
    gibbs2C

    	C = gibbs2C(Q) returns the direction cosine
    	matrix in terms of the 3x1 gibbs vector Q.
    """

    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    qm = np.linalg.norm(q)
    d1 = qm * qm
    C = np.zeros([3, 3])
    C[0, 0] = 1 + 2 * q1 * q1 - d1
    C[0, 1] = 2 * (q1 * q2 + q3)
    C[0, 2] = 2 * (q1 * q3 - q2)
    C[1, 0] = 2 * (q2 * q1 - q3)
    C[1, 1] = 1 + 2 * q2 * q2 - d1
    C[1, 2] = 2 * (q2 * q3 + q1)
    C[2, 0] = 2 * (q3 * q1 + q2)
    C[2, 1] = 2 * (q3 * q2 - q1)
    C[2, 2] = 1 + 2 * q3 * q3 - d1
    C = C / (1 + d1)
    return C


def gibbs2EP(q1):
    """
    gibbs2EP(Q1)

    	Q = gibbs2EP(Q1) translates the gibbs vector Q1
    	into the euler parameter vector Q.
    """

    qm = np.linalg.norm(q1)
    ps = np.sqrt(1 + qm * qm)
    q = np.array([
        1 / ps,
        q1[0] / ps,
        q1[1] / ps,
        q1[2] / ps
    ])
    return q


def gibbs2Euler121(q):
    """
    gibbs2Euler121(Q)

    	E = gibbs2Euler121(Q) translates the gibbs
        vector Q into the (1-2-1) euler angle vector E.
    """

    return EP2Euler121(gibbs2EP(q))


def gibbs2Euler123(q):
    """
    gibbs2Euler123(Q)

    	E = gibbs2Euler123(Q) translates the gibbs
    	 vector Q into the (1-2-3) euler angle vector E.
    """

    return EP2Euler123(gibbs2EP(q))


def gibbs2Euler131(q):
    """
    gibbs2Euler131(Q)

    	E = gibbs2Euler131(Q) translates the gibbs
    	 vector Q into the (1-3-1) euler angle vector E.
    """

    return EP2Euler131(gibbs2EP(q))


def gibbs2Euler132(q):
    """
    gibbs2Euler132(Q)

    	E = gibbs2Euler132(Q) translates the gibbs
        vector Q into the (1-3-2) euler angle vector E.
    """

    return EP2Euler132(gibbs2EP(q))


def gibbs2Euler212(q):
    """
    gibbs2Euler212(Q)

    	E = gibbs2Euler212(Q) translates the gibbs
    	 vector Q into the (2-1-2) euler angle vector E.
    """

    return EP2Euler212(gibbs2EP(q))


def gibbs2Euler213(q):
    """
    gibbs2Euler213(Q)

    	E = gibbs2Euler213(Q) translates the gibbs
    	 vector Q into the (2-1-3) euler angle vector E.
    """

    return EP2Euler213(gibbs2EP(q))


def gibbs2Euler231(q):
    """
    gibbs2Euler231(Q)

    	E = gibbs2Euler231(Q) translates the gibbs
    	 vector Q into the (2-3-1) euler angle vector E.
    """

    return EP2Euler231(gibbs2EP(q))


def gibbs2Euler232(q):
    """
    gibbs2Euler232(Q)

    	E = gibbs2Euler232(Q) translates the gibbs
    	 vector Q into the (2-3-2) euler angle vector E.
    """

    return EP2Euler232(gibbs2EP(q))


def gibbs2Euler312(q):
    """
    gibbs2Euler312(Q)

    	E = gibbs2Euler312(Q) translates the gibbs
    	 vector Q into the (3-1-2) euler angle vector E.
    """

    return EP2Euler312(gibbs2EP(q))


def gibbs2Euler313(q):
    """
    gibbs2Euler313(Q)

    	E = gibbs2Euler313(Q) translates the gibbs
    	 vector Q into the (3-1-3) euler angle vector E.
    """

    return EP2Euler313(gibbs2EP(q))


def gibbs2Euler321(q):
    """
    gibbs2Euler321(Q)

    	E = gibbs2Euler321(Q) translates the gibbs
    	 vector Q into the (3-2-1) euler angle vector E.
    """

    return EP2Euler321(gibbs2EP(q))


def gibbs2Euler323(q):
    """
    gibbs2Euler323(Q)

    	E = gibbs2Euler323(Q) translates the gibbs
    	 vector Q into the (3-2-3) euler angle vector E.
    """

    return EP2Euler323(gibbs2EP(q))


def gibbs2MRP(q1):
    """
    gibbs2MRP(Q1)

    	Q = gibbs2MRP(Q1) translates the gibbs vector Q1
    	into the MRP vector Q.
    """

    return q1 / (1 + math.sqrt(1 + np.dot(q1, q1)))


def gibbs2PRV(q):
    """
    gibbs2PRV(Q)

    	Q = gibbs2PRV(Q1) translates the gibbs vector Q1
    	into the principal rotation vector Q.
    """

    tp = np.linalg.norm(q)
    p = 2 * math.atan(tp)
    q0 = q[0] / tp * p
    q1 = q[1] / tp * p
    q2 = q[2] / tp * p
    q = np.array([q0, q1, q2])
    return q


def MRP2C(q):
    """
    MRP2C

    	C = MRP2C(Q) returns the direction cosine
    	matrix in terms of the 3x1 MRP vector Q.
    """

    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    qm = np.linalg.norm(q)
    d1 = qm * qm
    S = 1 - d1
    d = (1 + d1) * (1 + d1)
    C = np.zeros((3, 3))
    C[0, 0] = 4 * (2 * q1 * q1 - d1) + S * S
    C[0, 1] = 8 * q1 * q2 + 4 * q3 * S
    C[0, 2] = 8 * q1 * q3 - 4 * q2 * S
    C[1, 0] = 8 * q2 * q1 - 4 * q3 * S
    C[1, 1] = 4 * (2 * q2 * q2 - d1) + S * S
    C[1, 2] = 8 * q2 * q3 + 4 * q1 * S
    C[2, 0] = 8 * q3 * q1 + 4 * q2 * S
    C[2, 1] = 8 * q3 * q2 - 4 * q1 * S
    C[2, 2] = 4 * (2 * q3 * q3 - d1) + S * S
    C = C / d
    return C


def MRP2EP(q1):
    """
    MRP2EP(Q1)

    	Q = MRP2EP(Q1) translates the MRP vector Q1
    	into the euler parameter vector Q.
    """
    qm = np.linalg.norm(q1)
    ps = 1 + qm * qm
    q = np.array([
        (1 - qm * qm) / ps,
        2 * q1[0] / ps,
        2 * q1[1] / ps,
        2 * q1[2] / ps
    ])
    return q


def MRP2Euler121(q):
    """
    MRP2Euler121(Q)

    	E = MRP2Euler121(Q) translates the MRP
    	 vector Q into the (1-2-1) euler angle vector E.
    """

    return EP2Euler121(MRP2EP(q))


def MRP2Euler123(q):
    """
    MRP2Euler123(Q)

    	E = MRP2Euler123(Q) translates the MRP
    	 vector Q into the (1-2-3) euler angle vector E.
    """

    return EP2Euler123(MRP2EP(q))


def MRP2Euler131(q):
    """
    MRP2Euler131(Q)

    	E = MRP2Euler131(Q) translates the MRP
    	 vector Q into the (1-3-1) euler angle vector E.
    """

    return EP2Euler131(MRP2EP(q))


def MRP2Euler132(q):
    """
    MRP2Euler132(Q)

    	E = MRP2Euler132(Q) translates the MRP
    	 vector Q into the (1-3-2) euler angle vector E.
    """

    return EP2Euler132(MRP2EP(q))


def MRP2Euler212(q):
    """
    MRP2Euler212(Q)

    	E = MRP2Euler212(Q) translates the MRP
    	 vector Q into the (2-1-2) euler angle vector E.
    """

    return EP2Euler212(MRP2EP(q))


def MRP2Euler213(q):
    """
    MRP2Euler213(Q)

    	E = MRP2Euler213(Q) translates the MRP
    	 vector Q into the (2-1-3) euler angle vector E.
    """

    return EP2Euler213(MRP2EP(q))


def MRP2Euler231(q):
    """
    MRP2Euler231(Q)

    	E = MRP2Euler231(Q) translates the MRP
    	 vector Q into the (2-3-1) euler angle vector E.
    """

    return EP2Euler231(MRP2EP(q))


def MRP2Euler232(q):
    """
    MRP2Euler232(Q)

       E = MRP2Euler232(Q) translates the MRP
    	 vector Q into the (2-3-2) euler angle vector E.
    """

    return EP2Euler232(MRP2EP(q))


def MRP2Euler312(q):
    """
    MRP2Euler312(Q)

    	E = MRP2Euler312(Q) translates the MRP
    	 vector Q into the (3-1-2) euler angle vector E.
    """

    return EP2Euler312(MRP2EP(q))


def MRP2Euler313(q):
    """
    MRP2Euler313(Q)

    	E = MRP2Euler313(Q) translates the MRP
    	 vector Q into the (3-1-3) euler angle vector E.
    """

    return EP2Euler313(MRP2EP(q))


def MRP2Euler321(q):
    """
    MRP2Euler321(Q)

    	E = MRP2Euler321(Q) translates the MRP
    	 vector Q into the (3-2-1) euler angle vector E.
    """

    return EP2Euler321(MRP2EP(q))


def MRP2Euler323(q):
    """
    MRP2Euler323(Q)

    	E = MRP2Euler323(Q) translates the MRP
    	 vector Q into the (3-2-3) euler angle vector E.
    """

    return EP2Euler323(MRP2EP(q))


def MRP2Gibbs(q1):
    """
    MRP2Gibbs(Q1)

    	Q = MRP2Gibbs(Q1) translates the MRP vector Q1
    	into the gibbs vector Q.
    """

    return 2 * q1 / (1 - np.dot(q1, q1))


def MRP2PRV(q):
    """
    MRP2PRV(Q1)

    	Q = MRP2PRV(Q1) translates the MRP vector Q1
    	into the principal rotation vector Q.
    """

    tp = np.linalg.norm(q)
    p = 4 * math.atan(tp)
    q0 = q[0] / tp * p
    q1 = q[1] / tp * p
    q2 = q[2] / tp * p
    q = np.array([q0, q1, q2])

    return q


def MRPswitch(q, s2):
    """
    MRPswitch

    	S = MRPswitch(Q,s2) checks to see if norm(Q) is larger than s2.
    	If yes, then the MRP vector Q is mapped to its shadow set.
    """

    q2 = np.dot(q, q)
    if (q2 > s2 * s2):
        s = -q / q2
    else:
        s = q

    return s


def PRV2C(q):
    """
    PRV2C

    	C = PRV2C(Q) returns the direction cosine
    	matrix in terms of the 3x1 principal rotation vector
    	Q.
    """

    q0 = np.linalg.norm(q)
    if q0 == 0.0:
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
    else:
        q1 = q[0] / q0
        q2 = q[1] / q0
        q3 = q[2] / q0
    cp = np.cos(q0)
    sp = np.sin(q0)
    d1 = 1 - cp
    C = np.zeros((3, 3))
    C[0, 0] = q1 * q1 * d1 + cp
    C[0, 1] = q1 * q2 * d1 + q3 * sp
    C[0, 2] = q1 * q3 * d1 - q2 * sp
    C[1, 0] = q2 * q1 * d1 - q3 * sp
    C[1, 1] = q2 * q2 * d1 + cp
    C[1, 2] = q2 * q3 * d1 + q1 * sp
    C[2, 0] = q3 * q1 * d1 + q2 * sp
    C[2, 1] = q3 * q2 * d1 - q1 * sp
    C[2, 2] = q3 * q3 * d1 + cp
    return C


def PRV2EP(qq1):
    """"
    PRV2EP(Q1)

    	Q = PRV2EP(Q1) translates the principal rotation vector Q1
    	into the euler parameter vector Q.
    """

    q = np.zeros(4)
    q1 = PRV2elem(qq1)
    sp = math.sin(q1[0] / 2)
    q[0] = math.cos(q1[0] / 2)
    q[1] = q1[1] * sp
    q[2] = q1[2] * sp
    q[3] = q1[3] * sp

    return q


def PRV2Euler121(q):
    """
    PRV2Euler121(Q)

    	E = PRV2Euler121(Q) translates the principal rotation
    	vector Q into the (1-2-1) euler angle vector E.
    """

    return EP2Euler121(PRV2EP(q))


def PRV2Euler123(q):
    """
    PRV2Euler123(Q)

    	E = PRV2Euler123(Q) translates the principal rotation
    	vector Q into the (1-2-3) euler angle vector E.
    """

    return EP2Euler123(PRV2EP(q))


def PRV2Euler131(q):
    """
    PRV2Euler131(Q)

    	E = PRV2Euler131(Q) translates the principal rotation
    	vector Q into the (1-3-1) euler angle vector E.
    """

    return EP2Euler131(PRV2EP(q))


def PRV2Euler132(q):
    """
    PRV2Euler132(Q)

    	E = PRV2Euler132(Q) translates the principal rotation
    	vector Q into the (1-3-2) euler angle vector E.
    """

    return EP2Euler132(PRV2EP(q))


def PRV2Euler212(q):
    """
    PRV2Euler212(Q)

    	E = PRV2Euler212(Q) translates the principal rotation
    	vector Q into the (2-1-2) euler angle vector E.
    """

    return EP2Euler212(PRV2EP(q))


def PRV2Euler213(q):
    """
    PRV2Euler213(Q)

    	E = PRV2Euler213(Q) translates the principal rotation
    	vector Q into the (2-1-3) euler angle vector E.
    """

    return EP2Euler213(PRV2EP(q))


def PRV2Euler231(q):
    """
    PRV2Euler231(Q)

    	E = PRV2Euler231(Q) translates the principal rotation
    	vector Q into the (2-3-1) euler angle vector E.
    """

    return EP2Euler231(PRV2EP(q))


def PRV2Euler232(q):
    """
    PRV2Euler232(Q)

    	E = PRV2Euler232(Q) translates the principal rotation
    	vector Q into the (2-3-2) euler angle vector E.
    """

    return EP2Euler232(PRV2EP(q))


def PRV2Euler312(q):
    """
    PRV2Euler312(Q)

    	E = PRV2Euler312(Q) translates the principal rotation
    	vector Q into the (3-1-2) euler angle vector E.
    """

    return EP2Euler312(PRV2EP(q))


def PRV2Euler313(q):
    """
    PRV2Euler313(Q)

    	E = PRV2Euler313(Q) translates the principal rotation
    	vector Q into the (3-1-3) euler angle vector E.
    """

    return EP2Euler313(PRV2EP(q))


def PRV2Euler321(q):
    """
    PRV2Euler321(Q)

    	E = PRV2Euler321(Q) translates the principal rotation
    	vector Q into the (3-2-1) euler angle vector E.
    """

    return EP2Euler321(PRV2EP(q))


def PRV2Euler323(q):
    """
    PRV2Euler323(Q)

    	E = PRV2Euler323(Q) translates the principal rotation
    	vector Q into the (3-2-3) euler angle vector E.
    """

    return EP2Euler323(PRV2EP(q))


def PRV2Gibbs(q):
    """
    PRV2Gibbs(Q1)

    	Q = PRV2Gibbs(Q1) translates the principal rotation vector Q1
    	into the gibbs vector Q.
    """

    q = PRV2elem(q)
    tp = math.tan(q[0] / 2)
    q0 = q[1] * tp
    q1 = q[2] * tp
    q2 = q[3] * tp
    q = np.array([q0, q1, q2])

    return q


def PRV2MRP(q):
    """
     PRV2MRP(Q1)

    	Q = PRV2MRP(Q1) translates the principal rotation vector Q1
    	into the MRP vector Q.
    """

    q = PRV2elem(q)
    tp = math.tan(q[0] / 4)
    q0 = q[1] * tp
    q1 = q[2] * tp
    q2 = q[3] * tp

    q = np.array([q0, q1, q2])
    return q


def subEP(b1, b2):
    """
    subEP(B1,B2)

    	Q = subEP(B1,B2) provides the euler parameter vector
    	which corresponds to relative rotation from B2
    	to B1.
    """

    q = np.zeros(4)
    q[0] = b2[0] * b1[0] + b2[1] * b1[1] + b2[2] * b1[2] + b2[3] * b1[3]
    q[1] = -b2[1] * b1[0] + b2[0] * b1[1] + b2[3] * b1[2] - b2[2] * b1[3]
    q[2] = -b2[2] * b1[0] - b2[3] * b1[1] + b2[0] * b1[2] + b2[1] * b1[3]
    q[3] = -b2[3] * b1[0] + b2[2] * b1[1] - b2[1] * b1[2] + b2[0] * b1[3]

    return q


def subEuler121(e, e1):
    """
    subEuler121(E,E1)

    	E2 = subEuler121(E,E1) computes the relative
    	(1-2-1) euler angle vector from E1 to E.
    """

    cp = math.cos(e[1])
    cp1 = math.cos(e1[1])
    sp = math.sin(e[1])
    sp1 = math.sin(e1[1])
    dum = e[0] - e1[0]

    e2 = np.zeros(3)
    e2[1] = math.acos(cp1 * cp + sp1 * sp * math.cos(dum))
    cp2 = math.cos(e2[1])
    e2[0] = Picheck(-e1[2] + math.atan2(sp1 * sp * math.sin(dum), cp2 * cp1 - cp))
    e2[2] = Picheck(e[2] - math.atan2(sp1 * sp * math.sin(dum), cp1 - cp * cp2))

    return e2


def subEuler123(e, e1):
    """
    subEuler123(E,E1)

    	E2 = subEuler123(E,E1) computes the relative
    	(1-2-3) euler angle vector from E1 to E.
    """

    C = euler1232C(e)
    C1 = euler1232C(e1)
    C2 = np.dot(C, C1.T)
    e2 = C2Euler123(C2)

    return e2


def subEuler131(e, e1):
    """
    subEuler131(E,E1)

    	E2 = subEuler131(E,E1) computes the relative
    	(1-3-1) euler angle vector from E1 to E.
    """

    cp = math.cos(e[1])
    cp1 = math.cos(e1[1])
    sp = math.sin(e[1])
    sp1 = math.sin(e1[1])
    dum = e[0] - e1[0]

    e2 = np.zeros(3)
    e2[1] = math.acos(cp1 * cp + sp1 * sp * math.cos(dum))
    cp2 = math.cos(e2[1])
    e2[0] = Picheck(-e1[2] + math.atan2(sp1 * sp * math.sin(dum), cp2 * cp1 - cp))
    e2[2] = Picheck(e[2] - math.atan2(sp1 * sp * math.sin(dum), cp1 - cp * cp2))

    return e2


def subEuler132(e, e1):
    """
    subEuler132(E,E1)

    	E2 = subEuler132(E,E1) computes the relative
    	(1-3-2) euler angle vector from E1 to E.
    """

    C = euler1322C(e)
    C1 = euler1322C(e1)
    C2 = np.dot(C, C1.T)
    e2 = C2Euler132(C2)

    return e2


def subEuler212(e, e1):
    """
    subEuler212(E,E1)

    	E2 = subEuler212(E,E1) computes the relative
    	(2-1-2) euler angle vector from E1 to E.
    """

    cp = math.cos(e[1])
    cp1 = math.cos(e1[1])
    sp = math.sin(e[1])
    sp1 = math.sin(e1[1])
    dum = e[0] - e1[0]

    e2 = np.zeros(3)
    e2[1] = math.acos(cp1 * cp + sp1 * sp * math.cos(dum))
    cp2 = math.cos(e2[1])
    e2[0] = Picheck(-e1[2] + math.atan2(sp1 * sp * math.sin(dum), cp2 * cp1 - cp))
    e2[2] = Picheck(e[2] - math.atan2(sp1 * sp * math.sin(dum), cp1 - cp * cp2))

    return e2


def subEuler213(e, e1):
    """
    subEuler213(E,E1)

    	E2 = subEuler213(E,E1) computes the relative
    	(2-1-3) euler angle vector from E1 to E.
    """

    C = euler2132C(e)
    C1 = euler2132C(e1)
    C2 = np.dot(C, C1.T)
    e2 = C2Euler213(C2)

    return e2


def subEuler231(e, e1):
    """
    subEuler231(E,E1)

    	E2 = subEuler231(E,E1) computes the relative
    	(2-3-1) euler angle vector from E1 to E.
    """

    C = euler2312C(e)
    C1 = euler2312C(e1)
    C2 = np.dot(C, C1.T)
    e2 = C2Euler231(C2)

    return e2


def subEuler232(e, e1):
    """
    subEuler232(E,E1)

    	E2 = subEuler232(E,E1) computes the relative
    	(2-3-2) euler angle vector from E1 to E.
    """

    cp = math.cos(e[1])
    cp1 = math.cos(e1[1])
    sp = math.sin(e[1])
    sp1 = math.sin(e1[1])
    dum = e[0] - e1[0]

    e2 = np.zeros(3)
    e2[1] = math.acos(cp1 * cp + sp1 * sp * math.cos(dum))
    cp2 = math.cos(e2[1])
    e2[0] = Picheck(-e1[2] + math.atan2(sp1 * sp * math.sin(dum), cp2 * cp1 - cp))
    e2[2] = Picheck(e[2] - math.atan2(sp1 * sp * math.sin(dum), cp1 - cp * cp2))

    return e2


def subEuler312(e, e1):
    """
    subEuler312(E,E1)

    	E2 = subEuler312(E,E1) computes the relative
    	(3-1-2) euler angle vector from E1 to E.
    """

    C = euler3122C(e)
    C1 = euler3122C(e1)
    C2 = np.dot(C, C1.T)
    e2 = C2Euler312(C2)

    return e2


def subEuler313(e, e1):
    """
    subEuler313(E,E1)

    	E2 = subEuler313(E,E1) computes the relative
    	(3-1-3) euler angle vector from E1 to E.
    """

    cp = math.cos(e[1])
    cp1 = math.cos(e1[1])
    sp = math.sin(e[1])
    sp1 = math.sin(e1[1])
    dum = e[0] - e1[0]

    e2 = np.zeros(3)
    e2[1] = math.acos(cp1 * cp + sp1 * sp * math.cos(dum))
    cp2 = math.cos(e2[1])
    e2[0] = Picheck(-e1[2] + math.atan2(sp1 * sp * math.sin(dum), cp2 * cp1 - cp))
    e2[2] = Picheck(e[2] - math.atan2(sp1 * sp * math.sin(dum), cp1 - cp * cp2))

    return e2


def subEuler321(e, e1):
    """
    subEuler321(E,E1)

    	E2 = subEuler321(E,E1) computes the relative
    	(3-2-1) euler angle vector from E1 to E.
    """

    C = euler3212C(e)
    C1 = euler3212C(e1)
    C2 = np.dot(C, C1.T)
    e2 = C2Euler321(C2)

    return e2


def subEuler323(e, e1):
    """
    subEuler323(E,E1)

    	E2 = subEuler323(E,E1) computes the relative
    	(3-2-3) euler angle vector from E1 to E.
    """

    cp = math.cos(e[1])
    cp1 = math.cos(e1[1])
    sp = math.sin(e[1])
    sp1 = math.sin(e1[1])
    dum = e[0] - e1[0]

    e2 = np.zeros(3)
    e2[1] = math.acos(cp1 * cp + sp1 * sp * math.cos(dum))
    cp2 = math.cos(e2[1])
    e2[0] = Picheck(-e1[2] + math.atan2(sp1 * sp * math.sin(dum), cp2 * cp1 - cp))
    e2[2] = Picheck(e[2] - math.atan2(sp1 * sp * math.sin(dum), cp1 - cp * cp2))

    return e2


def subGibbs(q1, q2):
    """
    subGibbs(Q1,Q2)

    	Q = subGibbs(Q1,Q2) provides the gibbs vector
    	which corresponds to relative rotation from Q2
    	to Q1.
    """
    return (q1 - q2 + np.cross(q1, q2)) / (1. + np.dot(q1, q2))


def subMRP(q1, q2):
    """
    subMRP(Q1,Q2)

    	Q = subMRP(Q1,Q2) provides the MRP vector
    	which corresponds to relative rotation from Q2
    	to Q1.
    """

    den = 1 + np.dot(q1, q1) * np.dot(q2, q2) + 2 * np.dot(q1, q2)
    if den < 1e-5:
        q2 = -q2/np.dot(q2,q2)
        den = 1 + np.dot(q1, q1) * np.dot(q2, q2) + 2 * np.dot(q1, q2)
    num = (1 - np.dot(q2, q2)) * q1 - (1 - np.dot(q1, q1)) * q2 + 2 * np.cross(q1, q2)

    q = num / den
    if np.dot(q,q) > 1:
        q = -q/np.dot(q, q)

    return q


def subPRV(q1, q2):
    """
    subPRV(Q1,Q2)

    	Q = subPRV(Q1,Q2) provides the prinipal rotation vector
    	which corresponds to relative principal rotation from Q2
    	to Q1.
    """

    q1 = PRV2elem(q1)
    q2 = PRV2elem(q2)
    cp1 = math.cos(q1[0] / 2)
    cp2 = math.cos(q2[0] / 2)
    sp1 = math.sin(q1[0] / 2)
    sp2 = math.sin(q2[0] / 2)
    e1 = q1[1:4]
    e2 = q2[1:4]

    p = 2 * math.acos(cp1 * cp2 + sp1 * sp2 * np.dot(e1, e2))
    sp = math.sin(p / 2)
    e = (-cp1 * sp2 * e2 + cp2 * sp1 * e1 + sp1 * sp2 * np.cross(e1, e2)) / sp
    q = p * e

    return q


def EP2C(q):
    """
	EP2C

        C = EP2C(Q) returns the direction math.cosine
        matrix in terms of the 4x1 euler parameter vector
        Q.  The first element is the non-dimensional euler
        parameter, while the remain three elements form
        the eulerparameter vector.
	"""
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]
    C = np.zeros([3, 3])
    C[0, 0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3
    C[0, 1] = 2 * (q1 * q2 + q0 * q3)
    C[0, 2] = 2 * (q1 * q3 - q0 * q2)
    C[1, 0] = 2 * (q1 * q2 - q0 * q3)
    C[1, 1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3
    C[1, 2] = 2 * (q2 * q3 + q0 * q1)
    C[2, 0] = 2 * (q1 * q3 + q0 * q2)
    C[2, 1] = 2 * (q2 * q3 - q0 * q1)
    C[2, 2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
    return C


def EP2Euler121(q):
    """
	EP2Euler121(Q)

        E = EP2Euler121(Q) translates the euler parameter
        vector Q into the corresponding (1-2-1) euler angle
        vector E.
	"""

    t1 = math.atan2(q[3], q[2])
    t2 = math.atan2(q[1], q[0])

    e1 = t1 + t2
    e2 = 2 * math.acos(math.sqrt(q[0] * q[0] + q[1] * q[1]))
    e3 = t2 - t1

    e = np.array([e1, e2, e3])
    return e


def EP2Euler123(q):
    """
	EP2Euler123

        Q = EP2Euler123(Q) translates the euler parameter vector
        Q into the corresponding (1-2-3) euler angle set.
	"""

    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    e1 = math.atan2(-2 * (q2 * q3 - q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)
    e2 = math.asin(2 * (q1 * q3 + q0 * q2))
    e3 = math.atan2(-2 * (q1 * q2 - q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)

    e = np.array([e1, e2, e3])
    return e


def EP2Euler131(q):
    """
	EP2Euler131(Q)

        E = EP2Euler131(Q) translates the euler parameter
        vector Q into the corresponding (1-3-1) euler angle
        vector E.
	"""

    t1 = math.atan2(q[2], q[3])
    t2 = math.atan2(q[1], q[0])

    e1 = t2 - t1
    e2 = 2 * math.acos(math.sqrt(q[0] * q[0] + q[1] * q[1]))
    e3 = t2 + t1

    e = np.array([e1, e2, e3])
    return e


def EP2Euler132(q):
    """
    EP2Euler132

    	E = EP2Euler132(Q) translates the euler parameter vector
    	Q into the corresponding (1-3-2) euler angle set.

    """
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    e1 = math.atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3)
    e2 = math.asin(-2 * (q1 * q2 - q0 * q3))
    e3 = math.atan2(2 * (q1 * q3 + q0 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)

    e = np.array([e1, e2, e3])
    return e


def EP2Euler212(q):
    """
    EP2Euler212(Q)

        E = EP2Euler212(Q) translates the euler parameter
        vector Q into the corresponding (2-1-2) euler angle
        vector E.
    """

    t1 = math.atan2(q[3], q[1])
    t2 = math.atan2(q[2], q[0])

    e1 = t2 - t1
    e2 = 2 * math.acos(math.sqrt(q[0] * q[0] + q[2] * q[2]))
    e3 = t2 + t1

    e = np.array([e1, e2, e3])
    return e


def EP2Euler213(q):
    """
    EP2Euler213

    	Q = EP2Euler213(Q) translates the euler parameter vector
    	Q into the corresponding (2-1-3) euler angle set.
    """

    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    e1 = math.atan2(2 * (q1 * q3 + q0 * q2), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)
    e2 = math.asin(-2 * (q2 * q3 - q0 * q1))
    e3 = math.atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3)

    e = np.array([e1, e2, e3])
    return e


def EP2Euler231(q):
    """
    EP2Euler231

    	E = EP2Euler231(Q) translates the euler parameter vector
    	Q into the corresponding (2-3-1) euler angle set.
    """

    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    e1 = math.atan2(-2 * (q1 * q3 - q0 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)
    e2 = math.asin(2 * (q1 * q2 + q0 * q3))
    e3 = math.atan2(-2 * (q2 * q3 - q0 * q1), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3)

    e = np.array([e1, e2, e3])
    return e


def EP2Euler232(q):
    """
    EP2Euler232(Q)

    	E = EP2Euler232(Q) translates the euler parameter
    	vector Q into the corresponding (2-3-2) euler angle
    	vector E.
    """

    t1 = math.atan2(q[1], q[3])
    t2 = math.atan2(q[2], q[0])

    e1 = t1 + t2
    e2 = 2 * math.acos(math.sqrt(q[0] * q[0] + q[2] * q[2]))
    e3 = t2 - t1

    e = np.array([e1, e2, e3])
    return e


def EP2Euler312(q):
    """
    EP2Euler312

    	E = EP2Euler312(Q) translates the euler parameter vector
    	Q into the corresponding (3-1-2) euler angle set.
    """

    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    e1 = math.atan2(-2 * (q1 * q2 - q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3)
    e2 = math.asin(2 * (q2 * q3 + q0 * q1))
    e3 = math.atan2(-2 * (q1 * q3 - q0 * q2), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)

    e = np.array([e1, e2, e3])
    return e


def EP2Euler313(q):
    """
    EP2Euler313(Q)

    	E = EP2Euler313(Q) translates the euler parameter
    	vector Q into the corresponding (3-1-3) euler angle
    	vector E.
    """

    t1 = math.atan2(q[2], q[1])
    t2 = math.atan2(q[3], q[0])

    e1 = t1 + t2
    e2 = 2 * math.acos(math.sqrt(q[0] * q[0] + q[3] * q[3]))
    e3 = t2 - t1

    e = np.array([e1, e2, e3])
    return e


def EP2Euler321(q):
    """
    EP2Euler321

    	E = EP2Euler321(Q) translates the euler parameter vector
    	Q into the corresponding (3-2-1) euler angle set.
    """

    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    e1 = math.atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)
    e2 = math.asin(-2 * (q1 * q3 - q0 * q2))
    e3 = math.atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)

    e = np.array([e1, e2, e3])
    return e


def EP2Euler323(q):
    """
    EP2Euler323(Q)

    	E = EP2Euler323(Q) translates the euler parameter
    	vector Q into the corresponding (3-2-3) euler angle
    	vector E.
    """

    t1 = math.atan2(q[1], q[2])
    t2 = math.atan2(q[3], q[0])

    e1 = t2 - t1
    e2 = 2 * math.acos(math.sqrt(q[0] * q[0] + q[3] * q[3]))
    e3 = t2 + t1

    e = np.array([e1, e2, e3])
    return e


def EP2Gibbs(q):
    """
    EP2Gibbs(Q1)

    	Q = EP2Gibbs(Q1) translates the euler parameter vector Q1
    	into the gibbs vector Q.
    """

    q1 = q[1] / q[0]
    q2 = q[2] / q[0]
    q3 = q[3] / q[0]

    return np.array([q1, q2, q3])


def EP2MRP(q):
    """
    EP2MRP(Q1)
        Q = EP2MRP(Q1) translates the euler parameter vector Q1
        into the MRP vector Q.
    """

    if q[0] < 0:
        q = -q

    q1 = q[1] / (1 + q[0])
    q2 = q[2] / (1 + q[0])
    q3 = q[3] / (1 + q[0])

    return np.array([q1, q2, q3])


def EP2PRV(q):
    """
    EP2PRV(Q1)

    	Q = EP2PRV(Q1) translates the euler parameter vector Q1
    	into the principal rotation vector Q.
    """

    p = 2 * math.acos(q[0])
    sp = math.sin(p / 2)
    q1 = q[1] / sp * p
    q2 = q[2] / sp * p
    q3 = q[3] / sp * p

    return np.array([q1, q2, q3])


def euler1(x):
    """
	EULER1 	Elementary rotation matrix
	Returns the elementary rotation matrix about the first body axis.
	"""
    m = np.identity(3)
    m[1, 1] = math.cos(x)
    m[1, 2] = math.sin(x)
    m[2, 1] = -m[1, 2]
    m[2, 2] = m[1, 1]

    return m


def euler2(x):
    """
	EULER2 	Elementary rotation matrix
	Returns the elementary rotation matrix about the
	second body axis.
	"""
    m = np.identity(3)
    m[0, 0] = math.cos(x)
    m[0, 2] = -math.sin(x)
    m[2, 0] = -m[0, 2]
    m[2, 2] = m[0, 0]

    return m


def euler3(x):
    """
	EULER3 	Elementary rotation matrix
	Returns the elementary rotation matrix about the
	third body axis.
	"""
    m = np.identity(3)
    m[0, 0] = math.cos(x)
    m[0, 1] = math.sin(x)
    m[1, 0] = -m[0, 1]
    m[1, 1] = m[0, 0]

    return m


def euler1212C(q):
    """
	Euler1212C

        C = euler1212C(Q) returns the direction cosine
        matrix in terms of the 1-2-1 euler angles.
        Input Q must be a 3x1 vector of euler angles.
	"""
    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct2
    C[0, 1] = st1 * st2
    C[0, 2] = -ct1 * st2
    C[1, 0] = st2 * st3
    C[1, 1] = ct1 * ct3 - ct2 * st1 * st3
    C[1, 2] = ct3 * st1 + ct1 * ct2 * st3
    C[2, 0] = ct3 * st2
    C[2, 1] = -ct2 * ct3 * st1 - ct1 * st3
    C[2, 2] = ct1 * ct2 * ct3 - st1 * st3

    return C


def euler1212EP(e):
    """
	Euler1212EP(E)

        Q = euler1212EP(E) translates the 121 euler angle
        vector E into the euler parameter vector Q.
	"""

    e1 = e[0] / 2
    e2 = e[1] / 2
    e3 = e[2] / 2

    q0 = math.cos(e2) * math.cos(e1 + e3)
    q1 = math.cos(e2) * math.sin(e1 + e3)
    q2 = math.sin(e2) * math.cos(e1 - e3)
    q3 = math.sin(e2) * math.sin(e1 - e3)

    return np.array([q0, q1, q2, q3])


def euler1212Gibbs(e):
    """
	Euler1212Gibbs(E)

        Q = euler1212Gibbs(E) translates the (1-2-1) euler
        angle vector E into the gibbs vector Q.
	"""

    return EP2Gibbs(euler1212EP(e))


def euler1212MRP(e):
    """
    euler1212MRP(E)

    	Q = euler1212MRP(E) translates the (1-2-1) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler1212EP(e))


def euler1212PRV(e):
    """
    euler1212PRV(E)

    	Q = euler1212PRV(E) translates the (1-2-1) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler1212EP(e))


def euler1232C(q):
    """
    euler1232C

    	C = euler1232C(Q) returns the direction cosine
    	matrix in terms of the 1-2-3 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct2 * ct3
    C[0, 1] = ct3 * st1 * st2 + ct1 * st3
    C[0, 2] = st1 * st3 - ct1 * ct3 * st2
    C[1, 0] = -ct2 * st3
    C[1, 1] = ct1 * ct3 - st1 * st2 * st3
    C[1, 2] = ct3 * st1 + ct1 * st2 * st3
    C[2, 0] = st2
    C[2, 1] = -ct2 * st1
    C[2, 2] = ct1 * ct2

    return C


def euler1232EP(e):
    """
    euler1232EP(E)

    	Q = euler1232EP(E) translates the 123 euler angle
    	vector E into the euler parameter vector Q.
    """

    c1 = math.cos(e[0] / 2)
    s1 = math.sin(e[0] / 2)
    c2 = math.cos(e[1] / 2)
    s2 = math.sin(e[1] / 2)
    c3 = math.cos(e[2] / 2)
    s3 = math.sin(e[2] / 2)

    q0 = c1 * c2 * c3 - s1 * s2 * s3
    q1 = s1 * c2 * c3 + c1 * s2 * s3
    q2 = c1 * s2 * c3 - s1 * c2 * s3
    q3 = c1 * c2 * s3 + s1 * s2 * c3

    return np.array([q0, q1, q2, q3])


def euler1232Gibbs(e):
    """
    euler1232Gibbs(E)

    	Q = euler1232Gibbs(E) translates the (1-2-3) euler
    	angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler1232EP(e))


def euler1232MRP(e):
    """
    euler1232MRP(E)

    	Q = euler1232MRP(E) translates the (1-2-3) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler1232EP(e))


def euler1232PRV(e):
    """
    euler1232PRV(E)

    	Q = euler1232PRV(E) translates the (1-2-3) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler1232EP(e))


def euler1312C(q):
    """
    euler1312C

    	C = euler1312C(Q) returns the direction cosine
    	matrix in terms of the 1-3-1 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct2
    C[0, 1] = ct1 * st2
    C[0, 2] = st1 * st2
    C[1, 0] = -ct3 * st2
    C[1, 1] = ct1 * ct2 * ct3 - st1 * st3
    C[1, 2] = ct2 * ct3 * st1 + ct1 * st3
    C[2, 0] = st2 * st3
    C[2, 1] = -ct3 * st1 - ct1 * ct2 * st3
    C[2, 2] = ct1 * ct3 - ct2 * st1 * st3

    return C


def euler1312EP(e):
    """
    euler1312EP(E)

    	Q = euler1312EP(E) translates the 131 euler angle
    	vector E into the euler parameter vector Q.
    """

    e1 = e[0] / 2
    e2 = e[1] / 2
    e3 = e[2] / 2

    q0 = math.cos(e2) * math.cos(e1 + e3)
    q1 = math.cos(e2) * math.sin(e1 + e3)
    q2 = math.sin(e2) * math.sin(-e1 + e3)
    q3 = math.sin(e2) * math.cos(-e1 + e3)

    return np.array([q0, q1, q2, q3])


def euler1312Gibbs(e):
    """
    euler1312Gibbs(E)

    	Q = euler1312Gibbs(E) translates the (1-3-1) euler
    	angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler1312EP(e))


def euler1312MRP(e):
    """
    euler1312MRP(E)

    	Q = euler1312MRP(E) translates the (1-3-1) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler1312EP(e))


def euler1312PRV(e):
    """
    euler1312PRV(E)

    	Q = euler1312PRV(E) translates the (1-3-1) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler1312EP(e))


def euler1322C(q):
    """
    euler1322C

    	C = euler1322C(Q) returns the direction cosine
    	matrix in terms of the 1-3-2 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct2 * ct3
    C[0, 1] = ct1 * ct3 * st2 + st1 * st3
    C[0, 2] = ct3 * st1 * st2 - ct1 * st3
    C[1, 0] = -st2
    C[1, 1] = ct1 * ct2
    C[1, 2] = ct2 * st1
    C[2, 0] = ct2 * st3
    C[2, 1] = -ct3 * st1 + ct1 * st2 * st3
    C[2, 2] = ct1 * ct3 + st1 * st2 * st3

    return C


def euler1322EP(e):
    """
    euler1322EP(E)

    	Q = euler1322EP(E) translates the 132 euler angle
    	vector E into the euler parameter vector Q.
    """

    c1 = math.cos(e[0] / 2)
    s1 = math.sin(e[0] / 2)
    c2 = math.cos(e[1] / 2)
    s2 = math.sin(e[1] / 2)
    c3 = math.cos(e[2] / 2)
    s3 = math.sin(e[2] / 2)

    q0 = c1 * c2 * c3 + s1 * s2 * s3
    q1 = s1 * c2 * c3 - c1 * s2 * s3
    q2 = c1 * c2 * s3 - s1 * s2 * c3
    q3 = c1 * s2 * c3 + s1 * c2 * s3

    return np.array([q0, q1, q2, q3])


def euler1322Gibbs(e):
    """
    euler1322Gibbs(E)

    	Q = euler1322Gibbs(E) translates the (1-3-2) euler
    	angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler1322EP(e))


def euler1322MRP(e):
    """
    euler1322MRP(E)

    	Q = euler1322MRP(E) translates the (1-3-2) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler1322EP(e))


def euler1322PRV(e):
    """
    euler1322PRV(E)

    	Q = euler1322PRV(E) translates the (1-3-2) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler1322EP(e))


def euler2122C(q):
    """
    euler2122C

    	C = euler2122C(Q) returns the direction cosine
    	matrix in terms of the 2-1-2 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct1 * ct3 - ct2 * st1 * st3
    C[0, 1] = st2 * st3
    C[0, 2] = -ct3 * st1 - ct1 * ct2 * st3
    C[1, 0] = st1 * st2
    C[1, 1] = ct2
    C[1, 2] = ct1 * st2
    C[2, 0] = ct2 * ct3 * st1 + ct1 * st3
    C[2, 1] = -ct3 * st2
    C[2, 2] = ct1 * ct2 * ct3 - st1 * st3

    return C


def euler2132C(q):
    """
    euler2132C

    	C = euler2132C(Q) returns the direction cosine
    	matrix in terms of the 2-1-3 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct1 * ct3 + st1 * st2 * st3
    C[0, 1] = ct2 * st3
    C[0, 2] = -ct3 * st1 + ct1 * st2 * st3
    C[1, 0] = ct3 * st1 * st2 - ct1 * st3
    C[1, 1] = ct2 * ct3
    C[1, 2] = ct1 * ct3 * st2 + st1 * st3
    C[2, 0] = ct2 * st1
    C[2, 1] = -st2
    C[2, 2] = ct1 * ct2

    return C


def euler2312C(q):
    """
    euler2312C

    	C = euler2312C(Q) returns the direction cosine
    	matrix in terms of the 2-3-1 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct1 * ct2
    C[0, 1] = st2
    C[0, 2] = -ct2 * st1
    C[1, 0] = -ct1 * ct3 * st2 + st1 * st3
    C[1, 1] = ct2 * ct3
    C[1, 2] = ct3 * st1 * st2 + ct1 * st3
    C[2, 0] = ct3 * st1 + ct1 * st2 * st3
    C[2, 1] = -ct2 * st3
    C[2, 2] = ct1 * ct3 - st1 * st2 * st3

    return C


def euler2322C(q):
    """
    euler2322C

    	C = euler2322C(Q) returns the direction cosine
    	matrix in terms of the 2-3-2 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct1 * ct2 * ct3 - st1 * st3
    C[0, 1] = ct3 * st2
    C[0, 2] = -ct2 * ct3 * st1 - ct1 * st3
    C[1, 0] = -ct1 * st2
    C[1, 1] = ct2
    C[1, 2] = st1 * st2
    C[2, 0] = ct3 * st1 + ct1 * ct2 * st3
    C[2, 1] = st2 * st3
    C[2, 2] = ct1 * ct3 - ct2 * st1 * st3

    return C


def euler3122C(q):
    """
    euler3122C

    	C = euler3122C(Q) returns the direction cosine
    	matrix in terms of the 1-2-3 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct1 * ct3 - st1 * st2 * st3
    C[0, 1] = ct3 * st1 + ct1 * st2 * st3
    C[0, 2] = -ct2 * st3
    C[1, 0] = -ct2 * st1
    C[1, 1] = ct1 * ct2
    C[1, 2] = st2
    C[2, 0] = ct3 * st1 * st2 + ct1 * st3
    C[2, 1] = st1 * st3 - ct1 * ct3 * st2
    C[2, 2] = ct2 * ct3

    return C


def euler3132C(q):
    """
    euler3132C

    	C = euler3132C(Q) returns the direction cosine
    	matrix in terms of the 3-1-3 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct3 * ct1 - st3 * ct2 * st1
    C[0, 1] = ct3 * st1 + st3 * ct2 * ct1
    C[0, 2] = st3 * st2
    C[1, 0] = -st3 * ct1 - ct3 * ct2 * st1
    C[1, 1] = -st3 * st1 + ct3 * ct2 * ct1
    C[1, 2] = ct3 * st2
    C[2, 0] = st2 * st1
    C[2, 1] = -st2 * ct1
    C[2, 2] = ct2

    return C


def euler3212C(q):
    """
    euler3212C
    	C = euler3212C(Q) returns the direction cosine
    	matrix in terms of the 3-2-1 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct2 * ct1
    C[0, 1] = ct2 * st1
    C[0, 2] = -st2
    C[1, 0] = st3 * st2 * ct1 - ct3 * st1
    C[1, 1] = st3 * st2 * st1 + ct3 * ct1
    C[1, 2] = st3 * ct2
    C[2, 0] = ct3 * st2 * ct1 + st3 * st1
    C[2, 1] = ct3 * st2 * st1 - st3 * ct1
    C[2, 2] = ct3 * ct2

    return C


def euler3232C(q):
    """
    euler3232C

    	C = euler3232C(Q) returns the direction cosine
    	matrix in terms of the 3-2-3 euler angles.
    	Input Q must be a 3x1 vector of euler angles.
    """

    st1 = math.sin(q[0])
    ct1 = math.cos(q[0])
    st2 = math.sin(q[1])
    ct2 = math.cos(q[1])
    st3 = math.sin(q[2])
    ct3 = math.cos(q[2])

    C = np.identity(3)
    C[0, 0] = ct1 * ct2 * ct3 - st1 * st3
    C[0, 1] = ct2 * ct3 * st1 + ct1 * st3
    C[0, 2] = -ct3 * st2
    C[1, 0] = -ct3 * st1 - ct1 * ct2 * st3
    C[1, 1] = ct1 * ct3 - ct2 * st1 * st3
    C[1, 2] = st2 * st3
    C[2, 0] = ct1 * st2
    C[2, 1] = st1 * st2
    C[2, 2] = ct2

    return C


def euler2122EP(e):
    """
    euler2122EP(E)

    	Q = euler2122EP(E) translates the 212 euler angle
    	vector E into the euler parameter vector Q.
    """

    e1 = e[0] / 2
    e2 = e[1] / 2
    e3 = e[2] / 2

    q0 = math.cos(e2) * math.cos(e1 + e3)
    q1 = math.sin(e2) * math.cos(-e1 + e3)
    q2 = math.cos(e2) * math.sin(e1 + e3)
    q3 = math.sin(e2) * math.sin(-e1 + e3)

    return np.array([q0, q1, q2, q3])


def euler2132EP(e):
    """
    euler2132EP(E)

    	Q = euler2132EP(E) translates the 213 euler angle
    	vector E into the euler parameter vector Q.
    """

    c1 = math.cos(e[0] / 2)
    s1 = math.sin(e[0] / 2)
    c2 = math.cos(e[1] / 2)
    s2 = math.sin(e[1] / 2)
    c3 = math.cos(e[2] / 2)
    s3 = math.sin(e[2] / 2)

    q0 = c1 * c2 * c3 + s1 * s2 * s3
    q1 = c1 * s2 * c3 + s1 * c2 * s3
    q2 = s1 * c2 * c3 - c1 * s2 * s3
    q3 = c1 * c2 * s3 - s1 * s2 * c3

    return np.array([q0, q1, q2, q3])


def euler2312EP(e):
    """
    euler2312EP(E)

    	Q = euler2312EP(E) translates the 231 euler angle
    	vector E into the euler parameter vector Q.
    """

    c1 = math.cos(e[0] / 2)
    s1 = math.sin(e[0] / 2)
    c2 = math.cos(e[1] / 2)
    s2 = math.sin(e[1] / 2)
    c3 = math.cos(e[2] / 2)
    s3 = math.sin(e[2] / 2)

    q0 = c1 * c2 * c3 - s1 * s2 * s3
    q1 = c1 * c2 * s3 + s1 * s2 * c3
    q2 = s1 * c2 * c3 + c1 * s2 * s3
    q3 = c1 * s2 * c3 - s1 * c2 * s3

    return np.array([q0, q1, q2, q3])


def euler2322EP(e):
    """
    euler2322EP(E)

    	Q = euler2322EP(E) translates the 232 euler angle
    	vector E into the euler parameter vector Q.
    """

    e1 = e[0] / 2
    e2 = e[1] / 2
    e3 = e[2] / 2

    q0 = math.cos(e2) * math.cos(e1 + e3)
    q1 = math.sin(e2) * math.sin(e1 - e3)
    q2 = math.cos(e2) * math.sin(e1 + e3)
    q3 = math.sin(e2) * math.cos(e1 - e3)

    return np.array([q0, q1, q2, q3])


def euler3122EP(e):
    """
    euler3122EP(E)

    	Q = euler3122EP(E) translates the 312 euler angle
    	vector E into the euler parameter vector Q.
    """

    c1 = math.cos(e[0] / 2)
    s1 = math.sin(e[0] / 2)
    c2 = math.cos(e[1] / 2)
    s2 = math.sin(e[1] / 2)
    c3 = math.cos(e[2] / 2)
    s3 = math.sin(e[2] / 2)

    q0 = c1 * c2 * c3 - s1 * s2 * s3
    q1 = c1 * s2 * c3 - s1 * c2 * s3
    q2 = c1 * c2 * s3 + s1 * s2 * c3
    q3 = s1 * c2 * c3 + c1 * s2 * s3

    return np.array([q0, q1, q2, q3])


def euler3132EP(e):
    """
    euler3132EP(E)

    	Q = euler3132EP(E) translates the 313 euler angle
    	vector E into the euler parameter vector Q.
    """

    e1 = e[0] / 2
    e2 = e[1] / 2
    e3 = e[2] / 2

    q0 = math.cos(e2) * math.cos(e1 + e3)
    q1 = math.sin(e2) * math.cos(e1 - e3)
    q2 = math.sin(e2) * math.sin(e1 - e3)
    q3 = math.cos(e2) * math.sin(e1 + e3)

    return np.array([q0, q1, q2, q3])


def euler3212EP(e):
    """
    euler3212EP(E)
        Q = euler3212EP(E) translates the 321 euler angle
        vector E into the euler parameter vector Q.
    """

    c1 = math.cos(e[0] / 2)
    s1 = math.sin(e[0] / 2)
    c2 = math.cos(e[1] / 2)
    s2 = math.sin(e[1] / 2)
    c3 = math.cos(e[2] / 2)
    s3 = math.sin(e[2] / 2)

    q0 = c1 * c2 * c3 + s1 * s2 * s3
    q1 = c1 * c2 * s3 - s1 * s2 * c3
    q2 = c1 * s2 * c3 + s1 * c2 * s3
    q3 = s1 * c2 * c3 - c1 * s2 * s3

    return np.array([q0, q1, q2, q3])


def euler3232EP(e):
    """
    euler3232EP(E)
        Q = euler3232EP(E) translates the 323 euler angle
        vector E into the euler parameter vector Q.
    """

    e1 = e[0] / 2
    e2 = e[1] / 2
    e3 = e[2] / 2

    q0 = math.cos(e2) * math.cos(e1 + e3)
    q1 = math.sin(e2) * math.sin(-e1 + e3)
    q2 = math.sin(e2) * math.cos(-e1 + e3)
    q3 = math.cos(e2) * math.sin(e1 + e3)

    return np.array([q0, q1, q2, q3])


def euler2122Gibbs(e):
    """
    euler2122Gibbs(E)

    	Q = euler2122Gibbs(E) translates the (2-1-2) euler
    	angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler2122EP(e))


def euler2122MRP(e):
    """
    euler2122MRP(E)

    	Q = euler2122MRP(E) translates the (2-1-2) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler2122EP(e))


def euler2122PRV(e):
    """
    euler2122PRV(E)

    	Q = euler2122PRV(E) translates the (2-1-2) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler2122EP(e))


def euler2132Gibbs(e):
    """
    euler2132Gibbs(E)

    	Q = euler2132Gibbs(E) translates the (2-1-3) euler
    	angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler2132EP(e))


def euler2132MRP(e):
    """
    euler2132MRP(E)

    	Q = euler2132MRP(E) translates the (2-1-3) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler2132EP(e))


def euler2132PRV(e):
    """
    euler2132PRV(E)

    	Q = euler2132PRV(E) translates the (2-1-3) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler2132EP(e))


def euler2312Gibbs(e):
    """
    euler2312Gibbs(E)

    	Q = euler2312Gibbs(E) translates the (2-3-1) euler
    	angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler2312EP(e))


def euler2312MRP(e):
    """
    euler2312MRP(E)

    	Q = euler2312MRP(E) translates the (2-3-1) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler2312EP(e))


def euler2312PRV(e):
    """
    euler2312PRV(E)

    	Q = euler2312PRV(E) translates the (2-3-1) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler2312EP(e))


def euler2322Gibbs(e):
    """
    euler2322Gibbs(E)

    	Q = euler2322Gibbs(E) translates the (2-3-2) euler
    	angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler2322EP(e))


def euler2322MRP(e):
    """
    euler2322MRP(E)

    	Q = euler2322MRP(E) translates the (2-3-2) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler2322EP(e))


def euler2322PRV(e):
    """
    euler2322PRV(E)

    	Q = euler2322PRV(E) translates the (2-3-2) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler2322EP(e))


def euler3122Gibbs(e):
    """
    euler3122Gibbs(E)

    	Q = euler3122Gibbs(E) translates the (3-1-2) euler
    	angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler3122EP(e))


def euler3122MRP(e):
    """
    euler3122MRP(E)

    	Q = euler3122MRP(E) translates the (3-1-2) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler3122EP(e))


def euler3122PRV(e):
    """
    euler3122PRV(E)

    	Q = euler3122PRV(E) translates the (3-1-2) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler3122EP(e))


def euler3132Gibbs(e):
    """
    euler3132Gibbs(E)

    	Q = euler3132Gibbs(E) translates the (3-1-3) euler
    	angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler3132EP(e))


def euler3132MRP(e):
    """
    euler3132MRP(E)

    	Q = euler3132MRP(E) translates the (3-1-3) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler3132EP(e))


def euler3132PRV(e):
    """
    euler3132PRV(E)

    	Q = euler3132PRV(E) translates the (3-1-3) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler3132EP(e))


def euler3212Gibbs(e):
    """
    euler3212Gibbs(E)

    	Q = euler3212Gibbs(E) translates the (3-2-1) euler
    	angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler3212EP(e))


def euler3212MRP(e):
    """
    euler3212MRP(E)
        Q = euler3212MRP(E) translates the (3-2-1) euler
        angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler3212EP(e))


def euler3212PRV(e):
    """
     euler3212PRV(E)

    	Q = euler3212PRV(E) translates the (3-2-1) euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(euler3212EP(e))


def euler3232Gibbs(e):
    """
    euler3232Gibbs(E)
        Q = euler3232Gibbs(E) translates the (3-2-3) euler
        angle vector E into the gibbs vector Q.
    """

    return EP2Gibbs(euler3232EP(e))


def euler3232MRP(e):
    """
    euler3232MRP(E)

    	Q = euler3232MRP(E) translates the (3-2-3) euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(euler3232EP(e))


def euler3232PRV(e):
    """
    euler3232PRV(E)

    	Q = euler3232PRV(E) translates the (3-2-3) euler
    	angle vector Q1 into the principal rotation vector Q.
    """

    return EP2PRV(euler3232EP(e))


def Mi(theta, i):
    c = np.cos(theta)
    s = np.sin(theta)
    case = i
    C = np.zeros((3, 3))
    if case == 1:
        C[0][0] = 1.
        C[0][1] = 0.
        C[0][2] = 0.
        C[1][0] = 0.
        C[1][1] = c
        C[1][2] = s
        C[2][0] = 0.
        C[2][1] = -s
        C[2][2] = c
    elif case == 2:
        C[0][0] = c
        C[0][1] = 0.
        C[0][2] = -s
        C[1][0] = 0.
        C[1][1] = 1.
        C[1][2] = 0.
        C[2][0] = s
        C[2][1] = 0.
        C[2][2] = c
    elif case == 3:
        C[0][0] = c
        C[0][1] = s
        C[0][2] = 0.
        C[1][0] = -s
        C[1][1] = c
        C[1][2] = 0.
        C[2][0] = 0.
        C[2][1] = 0.
        C[2][2] = 1.
    else:
        print('Mi() error: incorrect axis', i, 'selected')
    return C

def v3Tilde(vector):
    x1 = vector[0]
    x2 = vector[1]
    x3 = vector[2]

    xTilde = [[0, -x3, x2]
        ,[x3, 0, -x1]
        ,[-x2, x1, 0]
              ]

    return xTilde
