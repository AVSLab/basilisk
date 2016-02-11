
# Import required modules:
import numpy as np
import math

M_PI = math.pi
D2R  = M_PI/180.
R2D  = 180./M_PI



def Picheck(x):
	"""
        Picheck(x)

        Makes sure that the angle x lies within +/- Pi.
        """
	if x> M_PI:
		return x-2* M_PI
	if x<-M_PI:
		return x+2* M_PI
	return x

def C2EP(C):
    """
    C2EP

    	Q = C2EP(C) translates the 3x3 direction cosine matrix
    	C into the corresponding 4x1 Euler parameter vector Q,
    	where the first component of Q is the non-dimensional
    	Euler parameter Beta_0 >= 0. Transformation is done
        using the Stanley method.
    """

    tr = np.trace(C);
    b2 = np.matrix('1.;0.;0.;0.');
    b2[0,0] = (1+tr)/4;
    b2[1,0] = (1+2*C[0,0]-tr)/4;
    b2[2,0] = (1+2*C[1,1]-tr)/4;
    b2[3,0] = (1+2*C[2,2]-tr)/4;

    case = np.argmax(b2);
    b = b2;
    if   case == 0:
            b[0,0] = math.sqrt(b2[0,0]);
            print b;
            b[1,0] = (C[1,2]-C[2,1])/4/b[0,0];
            b[2,0] = (C[2,0]-C[0,2])/4/b[0,0];
            b[3,0] = (C[0,1]-C[1,0])/4/b[0,0];
    elif case == 1:
            b[1,0] = math.sqrt(b2[1,0]);
            b[0,0] = (C[1,2]-C[2,1])/4/b[1,0];
            if b[0,0]<0:
                b[1,0] = -b[1,0];
                b[0,0] = -b[0,0];
            b[2,0] = (C[0,1]+C[1,0])/4/b[1,0];
            b[3,0] = (C[2,0]+C[0,2])/4/b[1,0];
    elif case == 2:
            b[2,0] = math.sqrt(b2[2,0]);
            b[0,0] = (C[2,0]-C[0,2])/4/b[2,0];
            if b[0,0]<0:
                b[2,0] = -b[2,0];
                b[0,0] = -b[0,0];
            b[1,0] = (C[0,1]+C[1,0])/4/b[2,0];
            b[3,0] = (C[1,2]+C[2,1])/4/b[2,0];
    elif case == 3:
            b[3,0] = math.sqrt(b2[3,0]);
            b[0,0] = (C[0,1]-C[1,0])/4/b[3,0];
            if b[0,0]<0:
                b[3,0] = -b[3,0];
                b[0,0] = -b[0,0];
            b[1,0] = (C[2,0]+C[0,2])/4/b[3,0];
            b[2,0] = (C[1,2]+C[2,1])/4/b[3,0];

    return b;

def C2Euler121(C):
    """
    C2Euler121

    	Q = C2Euler121(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (1-2-1) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(C[0,1],-C[0,2]);
    q[1,0] = math.acos(C[0,0]);
    q[2,0] = math.atan2(C[1,0],C[2,0]);
    return q;

def C2Euler123(C):
    """
    C2Euler123

    	Q = C2Euler123(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (1-2-3) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(-C[2,1],C[2,2]);
    q[1,0] = math.asin(C[2,0]);
    q[2,0] = math.atan2(-C[1,0],C[0,0]);
    return q;

def C2Euler131(C):
    """
    C2Euler131

    	Q = C2Euler131(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (1-3-1) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(C[0,2],C[0,1]);
    q[1,0] = math.acos(C[0,0]);
    q[2,0] = math.atan2(C[2,0],-C[1,0]);
    return q;

def C2Euler132(C):
    """
    C2Euler132

    	Q = C2Euler132(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (1-3-2) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(C[1,2],C[1,1]);
    q[1,0] = math.asin(-C[1,0]);
    q[2,0] = math.atan2(C[2,0],C[0,0]);
    return q;

def C2Euler212(C):
    """
    C2Euler212

    	Q = C2Euler212(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (2-1-2) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(C[1,0],C[1,2]);
    q[1,0] = math.acos(C[1,1]);
    q[2,0] = math.atan2(C[0,1],-C[2,1]);
    return q;

def C2Euler213(C):
    """
    C2Euler213

        Q = C2Euler213(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (2-1-3) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(C[2,0],C[2,2]);
    q[1,0] = math.asin(-C[2,1]);
    q[2,0] = math.atan2(C[0,1],C[1,1]);
    return q;

def C2Euler231(C):
    """
    C2Euler231

    	Q = C2Euler231(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (2-3-1) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(-C[0,2],C[0,0]);
    q[1,0] = math.asin(C[0,1]);
    q[2,0] = math.atan2(-C[2,1],C[1,1]);
    return q;

def C2Euler232(C):
    """
    C2Euler232

    	Q = C2Euler232(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (2-3-2) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(C[1,2],-C[1,0]);
    q[1,0] = math.acos(C[1,1]);
    q[2,0] = math.atan2(C[2,1],C[0,1]);
    return q;

def C2Euler312(C):
    """
    C2Euler312

    	Q = C2Euler312(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (3-1-2) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(-C[1,0],C[1,1]);
    q[1,0] = math.asin(C[1,2]);
    q[2,0] = math.atan2(-C[0,2],C[2,2]);
    return q;

def C2Euler313(C):
    """
    C2Euler313

    	Q = C2Euler313(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (3-1-3) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(C[2,0],-C[2,1]);
    q[1,0] = math.acos(C[2,2]);
    q[2,0] = math.atan2(C[0,2],C[1,2]);
    return q;

def C2Euler321(C):
    """
    C2Euler321

    	Q = C2Euler321(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (3-2-1) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(C[0,1],C[0,0]);
    q[1,0] = math.asin(-C[0,2]);
    q[2,0] = math.atan2(C[1,2],C[2,2]);
    return q;

def C2Euler323(C):
    """
    C2Euler323

    	Q = C2Euler323(C) translates the 3x3 direction cosine matrix
    	C into the corresponding (3-2-3) Euler angle set.
    """
    q = np.matrix('0.;0.;0.');
    q[0,0] = math.atan2(C[2,1],C[2,0]);
    q[1,0] = math.acos(C[2,2]);
    q[2,0] = math.atan2(C[1,2],-C[0,2]);
    return q;

def C2Gibbs(C):
    """
    C2Gibbs

    	Q = C2Gibbs(C) translates the 3x3 direction cosine matrix
    	C into the corresponding 3x1 Gibbs vector Q.
    """

    b = C2EP(C);

    q = np.matrix('0.;0.;0.');
    q[0,0] = b[1,0]/b[0,0];
    q[1,0] = b[2,0]/b[0,0];
    q[2,0] = b[3,0]/b[0,0];

    return q;

def C2MRP(C):
    """
    C2MRP

    	Q = C2MRP(C) translates the 3x3 direction cosine matrix
    	C into the corresponding 3x1 MRP vector Q where the
    	MRP vector is chosen such that |Q| <= 1.
    """

    b = C2EP(C);

    q = np.matrix('0.;0.;0.');
    q[0,0] = b[1,0]/(1+b[0,0]);
    q[1,0] = b[2,0]/(1+b[0,0]);
    q[2,0] = b[3,0]/(1+b[0,0]);

    return q;

def C2PRV(C):
    """
    C2PRV

    	Q = C2PRV(C) translates the 3x3 direction cosine matrix
    	C into the corresponding 3x1 principal rotation vector Q,
    	where the first component of Q is the principal rotation angle
    	phi (0<= phi <= Pi)
    """

    cp = (np.trace(C)-1)/2;
    p = math.acos(cp);
    sp = p/2/math.sin(p);
    q = np.matrix('0.;0.;0.');
    q[0,0] = (C[1,2]-C[2,1])*sp;
    q[1,0] = (C[2,0]-C[0,2])*sp;
    q[2,0] = (C[0,1]-C[1,0])*sp;

    return q;




def addEP(b1,b2):
    """
    addEP(B1,B2)

    	Q = addEP(B1,B2) provides the Euler parameter vector
    	which corresponds to performing to successive
    	rotations B1 and B2.
    """

    q = np.matrix('0.;0.;0.;0.');
    q[0,0] = b2[0,0]*b1[0,0]-b2[1,0]*b1[1,0]-b2[2,0]*b1[2,0]-b2[3,0]*b1[3,0];
    q[1,0] = b2[1,0]*b1[0,0]+b2[0,0]*b1[1,0]+b2[3,0]*b1[2,0]-b2[2,0]*b1[3,0];
    q[2,0] = b2[2,0]*b1[0,0]-b2[3,0]*b1[1,0]+b2[0,0]*b1[2,0]+b2[1,0]*b1[3,0];
    q[3,0] = b2[3,0]*b1[0,0]+b2[2,0]*b1[1,0]-b2[1,0]*b1[2,0]+b2[0,0]*b1[3,0];

    return q;

def addEuler121(e1,e2):
    """
    addEuler121(E1,E2)

    	Q = addEuler121(E1,E2) computes the overall (1-2-1) Euler
    	angle vector corresponding to two successive
    	(1-2-1) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1,0]);
    cp2 = math.cos(e2[1,0]);
    sp1 = math.sin(e1[1,0]);
    sp2 = math.sin(e2[1,0]);
    dum = e1[2,0]+e2[0,0];

    q = np.matrix('0.;0.;0.');
    q[1,0] = math.acos(cp1*cp2-sp1*sp2*math.cos(dum));
    cp3 = math.cos(q[1,0]);
    q[0,0] = Picheck(e1[0,0] + math.atan2(sp1*sp2*math.sin(dum),cp2-cp3*cp1));
    q[2,0] = Picheck(e2[2,0] + math.atan2(sp1*sp2*math.sin(dum),cp1-cp3*cp2));

    return q;

def addEuler123(e1,e2):
    """
    addEuler123(E1,E2)

    	Q = addEuler123(E1,E2) computes the overall (1-2-3) Euler
    	angle vector corresponding to two successive
    	(1-2-3) rotations E1 and E2.
    """

    C1 = Euler1232C(e1);
    C2 = Euler1232C(e2);
    C = C2*C1;
    return C2Euler123(C);

def addEuler131(e1,e2):
    """
    addEuler131(E1,E2)

    	Q = addEuler131(E1,E2) computes the overall (1-3-1) Euler
    	angle vector corresponding to two successive
    	(1-3-1) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1,0]);
    cp2 = math.cos(e2[1,0]);
    sp1 = math.sin(e1[1,0]);
    sp2 = math.sin(e2[1,0]);
    dum = e1[2,0]+e2[0,0];

    q = np.matrix('0.;0.;0.');
    q[1,0] = math.acos(cp1*cp2-sp1*sp2*math.cos(dum));
    cp3 = math.cos(q[1,0]);
    q[0,0] = Picheck(e1[0,0] + math.atan2(sp1*sp2*math.sin(dum),cp2-cp3*cp1));
    q[2,0] = Picheck(e2[2,0] + math.atan2(sp1*sp2*math.sin(dum),cp1-cp3*cp2));
    return q;

def addEuler132(e1,e2):
    """
    addEuler132(E1,E2)

    	Q = addEuler132(E1,E2) computes the overall (1-3-2) Euler
    	angle vector corresponding to two successive
    	(1-3-2) rotations E1 and E2.
    """

    C1 = Euler1322C(e1);
    C2 = Euler1322C(e2);
    C = C2*C1;
    return C2Euler132(C);

def addEuler212(e1,e2):
    """
    addEuler212(E1,E2)

    	Q = addEuler212(E1,E2) computes the overall (2-1-2) Euler
    	angle vector corresponding to two successive
    	(2-1-2) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1,0]);
    cp2 = math.cos(e2[1,0]);
    sp1 = math.sin(e1[1,0]);
    sp2 = math.sin(e2[1,0]);
    dum = e1[2,0]+e2[0,0];

    q = np.matrix('0.;0.;0.');
    q[1,0] = math.acos(cp1*cp2-sp1*sp2*math.cos(dum));
    cp3 = math.cos(q[1,0]);
    q[0,0] = Picheck(e1[0,0] + math.atan2(sp1*sp2*math.sin(dum),cp2-cp3*cp1));
    q[2,0] = Picheck(e2[2,0] + math.atan2(sp1*sp2*math.sin(dum),cp1-cp3*cp2));
    return q;

def addEuler213(e1,e2):
    """
    addEuler213(E1,E2)

    	Q = addEuler213(E1,E2) computes the overall (2-1-3) Euler
    	angle vector corresponding to two successive
    	(2-1-3) rotations E1 and E2.
    """

    C1 = Euler2132C(e1);
    C2 = Euler2132C(e2);
    C = C2*C1;
    return C2Euler213(C);

def addEuler231(e1,e2):
    """
    addEuler231(E1,E2)

    	Q = addEuler231(E1,E2) computes the overall (2-3-1) Euler
    	angle vector corresponding to two successive
    	(2-3-1) rotations E1 and E2.
    """

    C1 = Euler2312C(e1);
    C2 = Euler2312C(e2);
    C = C2*C1;
    return C2Euler231(C);

def addEuler232(e1,e2):
    """
    addEuler232(E1,E2)

    	Q = addEuler232(E1,E2) computes the overall (2-3-2) Euler
    	angle vector corresponding to two successive
    	(2-3-2) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1,0]);
    cp2 = math.cos(e2[1,0]);
    sp1 = math.sin(e1[1,0]);
    sp2 = math.sin(e2[1,0]);
    dum = e1[2,0]+e2[0,0];

    q = np.matrix('0.;0.;0.');
    q[1,0] = math.acos(cp1*cp2-sp1*sp2*math.cos(dum));
    cp3 = math.cos(q[1,0]);
    q[0,0] = Picheck(e1[0,0] + math.atan2(sp1*sp2*math.sin(dum),cp2-cp3*cp1));
    q[2,0] = Picheck(e2[2,0] + math.atan2(sp1*sp2*math.sin(dum),cp1-cp3*cp2));
    return q;

def addEuler312(e1,e2):
    """
    addEuler312(E1,E2)

    	Q = addEuler312(E1,E2) computes the overall (3-1-2) Euler
    	angle vector corresponding to two successive
    	(3-1-2) rotations E1 and E2.
    """

    C1 = Euler3122C(e1);
    C2 = Euler3122C(e2);
    C = C2*C1;
    return C2Euler312(C);

def addEuler313(e1,e2):
    """
    addEuler313(E1,E2)

    	Q = addEuler313(E1,E2) computes the overall (3-1-3) Euler
    	angle vector corresponding to two successive
    	(3-1-3) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1,0]);
    cp2 = math.cos(e2[1,0]);
    sp1 = math.sin(e1[1,0]);
    sp2 = math.sin(e2[1,0]);
    dum = e1[2,0]+e2[0,0];

    q = np.matrix('0.;0.;0.');
    q[1,0] = math.acos(cp1*cp2-sp1*sp2*math.cos(dum));
    cp3 = math.cos(q[1,0]);
    q[0,0] = Picheck(e1[0,0] + math.atan2(sp1*sp2*math.sin(dum),cp2-cp3*cp1));
    q[2,0] = Picheck(e2[2,0] + math.atan2(sp1*sp2*math.sin(dum),cp1-cp3*cp2));
    return q;

def addEuler321(e1,e2):
    """
    addEuler321(E1,E2)

    	Q = addEuler321(E1,E2) computes the overall (3-2-1) Euler
    	angle vector corresponding to two successive
    	(3-2-1) rotations E1 and E2.
    """

    C1 = Euler3212C(e1);
    C2 = Euler3212C(e2);
    C = C2*C1;
    return C2Euler321(C);

def addEuler323(e1,e2):
    """
    addEuler323(E1,E2)

    	Q = addEuler323(E1,E2) computes the overall (3-2-3) Euler
    	angle vector corresponding to two successive
    	(3-2-3) rotations E1 and E2.
    """

    cp1 = math.cos(e1[1,0]);
    cp2 = math.cos(e2[1,0]);
    sp1 = math.sin(e1[1,0]);
    sp2 = math.sin(e2[1,0]);
    dum = e1[2,0]+e2[0,0];

    q = np.matrix('0.;0.;0.');
    q[1,0] = math.acos(cp1*cp2-sp1*sp2*math.cos(dum));
    cp3 = math.cos(q[1,0]);
    q[0,0] = Picheck(e1[0,0] + math.atan2(sp1*sp2*math.sin(dum),cp2-cp3*cp1));
    q[2,0] = Picheck(e2[2,0] + math.atan2(sp1*sp2*math.sin(dum),cp1-cp3*cp2));
    return q;

def addGibbs(q1,q2):
    """
    addGibbs(Q1,Q2)

    	Q = addGibbs(Q1,Q2) provides the Gibbs vector
    	which corresponds to performing to successive
    	rotations Q1 and Q2.
    """

    return (q1+q2+np.cross(q1.T,q2.T).T)/(1-(q1.T*q2)[0,0]);

def addMRP(q1,q2):
    """
     addMRP(Q1,Q2)

    	Q = addMRP(Q1,Q2) provides the MRP vector
    	which corresponds to performing to successive
    	rotations Q1 and Q2.
    """

    q = (1-(q1.T*q1)[0,0])*q2+(1-(q2*q2.T)[0,0])*q1+2*np.cross(q1.T,q2.T).T;
    q = q/(1+q1.T*q1 * q2.T*q2-2*q1.T*q2);

    return q;

def PRV2elem(r):
    """
    PRV2elem(R)

    	Q = PRV2elem(R) translates a prinicpal rotation vector R
    	into the corresponding principal rotation element set Q.
    """
    q = np.matrix("0.;0.;0.;0.");
    q[0,0] = math.sqrt((r.T*r)[0,0]);
    q[1,0] = r[0,0]/q[0,0];
    q[2,0] = r[1,0]/q[0,0];
    q[3,0] = r[2,0]/q[0,0];
    return q;


def addPRV(qq1,qq2):
    """
     addPRV(Q1,Q2)

    	Q = addPRV(Q1,Q2) provides the principal rotation vector
    	which corresponds to performing to successive
    	prinicipal rotations Q1 and Q2.
    """

    q1 = PRV2elem(qq1);
    q2 = PRV2elem(qq2);
    cp1 = math.cos(q1[0,0]/2);
    cp2 = math.cos(q2[0,0]/2);
    sp1 = math.sin(q1[0,0]/2);
    sp2 = math.sin(q2[0,0]/2);
    e1 = q1[1:4,0];
    e2 = q2[1:4,0];

    p = 2*math.acos(cp1*cp2-sp1*sp2* (e1.T*e2)[0,0]);
    sp = math.sin(p/2);
    e = (cp1*sp2*e2+cp2*sp1*e1+sp1*sp2*np.cross(e1.T,e2.T).T)/sp;
    q = p*e;

    return q;

def BinvEP(q):
    """
    BinvEP(Q)

    	B = BinvEP(Q) returns the 3x4 matrix which relates
    	the derivative of Euler parameter vector Q to the
    	body angular velocity vector w.

    		w = 2 [B(Q)]^(-1) dQ/dt
    """
    B = np.matrix("0. 0. 0. 0.;0. 0. 0. 0; 0. 0. 0. 0.");
    B[0,0] = -q[1,0];
    B[0,1] =  q[0,0];
    B[0,2] =  q[3,0];
    B[0,3] = -q[2,0];
    B[1,0] = -q[2,0];
    B[1,1] = -q[3,0];
    B[1,2] =  q[0,0];
    B[1,3] =  q[1,0];
    B[2,0] = -q[3,0];
    B[2,1] =  q[2,0];
    B[2,2] = -q[1,0];
    B[2,3] =  q[0,0];

    return B;

def BinvEuler121(q):
    """
    BinvEuler121(Q)

    	B = BinvEuler121(Q) returns the 3x3 matrix which relates
    	the derivative of the (1-2-1) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = c2;
    B[0,1] = 0;
    B[0,2] = 1;
    B[1,0] = s2*s3;
    B[1,1] = c3;
    B[1,2] = 0;
    B[2,0] = s2*c3;
    B[2,1] = -s3;
    B[2,2] = 0;

    return B;

def BinvEuler123(q):
    """
    BinvEuler123(Q)

    	B = BinvEuler123(Q) returns the 3x3 matrix which relates
    	the derivative of the (1-2-3) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = c2*c3;
    B[0,1] = s3;
    B[0,2] = 0;
    B[1,0] = -c2*s3;
    B[1,1] = c3;
    B[1,2] = 0;
    B[2,0] = s2;
    B[2,1] = 0;
    B[2,2] = 1;

    return B;

def BinvEuler131(q):
    """
    BinvEuler131(Q)

    	B = BinvEuler131(Q) returns the 3x3 matrix which relates
    	the derivative of the (1-3-1) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = c2;
    B[0,1] = 0;
    B[0,2] = 1;
    B[1,0] = -s2*c3;
    B[1,1] = s3;
    B[1,2] = 0;
    B[2,0] = s2*s3;
    B[2,1] = c3;
    B[2,2] = 0;

    return B;

def BinvEuler132(q):
    """
    BinvEuler132(Q)

    	B = BinvEuler132(Q) returns the 3x3 matrix which relates
    	the derivative of the (1-3-2) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = c2*c3;
    B[0,1] = -s3;
    B[0,2] = 0;
    B[1,0] = -s2;
    B[1,1] = 0;
    B[1,2] = 1;
    B[2,0] = c2*s3;
    B[2,1] = c3;
    B[2,2] = 0;

    return B;

def BinvEuler212(q):
    """
    BinvEuler212(Q)

    	B = BinvEuler212(Q) returns the 3x3 matrix which relates
    	the derivative of the (2-1-2) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = s2*s3;
    B[0,1] = c3;
    B[0,2] = 0;
    B[1,0] = c2;
    B[1,1] = 0;
    B[1,2] = 1;
    B[2,0] = -s2*c3;
    B[2,1] = s3;
    B[2,2] = 0;

    return B;

def BinvEuler213(q):
    """
    BinvEuler213(Q)

    	B = BinvEuler213(Q) returns the 3x3 matrix which relates
    	the derivative of the (2-1-3) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = c2*s3;
    B[0,1] = c3;
    B[0,2] = 0;
    B[1,0] = c2*c3;
    B[1,1] = -s3;
    B[1,2] = 0;
    B[2,0] = -s2;
    B[2,1] = 0;
    B[2,2] = 1;

    return B;

def BinvEuler231(q):
    """
    BinvEuler231(Q)

    	B = BinvEuler231(Q) returns the 3x3 matrix which relates
    	the derivative of the (2-3-1) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = s2;
    B[0,1] = 0;
    B[0,2] = 1;
    B[1,0] = c2*c3;
    B[1,1] = s3;
    B[1,2] = 0;
    B[2,0] = -c2*s3;
    B[2,1] = c3;
    B[2,2] = 0;

    return B;

def BinvEuler232(q):
    """
    BinvEuler232(Q)

    	B = BinvEuler232(Q) returns the 3x3 matrix which relates
    	the derivative of the (2-3-2) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = s2*c3;
    B[0,1] = -s3;
    B[0,2] = 0;
    B[1,0] = c2;
    B[1,1] = 0;
    B[1,2] = 1;
    B[2,0] = s2*s3;
    B[2,1] = c3;
    B[2,2] = 0;

    return B;

def BinvEuler312(q):
    """
    BinvEuler312(Q)

    	B = BinvEuler312(Q) returns the 3x3 matrix which relates
    	the derivative of the (3-1-2) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = -c2*s3;
    B[0,1] = c3;
    B[0,2] = 0;
    B[1,0] = s2;
    B[1,1] = 0;
    B[1,2] = 1;
    B[2,0] = c2*c3;
    B[2,1] = s3;
    B[2,2] = 0;

    return B;

def BinvEuler313(q):
    """
    BinvEuler313(Q)

    	B = BinvEuler313(Q) returns the 3x3 matrix which relates
    	the derivative of the (3-1-3) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = s2*s3;
    B[0,1] = c3;
    B[0,2] = 0;
    B[1,0] = s2*c3;
    B[1,1] = -s3;
    B[1,2] = 0;
    B[2,0] = c2;
    B[2,1] = 0;
    B[2,2] = 1;

    return B;

def BinvEuler321(q):
    """
    BinvEuler321(Q)

    	B = BinvEuler321(Q) returns the 3x3 matrix which relates
    	the derivative of the (3-2-1) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = -s2;
    B[0,1] = 0;
    B[0,2] = 1;
    B[1,0] = c2*s3;
    B[1,1] = c3;
    B[1,2] = 0;
    B[2,0] = c2*c3;
    B[2,1] = -s3;
    B[2,2] = 0;

    return B;

def BinvEuler323(q):
    """
    BinvEuler323(Q)

    	B = BinvEuler323(Q) returns the 3x3 matrix which relates
    	the derivative of the (3-2-3) Euler angle vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """


    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = -s2*c3;
    B[0,1] = s3;
    B[0,2] = 0;
    B[1,0] = s2*s3;
    B[1,1] = c3;
    B[1,2] = 0;
    B[2,0] = c2;
    B[2,1] = 0;
    B[2,2] = 1;

    return B;

def BinvGibbs(q):
    """
    BinvGibbs(Q)

    	B = BinvGibbs(Q) returns the 3x3 matrix which relates
    	the derivative of Gibbs vector Q to the
    	body angular velocity vector w.

    		w = 2 [B(Q)]^(-1) dQ/dt
    """

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = 1;
    B[0,1] = q[2,0];
    B[0,2] = -q[1,0];
    B[1,0] = -q[2,0];
    B[1,1] = 1;
    B[1,2] = q[0,0];
    B[2,0] = q[1,0];
    B[2,1] = -q[0,0];
    B[2,2] = 1;
    B = B/(1+(q.T*q)[0,0]);

    return B;

def BinvMRP(q):
    """
    BinvMRP(Q)

    	B = BinvMRP(Q) returns the 3x3 matrix which relates
    	the derivative of MRP vector Q to the
    	body angular velocity vector w.

    		w = 4 [B(Q)]^(-1) dQ/dt
    """

    s2 = (q.T*q)[0,0];
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = 1-s2+2*q[0,0]*q[0,0];
    B[0,1] = 2*(q[0,0]*q[1,0]+q[2,0]);
    B[0,2] = 2*(q[0,0]*q[2,0]-q[1,0]);
    B[1,0] = 2*(q[1,0]*q[0,0]-q[2,0]);
    B[1,1] = 1-s2+2*q[1,0]*q[1,0];
    B[1,2] = 2*(q[1,0]*q[2,0]+q[0,0]);
    B[2,0] = 2*(q[2,0]*q[0,0]+q[1,0]);
    B[2,1] = 2*(q[2,0]*q[1,0]-q[0,0]);
    B[2,2] = 1-s2+2*q[2,0]*q[2,0];
    B = B/(1+s2)/(1+s2);

    return B;

def BinvPRV(q):
    """
    BinvPRV(Q)

    	B = BinvPRV(Q) returns the 3x3 matrix which relates
    	the derivative of principal rotation vector Q to the
    	body angular velocity vector w.

    		w = [B(Q)]^(-1) dQ/dt
    """

    p = math.sqrt(q.T*q);
    c1 = (1-math.cos(p))/p/p;
    c2 = (p-math.sin(p))/p/p/p;

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = 1-c2*(q[1,0]*q[1,0]+q[2,0]*q[2,0]);
    B[0,1] = c1*q[2,0] + c2*q[0,0]*q[1,0];
    B[0,2] = -c1*q[1,0] + c2*q[0,0]*q[2,0];
    B[1,0] = -c1*q[2,0] + c2*q[0,0]*q[1,0];
    B[1,1] = 1 - c2*(q[0,0]*q[0,0]+ q[2,0]*q[2,0]);
    B[1,2] = c1*q[0,0] + c2*q[1,0]*q[2,0];
    B[2,0] = c1*q[1,0] + c2*q[2,0]*q[0,0];
    B[2,1] = -c1*q[0,0] + c2*q[2,0]*q[1,0];
    B[2,2] = 1 - c2*(q[0,0]*q[0,0]+q[1,0]*q[1,0]);

    return B;

def BmatEP(q):
    """
    BmatEP(Q)

    	B = BmatEP(Q) returns the 4x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	Euler parameter vector Q.

    		dQ/dt = 1/2 [B(Q)] w
    """

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = -q[1,0];
    B[0,1] = -q[2,0];
    B[0,2] = -q[3,0];
    B[1,0] = q[0,0];
    B[1,1] = -q[3,0];
    B[1,2] = q[2,0];
    B[2,0] = q[3,0];
    B[2,1] = q[0,0];
    B[2,2] = -q[1,0];
    B[3,0] = -q[2,0];
    B[3,1] = q[1,0];
    B[3,2] = q[0,0];

    return B;

def BmatEuler121(q):
    """
    BmatEuler121(Q)

    	B = BmatEuler121(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(1-2-1) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = 0;
    B[0,1] = s3;
    B[0,2] = c3;
    B[1,0] = 0;
    B[1,1] = s2*c3;
    B[1,2] = -s2*s3;
    B[2,0] = s2;
    B[2,1] = -c2*s3;
    B[2,2] = -c2*c3;
    B = B/s2;

    return B;

def BmatEuler123(q):
    """
    BmatEuler123(Q)

    	B = BmatEuler123(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(1-2-3) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = c3;
    B[0,1] = -s3;
    B[0,2] = 0;
    B[1,0] = c2*s3;
    B[1,1] = c2*c3;
    B[1,2] = 0;
    B[2,0] = -s2*c3;
    B[2,1] = s2*s3;
    B[2,2] = c2;
    B = B/c2;

    return B;

def BmatEuler131(q):
    """
    BmatEuler131(Q)

    	B = BmatEuler131(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(1-3-1) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = 0;
    B[0,1] = -c3;
    B[0,2] = s3;
    B[1,0] = 0;
    B[1,1] = s2*s3;
    B[1,2] = s2*c3;
    B[2,0] = s2;
    B[2,1] = c2*c3;
    B[2,2] = -c2*s3;
    B = B/s2;

    return B;

def BmatEuler132(q):
    """
    BmatEuler132(Q)

    	B = BmatEuler132(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(1-3-2) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = c3;
    B[0,1] = 0;
    B[0,2] = s3;
    B[1,0] = -c2*s3;
    B[1,1] = 0;
    B[1,2] = c2*c3;
    B[2,0] = s2*c3;
    B[2,1] = c2;
    B[2,2] = s2*s3;
    B = B/c2;

    return B;

def BmatEuler212(q):
    """
    BmatEuler212(Q)

    	B = BmatEuler212(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(2-1-2) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = s3;
    B[0,1] = 0;
    B[0,2] = -c3;
    B[1,0] = s2*c3;
    B[1,1] = 0;
    B[1,2] = s2*s3;
    B[2,0] = -c2*s3;
    B[2,1] = s2;
    B[2,2] = c2*c3;
    B = B/s2;

    return B;

def BmatEuler213(q):
    """
    BmatEuler213(Q)

    	B = BmatEuler213(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(2-1-3) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = s3;
    B[0,1] = c3;
    B[0,2] = 0;
    B[1,0] = c2*c3;
    B[1,1] = -c2*s3;
    B[1,2] = 0;
    B[2,0] = s2*s3;
    B[2,1] = s2*c3;
    B[2,2] = c2;
    B = B/c2;

    return B;

def BmatEuler231(q):
    """
    BmatEuler231(Q)

    	B = BmatEuler231(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(2-3-1) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = 0;
    B[0,1] = c3;
    B[0,2] = -s3;
    B[1,0] = 0;
    B[1,1] = c2*s3;
    B[1,2] = c2*c3;
    B[2,0] = c2;
    B[2,1] = -s2*c3;
    B[2,2] = s2*s3;
    B = B/c2;

    return B;

def BmatEuler232(q):
    """
    BmatEuler232(Q)

    	B = BmatEuler232(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(2-3-2) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = c3;
    B[0,1] = 0;
    B[0,2] = s3;
    B[1,0] = -s2*s3;
    B[1,1] = 0;
    B[1,2] = s2*c3;
    B[2,0] = -c2*c3;
    B[2,1] = s2;
    B[2,2] = -c2*s3;
    B = B/s2;

    return B;

def BmatEuler312(q):
    """
    BmatEuler312(Q)

    	B = BmatEuler312(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(3-1-2) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = -s3;
    B[0,1] = 0;
    B[0,2] = c3;
    B[1,0] = c2*c3;
    B[1,1] = 0;
    B[1,2] = c2*s3;
    B[2,0] = s2*s3;
    B[2,1] = c2;
    B[2,2] = -s2*c3;
    B = B/c2;

    return B;

def BmatEuler313(q):
    """
    BmatEuler313(Q)

    	B = BmatEuler313(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(3-1-3) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = s3;
    B[0,1] = c3;
    B[0,2] = 0;
    B[1,0] = c3*s2;
    B[1,1] = -s3*s2;
    B[1,2] = 0;
    B[2,0] = -s3*c2;
    B[2,1] = -c3*c2;
    B[2,2] = s2;
    B = B/s2;

    return B;

def BmatEuler321(q):
    """
    BmatEuler321(Q)

    	B = BmatEuler321(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(3-2-1) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = 0;
    B[0,1] = s3;
    B[0,2] = c3;
    B[1,0] = 0;
    B[1,1] = c2*c3;
    B[1,2] = -c2*s3;
    B[2,0] = c2;
    B[2,1] = s2*s3;
    B[2,2] = s2*c3;
    B = B/c2;

    return B;

def BmatEuler323(q):
    """
    BmatEuler323(Q)

    	B = BmatEuler323(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	(3-2-3) Euler angle vector Q.

    		dQ/dt = [B(Q)] w
    """

    s2 = math.sin(q[1,0]);
    c2 = math.cos(q[1,0]);
    s3 = math.sin(q[2,0]);
    c3 = math.cos(q[2,0]);
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");

    B[0,0] = -c3;
    B[0,1] = s3;
    B[0,2] = 0;
    B[1,0] = s2*s3;
    B[1,1] = s2*c3;
    B[1,2] = 0;
    B[2,0] = c2*c3;
    B[2,1] = -c2*s3;
    B[2,2] = s2;
    B = B/s2;

    return B;

def BmatGibbs(q):
    """
    BmatGibbs(Q)

    	B = BmatGibbs(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	Gibbs vector Q.

    		dQ/dt = 1/2 [B(Q)] w
    """

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = 1+q[0,0]*q[0,0];
    B[0,1] = q[0,0]*q[1,0]-q[2,0];
    B[0,2] = q[0,0]*q[2,0]+q[1,0];
    B[1,0] = q[1,0]*q[0,0]+q[2,0];
    B[1,1] = 1+q[1,0]*q[1,0];
    B[1,2] = q[1,0]*q[2,0]-q[0,0];
    B[2,0] = q[2,0]*q[0,0]-q[1,0];
    B[2,1] = q[2,0]*q[1,0]+q[0,0];
    B[2,2] = 1+q[2,0]*q[2,0];

    return B;

def BmatMRP(q):
    """
    BmatMRP(Q)

    	B = BmatMRP(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	MRP vector Q.

    		dQ/dt = 1/4 [B(Q)] w
    """

    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    s2 = (q.T*q)[0,0];
    B[0,0] = 1-s2+2*q[0,0]*q[0,0];
    B[0,1] = 2*(q[0,0]*q[1,0]-q[2,0]);
    B[0,2] = 2*(q[0,0]*q[2,0]+q[1,0]);
    B[1,0] = 2*(q[1,0]*q[0,0]+q[2,0]);
    B[1,1] = 1-s2+2*q[1,0]*q[1,0];
    B[1,2] = 2*(q[1,0]*q[2,0]-q[0,0]);
    B[2,0] = 2*(q[2,0]*q[0,0]-q[1,0]);
    B[2,1] = 2*(q[2,0]*q[1,0]+q[0,0]);
    B[2,2] = 1-s2+2*q[2,0]*q[2,0];

    return B;

def BmatPRV(q):
    """
    BmatPRV(Q)

    	B = BmatPRV(Q) returns the 3x3 matrix which relates the
    	body angular velocity vector w to the derivative of
    	principal rotation vector Q.

    		dQ/dt = [B(Q)] w
    """

    p = math.sqrt(q.T*q);
    c = 1/p/p*(1-p/2/math.tan(p/2));
    B = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    B[0,0] = 1- c*(q[1,0]*q[1,0]+q[2,0]*q[2,0]);
    B[0,1] = -q[2,0]/2 + c*(q[0,0]*q[1,0]);
    B[0,2] = q[1,0]/2 + c*(q[0,0]*q[2,0]);
    B[1,0] = q[2,0]/2 + c*(q[0,0]*q[1,0]);
    B[1,1] = 1 - c*(q[0,0]*q[0,0]+q[2,0]*q[2,0]);
    B[1,2] = -q[0,0]/2 + c*(q[1,0]*q[2,0]);
    B[2,0] = -q[1,0]/2 + c*(q[0,0]*q[2,0]);
    B[2,1] = q[0,0]/2 + c*(q[1,0]*q[2,0]);
    B[2,2] = 1-c*(q[0,0]*q[0,0]+q[1,0]*q[1,0]);

    return B;

def dEP(q,w):
    """
    dEP(Q,W)

    	dq = dEP(Q,W) returns the Euler parameter derivative
    	for a given Euler parameter vector Q and body
    	angular velocity vector w.

    	dQ/dt = 1/2 [B(Q)] w
    """

    return .5*BmatEP(q)*w;

def dEuler121(q,w):
    """
    dEuler121(Q,W)

    	dq = dEuler121(Q,W) returns the (1-2-1) Euler angle derivative
    	vector for a given (1-2-1) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler121(q)*w;

def dEuler123(q,w):
    """
    dEuler123(Q,W)

    	dq = dEuler123(Q,W) returns the (1-2-3) Euler angle derivative
    	vector for a given (1-2-3) Euler angle vector Q and body
    	angular velocity vector w.

        dQ/dt =  [B(Q)] w
    """

    return BmatEuler123(q)*w;

def dEuler131(q,w):
    """
    dEuler131(Q,W)

    	dq = dEuler131(Q,W) returns the (1-3-1) Euler angle derivative
    	vector for a given (1-3-1) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler131(q)*w;

def dEuler132(q,w):
    """
    dEuler132(Q,W)

    	dq = dEuler132(Q,W) returns the (1-3-2) Euler angle derivative
    	vector for a given (1-3-2) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler132(q)*w;

def dEuler212(q,w):
    """
    dEuler212(Q,W)

    	dq = dEuler212(Q,W) returns the (2-1-2) Euler angle derivative
    	vector for a given (2-1-2) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler212(q)*w;

def dEuler213(q,w):
    """
    dEuler213(Q,W)

    	dq = dEuler213(Q,W) returns the (2-1-3) Euler angle derivative
    	vector for a given (2-1-3) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler213(q)*w;

def dEuler231(q,w):
    """
    dEuler231(Q,W)

    	dq = dEuler231(Q,W) returns the (2-3-1) Euler angle derivative
    	vector for a given (2-3-1) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler231(q)*w;

def dEuler232(q,w):
    """
    dEuler232(Q,W)

    	dq = dEuler232(Q,W) returns the (2-3-2) Euler angle derivative
    	vector for a given (2-3-2) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler232(q)*w;

def dEuler312(q,w):
    """
    dEuler312(Q,W)

    	dq = dEuler312(Q,W) returns the (3-1-2) Euler angle derivative
    	vector for a given (3-1-2) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler312(q)*w;

def dEuler313(q,w):
    """
    dEuler313(Q,W)

    	dq = dEuler313(Q,W) returns the (3-1-3) Euler angle derivative
    	vector for a given (3-1-3) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler313(q)*w;

def dEuler321(q,w):
    """
    dEuler321(Q,W)

    	dq = dEuler321(Q,W) returns the (3-2-1) Euler angle derivative
    	vector for a given (3-2-1) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler321(q)*w;

def dEuler323(q,w):
    """
    dEuler323(Q,W)

    	dq = dEuler323(Q,W) returns the (3-2-3) Euler angle derivative
    	vector for a given (3-2-3) Euler angle vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatEuler323(q)*w;

def dGibbs(q,w):
    """
    dGibbs(Q,W)

    	dq = dGibbs(Q,W) returns the Gibbs derivative
    	for a given Gibbs vector Q and body
    	angular velocity vector w.

    	dQ/dt = 1/2 [B(Q)] w
    """

    return .5*BmatGibbs(q)*w;

def dMRP(q,w):
    """
    dMRP(Q,W)

    	dq = dMRP(Q,W) returns the MRP derivative
    	for a given MRP vector Q and body
    	angular velocity vector w.

    	dQ/dt = 1/4 [B(Q)] w
    """

    return .25*BmatMRP(q)*w;

def dPRV(q,w):
    """
    dPRV(Q,W)

    	dq = dPRV(Q,W) returns the PRV derivative
    	for a given PRV vector Q and body
    	angular velocity vector w.

    	dQ/dt =  [B(Q)] w
    """

    return BmatPRV(q)*w;

def elem2PRV(r):
    """
    elem2PRV(R)

    	Q = elem2PRV(R) translates a prinicpal rotation
    	element set R into the corresponding principal
    	rotation vector Q.
    """

    q = np.matrix("0.;0.;0.");
    q[0,0] = r[1,0]*r[0,0];
    q[1,0] = r[2,0]*r[0,0];
    q[2,0] = r[3,0]*r[0,0];

    return q;

def Gibbs2C(q):
    """
    Gibbs2C

    	C = Gibbs2C(Q) returns the direction cosine
    	matrix in terms of the 3x1 Gibbs vector Q.
    """

    q1 = q[0,0];
    q2 = q[1,0];
    q3 = q[2,0];

    d1 = (q.T*q)[0,0];
    C = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    C[0,0] = 1+2*q1*q1-d1;
    C[0,1] = 2*(q1*q2+q3);
    C[0,2] = 2*(q1*q3-q2);
    C[1,0] = 2*(q2*q1-q3);
    C[1,1] = 1+2*q2*q2-d1;
    C[1,2] = 2*(q2*q3+q1);
    C[2,0] = 2*(q3*q1+q2);
    C[2,1] = 2*(q3*q2-q1);
    C[2,2] = 1+2*q3*q3-d1;
    C = C/(1+d1);

    return C;

def Gibbs2EP(q1):
    """
    Gibbs2EP(Q1)

    	Q = Gibbs2EP(Q1) translates the Gibbs vector Q1
    	into the Euler parameter vector Q.
    """

    q = np.matrix("0.;0.;0.;0.");
    q[0,0] = 1/math.sqrt(1+(q1.T*q1)[0,0]);
    q[1,0] = q1[0,0]*q[0,0];
    q[2,0] = q1[1,0]*q[0,0];
    q[3,0] = q1[2,0]*q[0,0];

    return q;

def Gibbs2Euler121(q):
    """
    Gibbs2Euler121(Q)

    	E = Gibbs2Euler121(Q) translates the Gibbs
        vector Q into the (1-2-1) Euler angle vector E.
    """

    return EP2Euler121(Gibbs2EP(q));

def Gibbs2Euler123(q):
    """
    Gibbs2Euler123(Q)

    	E = Gibbs2Euler123(Q) translates the Gibbs
    	 vector Q into the (1-2-3) Euler angle vector E.
    """

    return EP2Euler123(Gibbs2EP(q));

def Gibbs2Euler131(q):
    """
    Gibbs2Euler131(Q)

    	E = Gibbs2Euler131(Q) translates the Gibbs
    	 vector Q into the (1-3-1) Euler angle vector E.
    """

    return EP2Euler131(Gibbs2EP(q));

def Gibbs2Euler132(q):
    """
    Gibbs2Euler132(Q)

    	E = Gibbs2Euler132(Q) translates the Gibbs
        vector Q into the (1-3-2) Euler angle vector E.
    """

    return EP2Euler132(Gibbs2EP(q));

def Gibbs2Euler212(q):
    """
    Gibbs2Euler212(Q)

    	E = Gibbs2Euler212(Q) translates the Gibbs
    	 vector Q into the (2-1-2) Euler angle vector E.
    """

    return EP2Euler212(Gibbs2EP(q));

def Gibbs2Euler213(q):
    """
    Gibbs2Euler213(Q)

    	E = Gibbs2Euler213(Q) translates the Gibbs
    	 vector Q into the (2-1-3) Euler angle vector E.
    """

    return EP2Euler213(Gibbs2EP(q));

def Gibbs2Euler231(q):
    """
    Gibbs2Euler231(Q)

    	E = Gibbs2Euler231(Q) translates the Gibbs
    	 vector Q into the (2-3-1) Euler angle vector E.
    """

    return EP2Euler231(Gibbs2EP(q));

def Gibbs2Euler232(q):
    """
    Gibbs2Euler232(Q)

    	E = Gibbs2Euler232(Q) translates the Gibbs
    	 vector Q into the (2-3-2) Euler angle vector E.
    """

    return EP2Euler232(Gibbs2EP(q));

def Gibbs2Euler312(q):
    """
    Gibbs2Euler312(Q)

    	E = Gibbs2Euler312(Q) translates the Gibbs
    	 vector Q into the (3-1-2) Euler angle vector E.
    """

    return EP2Euler312(Gibbs2EP(q));

def Gibbs2Euler313(q):
    """
    Gibbs2Euler313(Q)

    	E = Gibbs2Euler313(Q) translates the Gibbs
    	 vector Q into the (3-1-3) Euler angle vector E.
    """

    return EP2Euler313(Gibbs2EP(q));

def Gibbs2Euler321(q):
    """
    Gibbs2Euler321(Q)

    	E = Gibbs2Euler321(Q) translates the Gibbs
    	 vector Q into the (3-2-1) Euler angle vector E.
    """

    return EP2Euler321(Gibbs2EP(q));

def Gibbs2Euler323(q):
    """
    Gibbs2Euler323(Q)

    	E = Gibbs2Euler323(Q) translates the Gibbs
    	 vector Q into the (3-2-3) Euler angle vector E.
    """

    return EP2Euler323(Gibbs2EP(q));

def Gibbs2MRP(q1):
    """
    Gibbs2MRP(Q1)

    	Q = Gibbs2MRP(Q1) translates the Gibbs vector Q1
    	into the MRP vector Q.
    """

    return q1/(1+math.sqrt(1+(q1.T*q1)[0,0]));

def Gibbs2PRV(q1):
    """
    Gibbs2PRV(Q1)

    	Q = Gibbs2PRV(Q1) translates the Gibbs vector Q1
    	into the principal rotation vector Q.
    """

    tp = math.sqrt(q1.T*q1);
    p = 2*math.atan(tp);
    q = np.matrix("0.;0.;0.");
    q[0,0] = q1[0,0]/tp*p;
    q[1,0] = q1[1,0]/tp*p;
    q[2,0] = q1[2,0]/tp*p;

    return q;


def MRP2C(q):
    """
    MRP2C

    	C = MRP2C(Q) returns the direction cosine
    	matrix in terms of the 3x1 MRP vector Q.
    """

    q1 = q[0,0];
    q2 = q[1,0];
    q3 = q[2,0];

    d1 = np.dot(q.T, q);
    S = 1-d1;
    d = (1+d1)*(1+d1);
    C = np.matrix("0. 0. 0.;0. 0. 0.;0. 0. 0.");
    C[0,0] = 4*(2*q1*q1-d1)+S*S;
    C[0,1] = 8*q1*q2+4*q3*S;
    C[0,2] = 8*q1*q3-4*q2*S;
    C[1,0] = 8*q2*q1-4*q3*S;
    C[1,1] = 4*(2*q2*q2-d1)+S*S;
    C[1,2] = 8*q2*q3+4*q1*S;
    C[2,0] = 8*q3*q1+4*q2*S;
    C[2,1] = 8*q3*q2-4*q1*S;
    C[2,2] = 4*(2*q3*q3-d1)+S*S;
    C = C/d;

    return C;

def MRP2EP(q1):
    """
    MRP2EP(Q1)

    	Q = MRP2EP(Q1) translates the MRP vector Q1
    	into the Euler parameter vector Q.
    """

    ps = 1+(q1.T*q1)[0,0];
    q = np.matrix("0.;0.;0.;0.");
    q[0,0] = (1-(q1.T*q1)[0,0])/ps;
    q[1,0] = 2*q1[0,0]/ps;
    q[2,0] = 2*q1[1,0]/ps;
    q[3,0] = 2*q1[2,0]/ps;

    return q;

def MRP2Euler121(q):
    """
    MRP2Euler121(Q)

    	E = MRP2Euler121(Q) translates the MRP
    	 vector Q into the (1-2-1) Euler angle vector E.
    """

    return EP2Euler121(MRP2EP(q));

def MRP2Euler123(q):
    """
    MRP2Euler123(Q)

    	E = MRP2Euler123(Q) translates the MRP
    	 vector Q into the (1-2-3) Euler angle vector E.
    """

    return EP2Euler123(MRP2EP(q));

def MRP2Euler131(q):
    """
    MRP2Euler131(Q)

    	E = MRP2Euler131(Q) translates the MRP
    	 vector Q into the (1-3-1) Euler angle vector E.
    """

    return EP2Euler131(MRP2EP(q));

def MRP2Euler132(q):
    """
    MRP2Euler132(Q)

    	E = MRP2Euler132(Q) translates the MRP
    	 vector Q into the (1-3-2) Euler angle vector E.
    """

    return EP2Euler132(MRP2EP(q));

def MRP2Euler212(q):
    """
    MRP2Euler212(Q)

    	E = MRP2Euler212(Q) translates the MRP
    	 vector Q into the (2-1-2) Euler angle vector E.
    """

    return EP2Euler212(MRP2EP(q));

def MRP2Euler213(q):
    """
    MRP2Euler213(Q)

    	E = MRP2Euler213(Q) translates the MRP
    	 vector Q into the (2-1-3) Euler angle vector E.
    """

    return EP2Euler213(MRP2EP(q));

def MRP2Euler231(q):
    """
    MRP2Euler231(Q)

    	E = MRP2Euler231(Q) translates the MRP
    	 vector Q into the (2-3-1) Euler angle vector E.
    """

    return EP2Euler231(MRP2EP(q));

def MRP2Euler232(q):
    """
    MRP2Euler232(Q)

       E = MRP2Euler232(Q) translates the MRP
    	 vector Q into the (2-3-2) Euler angle vector E.
    """

    return EP2Euler232(MRP2EP(q));

def MRP2Euler312(q):
    """
    MRP2Euler312(Q)

    	E = MRP2Euler312(Q) translates the MRP
    	 vector Q into the (3-1-2) Euler angle vector E.
    """

    return EP2Euler312(MRP2EP(q));

def MRP2Euler313(q):
    """
    MRP2Euler313(Q)

    	E = MRP2Euler313(Q) translates the MRP
    	 vector Q into the (3-1-3) Euler angle vector E.
    """

    return EP2Euler313(MRP2EP(q));

def MRP2Euler321(q):
    """
    MRP2Euler321(Q)

    	E = MRP2Euler321(Q) translates the MRP
    	 vector Q into the (3-2-1) Euler angle vector E.
    """

    return EP2Euler321(MRP2EP(q));

def MRP2Euler323(q):
    """
    MRP2Euler323(Q)

    	E = MRP2Euler323(Q) translates the MRP
    	 vector Q into the (3-2-3) Euler angle vector E.
    """

    return EP2Euler323(MRP2EP(q));

def MRP2Gibbs(q1):
    """
    MRP2Gibbs(Q1)

    	Q = MRP2Gibbs(Q1) translates the MRP vector Q1
    	into the Gibbs vector Q.
    """

    return 2*q1/(1-(q1.T*q1)[0,0]);

def MRP2PRV(q1):
    """
    MRP2PRV(Q1)

    	Q = MRP2PRV(Q1) translates the MRP vector Q1
    	into the principal rotation vector Q.
    """

    tp = math.sqrt(q1.T*q1);
    p = 4*math.atan(tp);
    q = np.matrix("0.;0.;0.");
    q[0,0] = q1[0,0]/tp*p;
    q[1,0] = q1[1,0]/tp*p;
    q[2,0] = q1[2,0]/tp*p;

    return q;

def MRPswitch(q,s2):
    """
    MRPswitch

    	S = MRPswitch(Q,s2) checks to see if norm(Q) is larger than s2.
    	If yes, then the MRP vector Q is mapped to its shadow set.
    """

    q2 = (q.T*q)[0,0];
    if (q2>s2*s2):
        s = -q/q2;
    else:
        s = q;

    return s;

def PRV2C(q):
    """
    PRV2C

    	C = PRV2C(Q) returns the direction cosine
    	matrix in terms of the 3x1 principal rotation vector
    	Q.
    """

    q0 = math.sqrt(q.T*q);
    q1 = q[0,0]/q0;
    q2 = q[1,0]/q0;
    q3 = q[2,0]/q0;

    cp= math.cos(q0);
    sp= math.sin(q0);
    d1 = 1-cp;
    C = np.matrix("0.,0.,0.;0.,0.,0.;0.,0.,0.");
    C[0,0] = q1*q1*d1+cp;
    C[0,1] = q1*q2*d1+q3*sp;
    C[0,2] = q1*q3*d1-q2*sp;
    C[1,0] = q2*q1*d1-q3*sp;
    C[1,1] = q2*q2*d1+cp;
    C[1,2] = q2*q3*d1+q1*sp;
    C[2,0] = q3*q1*d1+q2*sp;
    C[2,1] = q3*q2*d1-q1*sp;
    C[2,2] = q3*q3*d1+cp;

    return C;

def PRV2EP(qq1):
    """"
    PRV2EP(Q1)

    	Q = PRV2EP(Q1) translates the principal rotation vector Q1
    	into the Euler parameter vector Q.
    """

    q = np.matrix("0.;0.;0.;0.");
    q1 = PRV2elem(qq1);
    sp = math.sin(q1[0,0]/2);
    q[0,0] = math.cos(q1[0,0]/2);
    q[1,0] = q1[1,0]*sp;
    q[2,0] = q1[2,0]*sp;
    q[3,0] = q1[3,0]*sp;

    return q;

def PRV2Euler121(q):
    """
    PRV2Euler121(Q)

    	E = PRV2Euler121(Q) translates the principal rotation
    	vector Q into the (1-2-1) Euler angle vector E.
    """

    return EP2Euler121(PRV2EP(q));

def PRV2Euler123(q):
    """
    PRV2Euler123(Q)

    	E = PRV2Euler123(Q) translates the principal rotation
    	vector Q into the (1-2-3) Euler angle vector E.
    """

    return EP2Euler123(PRV2EP(q));

def PRV2Euler131(q):
    """
    PRV2Euler131(Q)

    	E = PRV2Euler131(Q) translates the principal rotation
    	vector Q into the (1-3-1) Euler angle vector E.
    """

    return EP2Euler131(PRV2EP(q));

def PRV2Euler132(q):
    """
    PRV2Euler132(Q)

    	E = PRV2Euler132(Q) translates the principal rotation
    	vector Q into the (1-3-2) Euler angle vector E.
    """

    return EP2Euler132(PRV2EP(q));

def PRV2Euler212(q):
    """
    PRV2Euler212(Q)

    	E = PRV2Euler212(Q) translates the principal rotation
    	vector Q into the (2-1-2) Euler angle vector E.
    """

    return EP2Euler212(PRV2EP(q));

def PRV2Euler213(q):
    """
    PRV2Euler213(Q)

    	E = PRV2Euler213(Q) translates the principal rotation
    	vector Q into the (2-1-3) Euler angle vector E.
    """

    return EP2Euler213(PRV2EP(q));

def PRV2Euler231(q):
    """
    PRV2Euler231(Q)

    	E = PRV2Euler231(Q) translates the principal rotation
    	vector Q into the (2-3-1) Euler angle vector E.
    """

    return EP2Euler231(PRV2EP(q));

def PRV2Euler232(q):
    """
    PRV2Euler232(Q)

    	E = PRV2Euler232(Q) translates the principal rotation
    	vector Q into the (2-3-2) Euler angle vector E.
    """

    return EP2Euler232(PRV2EP(q));

def PRV2Euler312(q):
    """
    PRV2Euler312(Q)

    	E = PRV2Euler312(Q) translates the principal rotation
    	vector Q into the (3-1-2) Euler angle vector E.
    """

    return EP2Euler312(PRV2EP(q));

def PRV2Euler313(q):
    """
    PRV2Euler313(Q)

    	E = PRV2Euler313(Q) translates the principal rotation
    	vector Q into the (3-1-3) Euler angle vector E.
    """

    return EP2Euler313(PRV2EP(q));

def PRV2Euler321(q):
    """
    PRV2Euler321(Q)

    	E = PRV2Euler321(Q) translates the principal rotation
    	vector Q into the (3-2-1) Euler angle vector E.
    """

    return EP2Euler321(PRV2EP(q));

def PRV2Euler323(q):
    """
    PRV2Euler323(Q)

    	E = PRV2Euler323(Q) translates the principal rotation
    	vector Q into the (3-2-3) Euler angle vector E.
    """

    return EP2Euler323(PRV2EP(q));

def PRV2Gibbs(q1):
    """
    PRV2Gibbs(Q1)

    	Q = PRV2Gibbs(Q1) translates the principal rotation vector Q1
    	into the Gibbs vector Q.
    """

    q1 = PRV2elem(q1);
    tp = math.tan(q1[0,0]/2);
    q = np.matrix("0.;0.;0.");
    q[0,0] = q1[1,0]*tp;
    q[1,0] = q1[2,0]*tp;
    q[2,0] = q1[3,0]*tp;

    return q;

def PRV2MRP(q1):
    """
     PRV2MRP(Q1)

    	Q = PRV2MRP(Q1) translates the principal rotation vector Q1
    	into the MRP vector Q.
    """

    q1 = PRV2elem(q1);
    tp = math.tan(q1[0,0]/4);
    q = np.matrix("0.;0.;0.");
    q[0,0] = q1[1,0]*tp;
    q[1,0] = q1[2,0]*tp;
    q[2,0] = q1[3,0]*tp;

    return q;

def subEP(b1,b2):
    """
    subEP(B1,B2)

    	Q = subEP(B1,B2) provides the Euler parameter vector
    	which corresponds to relative rotation from B2
    	to B1.
    """

    q = np.matrix("0.;0.;0.;0.");
    q[0,0] =  b2[0,0]*b1[0,0]+b2[1,0]*b1[1,0]+b2[2,0]*b1[2,0]+b2[3,0]*b1[3,0];
    q[1,0] = -b2[1,0]*b1[0,0]+b2[0,0]*b1[1,0]+b2[3,0]*b1[2,0]-b2[2,0]*b1[3,0];
    q[2,0] = -b2[2,0]*b1[0,0]-b2[3,0]*b1[1,0]+b2[0,0]*b1[2,0]+b2[1,0]*b1[3,0];
    q[3,0] = -b2[3,0]*b1[0,0]+b2[2,0]*b1[1,0]-b2[1,0]*b1[2,0]+b2[0,0]*b1[3,0];

    return q;

def subEuler121(e,e1):
    """
    subEuler121(E,E1)

    	E2 = subEuler121(E,E1) computes the relative
    	(1-2-1) Euler angle vector from E1 to E.
    """

    cp = math.cos(e[1,0]);
    cp1 = math.cos(e1[1,0]);
    sp = math.sin(e[1,0]);
    sp1 = math.sin(e1[1,0]);
    dum = e[0,0]-e1[0,0];

    e2 = np.matrix("0.;0.;0.");
    e2[1,0] = math.acos(cp1*cp+sp1*sp*math.cos(dum));
    cp2 = math.cos(e2[1,0]);
    e2[0,0] = Picheck(-e1[2,0] + math.atan2(sp1*sp*math.sin(dum),cp2*cp1-cp));
    e2[2,0] = Picheck(e[2,0] - math.atan2(sp1*sp*math.sin(dum),cp1-cp*cp2));

    return e2;

def subEuler123(e,e1):
    """
    subEuler123(E,E1)

    	E2 = subEuler123(E,E1) computes the relative
    	(1-2-3) Euler angle vector from E1 to E.
    """

    C = Euler1232C(e);
    C1 = Euler1232C(e1);
    C2 = C*C1.T;
    e2 = C2Euler123(C2);

    return e2;

def subEuler131(e,e1):
    """
    subEuler131(E,E1)

    	E2 = subEuler131(E,E1) computes the relative
    	(1-3-1) Euler angle vector from E1 to E.
    """

    cp = math.cos(e[1,0]);
    cp1 = math.cos(e1[1,0]);
    sp = math.sin(e[1,0]);
    sp1 = math.sin(e1[1,0]);
    dum = e[0,0]-e1[0,0];

    e2 = np.matrix("0.;0.;0.");
    e2[1,0] = math.acos(cp1*cp+sp1*sp*math.cos(dum));
    cp2 = math.cos(e2[1,0]);
    e2[0,0] = Picheck(-e1[2,0] + math.atan2(sp1*sp*math.sin(dum),cp2*cp1-cp));
    e2[2,0] = Picheck(e[2,0] - math.atan2(sp1*sp*math.sin(dum),cp1-cp*cp2));

    return e2;

def subEuler132(e,e1):
    """
    subEuler132(E,E1)

    	E2 = subEuler132(E,E1) computes the relative
    	(1-3-2) Euler angle vector from E1 to E.
    """

    C = Euler1322C(e);
    C1 = Euler1322C(e1);
    C2 = C*C1.T;
    e2 = C2Euler132(C2);

    return e2;

def subEuler212(e,e1):
    """
    subEuler212(E,E1)

    	E2 = subEuler212(E,E1) computes the relative
    	(2-1-2) Euler angle vector from E1 to E.
    """

    cp = math.cos(e[1,0]);
    cp1 = math.cos(e1[1,0]);
    sp = math.sin(e[1,0]);
    sp1 = math.sin(e1[1,0]);
    dum = e[0,0]-e1[0,0];

    e2 = np.matrix("0.;0.;0.");
    e2[1,0] = math.acos(cp1*cp+sp1*sp*math.cos(dum));
    cp2 = math.cos(e2[1,0]);
    e2[0,0] = Picheck(-e1[2,0] + math.atan2(sp1*sp*math.sin(dum),cp2*cp1-cp));
    e2[2,0] = Picheck(e[2,0] - math.atan2(sp1*sp*math.sin(dum),cp1-cp*cp2));

    return e2;

def subEuler213(e,e1):
    """
    subEuler213(E,E1)

    	E2 = subEuler213(E,E1) computes the relative
    	(2-1-3) Euler angle vector from E1 to E.
    """

    C = Euler2132C(e);
    C1 = Euler2132C(e1);
    C2 = C*C1.T;
    e2 = C2Euler213(C2);

    return e2;

def subEuler231(e,e1):
    """
    subEuler231(E,E1)

    	E2 = subEuler231(E,E1) computes the relative
    	(2-3-1) Euler angle vector from E1 to E.
    """

    C = Euler2312C(e);
    C1 = Euler2312C(e1);
    C2 = C*C1.T;
    e2 = C2Euler231(C2);

    return e2;

def subEuler232(e,e1):
    """
    subEuler232(E,E1)

    	E2 = subEuler232(E,E1) computes the relative
    	(2-3-2) Euler angle vector from E1 to E.
    """

    cp = math.cos(e[1,0]);
    cp1 = math.cos(e1[1,0]);
    sp = math.sin(e[1,0]);
    sp1 = math.sin(e1[1,0]);
    dum = e[0,0]-e1[0,0];

    e2 = np.matrix("0.;0.;0.");
    e2[1,0] = math.acos(cp1*cp+sp1*sp*math.cos(dum));
    cp2 = math.cos(e2[1,0]);
    e2[0,0] = Picheck(-e1[2,0] + math.atan2(sp1*sp*math.sin(dum),cp2*cp1-cp));
    e2[2,0] = Picheck(e[2,0] - math.atan2(sp1*sp*math.sin(dum),cp1-cp*cp2));

    return e2;

def subEuler312(e,e1):
    """
    subEuler312(E,E1)

    	E2 = subEuler312(E,E1) computes the relative
    	(3-1-2) Euler angle vector from E1 to E.
    """

    C = Euler3122C(e);
    C1 = Euler3122C(e1);
    C2 = C*C1.T;
    e2 = C2Euler312(C2);

    return e2;

def subEuler313(e,e1):
    """
    subEuler313(E,E1)

    	E2 = subEuler313(E,E1) computes the relative
    	(3-1-3) Euler angle vector from E1 to E.
    """

    cp = math.cos(e[1,0]);
    cp1 = math.cos(e1[1,0]);
    sp = math.sin(e[1,0]);
    sp1 = math.sin(e1[1,0]);
    dum = e[0,0]-e1[0,0];

    e2 = np.matrix("0.;0.;0.");
    e2[1,0] = math.acos(cp1*cp+sp1*sp*math.cos(dum));
    cp2 = math.cos(e2[1,0]);
    e2[0,0] = Picheck(-e1[2,0] + math.atan2(sp1*sp*math.sin(dum),cp2*cp1-cp));
    e2[2,0] = Picheck(e[2,0] - math.atan2(sp1*sp*math.sin(dum),cp1-cp*cp2));

    return e2;

def subEuler321(e,e1):
    """
    subEuler321(E,E1)

    	E2 = subEuler321(E,E1) computes the relative
    	(3-2-1) Euler angle vector from E1 to E.
    """

    C = Euler3212C(e);
    C1 = Euler3212C(e1);
    C2 = C*C1.T;
    e2 = C2Euler321(C2);

    return e2;

def subEuler323(e,e1):
    """
    subEuler323(E,E1)

    	E2 = subEuler323(E,E1) computes the relative
    	(3-2-3) Euler angle vector from E1 to E.
    """

    cp = math.cos(e[1,0]);
    cp1 = math.cos(e1[1,0]);
    sp = math.sin(e[1,0]);
    sp1 = math.sin(e1[1,0]);
    dum = e[0,0]-e1[0,0];

    e2 = np.matrix("0.;0.;0.");
    e2[1,0] = math.acos(cp1*cp+sp1*sp*math.cos(dum));
    cp2 = math.cos(e2[1,0]);
    e2[0,0] = Picheck(-e1[2,0] + math.atan2(sp1*sp*math.sin(dum),cp2*cp1-cp));
    e2[2,0] = Picheck(e[2,0] - math.atan2(sp1*sp*math.sin(dum),cp1-cp*cp2));

    return e2;

def subGibbs(q1,q2):
    """
    subGibbs(Q1,Q2)

    	Q = subGibbs(Q1,Q2) provides the Gibbs vector
    	which corresponds to relative rotation from Q2
    	to Q1.
    """

    return (q1-q2+np.cross(q1.T,q2.T).T)/(1+(q1.T*q2)[0,0]);

def subMRP(q1,q2):
    """
    subMRP(Q1,Q2)

    	Q = subMRP(Q1,Q2) provides the MRP vector
    	which corresponds to relative rotation from Q2
    	to Q1.
    """

    q = (1-(q2.T*q2)[0,0])*q1-(1-(q1.T*q1)[0,0])*q2+2*np.cross(q1.T,q2.T).T;
    q = q/(1+ (q1.T*q1)[0,0] * (q2.T*q2)[0,0]+2*(q1.T*q2)[0,0]);

    return q;

def subPRV(q1,q2):
    """
    subPRV(Q1,Q2)

    	Q = subPRV(Q1,Q2) provides the prinipal rotation vector
    	which corresponds to relative principal rotation from Q2
    	to Q1.
    """

    q1 = PRV2elem(q1);
    q2 = PRV2elem(q2);
    cp1 = math.cos(q1[0,0]/2);
    cp2 = math.cos(q2[0,0]/2);
    sp1 = math.sin(q1[0,0]/2);
    sp2 = math.sin(q2[0,0]/2);
    e1 = q1[1:4,0];
    e2 = q2[1:4,0];

    p = 2*math.acos(cp1*cp2+sp1*sp2*(e1.T*e2)[0,0]);
    sp = math.sin(p/2);
    e = (-cp1*sp2*e2+cp2*sp1*e1+sp1*sp2*np.cross(e1.T,e2.T).T)/sp;
    q = p*e;

    return q;


def EP2C(q):
	"""
	EP2C	
	
        C = EP2C(Q) returns the direction math.cosine
        matrix in terms of the 4x1 Euler parameter vector
        Q.  The first element is the non-dimensional Euler
        parameter, while the remain three elements form 
        the Eulerparameter vector.
	"""
	
	q0 = q[0,0];
	q1 = q[1,0];
	q2 = q[2,0];
	q3 = q[3,0];
	
	C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
	C[0,0] = q0*q0+q1*q1-q2*q2-q3*q3;
	C[0,1] = 2*(q1*q2+q0*q3);
	C[0,2] = 2*(q1*q3-q0*q2);
	C[1,0] = 2*(q1*q2-q0*q3);
	C[1,1] = q0*q0-q1*q1+q2*q2-q3*q3;
	C[1,2] = 2*(q2*q3+q0*q1);
	C[2,0] = 2*(q1*q3 + q0*q2);
	C[2,1] = 2*(q2*q3-q0*q1);
	C[2,2] = q0*q0-q1*q1-q2*q2+q3*q3;
	
	return C;


def EP2Euler121(q):
	"""
	EP2Euler121(Q)

        E = EP2Euler121(Q) translates the Euler parameter
        vector Q into the corresponding (1-2-1) Euler angle
        vector E.
	"""
	
	t1 = math.atan2(q[3,0],q[2,0]);
	t2 = math.atan2(q[1,0],q[0,0]);
	
	e1 = t1+t2;
	e2 = 2*math.acos(math.sqrt(q[0,0]*q[0,0]+q[1,0]*q[1,0]));
	e3 = t2-t1;
	
	return np.matrix([[e1],[e2],[e3]]);

def EP2Euler123(q):
	"""
	EP2Euler123

        Q = EP2Euler123(Q) translates the Euler parameter vector
        Q into the corresponding (1-2-3) Euler angle set.
	"""	
	
	q0 = q[0,0];
	q1 = q[1,0];
	q2 = q[2,0];
	q3 = q[3,0];
	
	e1 = math.atan2(-2*(q2*q3-q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
	e2 = math.asin(2*(q1*q3 + q0*q2));
	e3 = math.atan2(-2*(q1*q2-q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
	
	return np.matrix([[e1],[e2],[e3]]);

def EP2Euler131(q):
	"""
	EP2Euler131(Q)

        E = EP2Euler131(Q) translates the Euler parameter
        vector Q into the corresponding (1-3-1) Euler angle
        vector E.
	"""
	
	t1 = math.atan2(q[2,0],q[3,0]);
	t2 = math.atan2(q[1,0],q[0,0]);
	
	e1 = t2-t1;
	e2 = 2*math.acos(math.sqrt(q[0,0]*q[0,0]+q[1,0]*q[1,0]));
	e3 = t2+t1;
	
	return np.matrix([[e1],[e2],[e3]]);

def EP2Euler132(q):
    """
    EP2Euler132

    	E = EP2Euler132(Q) translates the Euler parameter vector
    	Q into the corresponding (1-3-2) Euler angle set.

    """
    q0 = q[0,0];
    q1 = q[1,0];
    q2 = q[2,0];
    q3 = q[3,0];

    e1 = math.atan2(2*(q2*q3+q0*q1),q0*q0-q1*q1+q2*q2-q3*q3);
    e2 = math.asin(-2*(q1*q2-q0*q3));
    e3 = math.atan2(2*(q1*q3 + q0*q2),q0*q0+q1*q1-q2*q2-q3*q3);

    return np.matrix([[e1],[e2],[e3]]);

def EP2Euler212(q):
    """
    EP2Euler212(Q)

        E = EP2Euler212(Q) translates the Euler parameter
        vector Q into the corresponding (2-1-2) Euler angle
        vector E.
    """

    t1 = math.atan2(q[3,0],q[1,0]);
    t2 = math.atan2(q[2,0],q[0,0]);

    e1 = t2-t1;
    e2 = 2*math.acos(math.sqrt(q[0,0]*q[0,0]+q[2,0]*q[2,0]));
    e3 = t2+t1;

    return np.matrix([[e1],[e2],[e3]]);

def EP2Euler213(q):
    """
    EP2Euler213

    	Q = EP2Euler213(Q) translates the Euler parameter vector
    	Q into the corresponding (2-1-3) Euler angle set.
    """

    q0 = q[0,0];
    q1 = q[1,0];
    q2 = q[2,0];
    q3 = q[3,0];

    e1 = math.atan2(2*(q1*q3 + q0*q2),q0*q0-q1*q1-q2*q2+q3*q3);
    e2 = math.asin(-2*(q2*q3-q0*q1));
    e3 = math.atan2(2*(q1*q2+q0*q3),q0*q0-q1*q1+q2*q2-q3*q3);

    return np.matrix([[e1],[e2],[e3]]);

def EP2Euler231(q):
    """
    EP2Euler231

    	E = EP2Euler231(Q) translates the Euler parameter vector
    	Q into the corresponding (2-3-1) Euler angle set.
    """

    q0 = q[0,0];
    q1 = q[1,0];
    q2 = q[2,0];
    q3 = q[3,0];

    e1 = math.atan2(-2*(q1*q3-q0*q2), q0*q0+q1*q1-q2*q2-q3*q3);
    e2 = math.asin(2*(q1*q2+q0*q3));
    e3 = math.atan2(-2*(q2*q3-q0*q1),q0*q0-q1*q1+q2*q2-q3*q3);

    return np.matrix([[e1],[e2],[e3]]);

def EP2Euler232(q):
    """
    EP2Euler232(Q)

    	E = EP2Euler232(Q) translates the Euler parameter
    	vector Q into the corresponding (2-3-2) Euler angle
    	vector E.
    """

    t1 = math.atan2(q[1,0],q[3,0]);
    t2 = math.atan2(q[2,0],q[0,0]);

    e1 = t1+t2;
    e2 = 2*math.acos(math.sqrt(q[0,0]*q[0,0]+q[2,0]*q[2,0]));
    e3 = t2-t1;

    return np.matrix([[e1],[e2],[e3]]);

def EP2Euler312(q):
    """
    EP2Euler312

    	E = EP2Euler312(Q) translates the Euler parameter vector
    	Q into the corresponding (3-1-2) Euler angle set.
    """

    q0 = q[0,0];
    q1 = q[1,0];
    q2 = q[2,0];
    q3 = q[3,0];

    e1 = math.atan2(-2*(q1*q2-q0*q3),q0*q0-q1*q1+q2*q2-q3*q3);
    e2 = math.asin(2*(q2*q3+q0*q1));
    e3 = math.atan2(-2*(q1*q3-q0*q2),q0*q0-q1*q1-q2*q2+q3*q3);

    return np.matrix([[e1],[e2],[e3]]);

def EP2Euler313(q):
    """
    EP2Euler313(Q)

    	E = EP2Euler313(Q) translates the Euler parameter
    	vector Q into the corresponding (3-1-3) Euler angle
    	vector E.
    """

    t1 = math.atan2(q[2,0],q[1,0]);
    t2 = math.atan2(q[3,0],q[0,0]);

    e1 = t1+t2;
    e2 = 2*math.acos(math.sqrt(q[0,0]*q[0,0]+q[3,0]*q[3,0]));
    e3 = t2-t1;

    return np.matrix([[e1],[e2],[e3]]);

def EP2Euler321(q):
    """
    EP2Euler321

    	E = EP2Euler321(Q) translates the Euler parameter vector
    	Q into the corresponding (3-2-1) Euler angle set.
    """

    q0 = q[0,0];
    q1 = q[1,0];
    q2 = q[2,0];
    q3 = q[3,0];

    e1 = math.atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
    e2 = math.asin(-2*(q1*q3-q0*q2));
    e3 = math.atan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);

    return np.matrix([[e1],[e2],[e3]]);

def EP2Euler323(q):
    """
    EP2Euler323(Q)

    	E = EP2Euler323(Q) translates the Euler parameter
    	vector Q into the corresponding (3-2-3) Euler angle
    	vector E.
    """

    t1 = math.atan2(q[1,0],q[2,0]);
    t2 = math.atan2(q[3,0],q[0,0]);

    e1 = t2-t1;
    e2 = 2*math.acos(math.sqrt(q[0,0]*q[0,0]+q[3,0]*q[3,0]));
    e3 = t2+t1;

    return np.matrix([[e1],[e2],[e3]]);

def EP2Gibbs(q):
    """
    EP2Gibbs(Q1)

    	Q = EP2Gibbs(Q1) translates the Euler parameter vector Q1
    	into the Gibbs vector Q.
    """

    q1 = q[1,0]/q[0,0];
    q2 = q[2,0]/q[0,0];
    q3 = q[3,0]/q[0,0];

    return np.matrix([[q1],[q2],[q3]]);

def EP2MRP(q):
    """
    EP2MRP(Q1)

    	Q = EP2MRP(Q1) translates the Euler parameter vector Q1
    	into the MRP vector Q.
    """

    if q[0,0] < 0:
        q = -q;

    q1 = q[1,0]/(1+q[0,0]);
    q2 = q[2,0]/(1+q[0,0]);
    q3 = q[3,0]/(1+q[0,0]);

    return np.matrix([[q1],[q2],[q3]]);

def EP2PRV(q):
    """
    EP2PRV(Q1)

    	Q = EP2PRV(Q1) translates the Euler parameter vector Q1
    	into the principal rotation vector Q.
    """

    p = 2*math.acos(q[0,0]);
    sp = math.sin(p/2);
    q1 = q[1,0]/sp*p;
    q2 = q[2,0]/sp*p;
    q3 = q[3,0]/sp*p;

    return np.matrix([[q1],[q2],[q3]]);




def Euler1(x):
	"""
	EULER1 	Elementary rotation matrix
	Returns the elementary rotation matrix about the first body axis.
	"""
	m = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
	m[1,1] = math.cos(x)
	m[1,2] = math.sin(x)
	m[2,1]= -m[1,2]
	m[2,2] = m[1,1]
	
	return m

def Euler2(x):
	"""
	EULER2 	Elementary rotation matrix
	Returns the elementary rotation matrix about the
	second body axis.
	"""	
	m = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
	m[0,0] = math.cos(x);
	m[0,2] = -math.sin(x);
	m[2,0] = -m[0,2];
	m[2,2] = m[0,0];
	
	return m

def Euler3(x):
	"""
	EULER3 	Elementary rotation matrix
	Returns the elementary rotation matrix about the
	third body axis.
	"""
	m = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
	m[0,0] = math.cos(x);
	m[0,1] = math.sin(x);
	m[1,0]= -m[0,1];
	m[1,1] = m[0,0];

	return m


def Euler1212C(q):
	"""
	Euler1212C	
	
        C = Euler1212C(Q) returns the direction cosine
        matrix in terms of the 1-2-1 Euler angles.  
        Input Q must be a 3x1 vector of Euler angles.
	"""
	st1 = math.sin(q[0,0]);
	ct1 = math.cos(q[0,0]);
	st2 = math.sin(q[1,0]);
	ct2 = math.cos(q[1,0]);
	st3 = math.sin(q[2,0]);
	ct3 = math.cos(q[2,0]);
	
	C      = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
	C[0,0] = ct2;
	C[0,1] = st1*st2;
	C[0,2] = -ct1*st2;
	C[1,0] = st2*st3;
	C[1,1] = ct1*ct3-ct2*st1*st3;
	C[1,2] = ct3*st1+ct1*ct2*st3;
	C[2,0] = ct3*st2;
	C[2,1] = -ct2*ct3*st1-ct1*st3;
	C[2,2] = ct1*ct2*ct3-st1*st3;
	
	return C

def Euler1212EP(e):
	"""
	Euler1212EP(E)
	
        Q = Euler1212EP(E) translates the 121 Euler angle
        vector E into the Euler parameter vector Q.
	"""
	
	e1 = e[0,0]/2;
	e2 = e[1,0]/2;
	e3 = e[2,0]/2;
	
	q0 = math.cos(e2)*math.cos(e1+e3);
	q1 = math.cos(e2)*math.sin(e1+e3);
	q2 = math.sin(e2)*math.cos(e1-e3);
	q3 = math.sin(e2)*math.sin(e1-e3);

	return np.matrix([[q0],[q1],[q2],[q3]])


def Euler1212Gibbs(e):
	"""
	Euler1212Gibbs(E)
	
        Q = Euler1212Gibbs(E) translates the (1-2-1) Euler
        angle vector E into the Gibbs vector Q.
	"""
	
	return EP2Gibbs(Euler1212EP(e));

def Euler1212MRP(e):
    """
    Euler1212MRP(E)

    	Q = Euler1212MRP(E) translates the (1-2-1) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler1212EP(e));

def Euler1212PRV(e):
    """
    Euler1212PRV(E)

    	Q = Euler1212PRV(E) translates the (1-2-1) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler1212EP(e));

def Euler1232C(q):
    """
    Euler1232C

    	C = Euler1232C(Q) returns the direction cosine
    	matrix in terms of the 1-2-3 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct2*ct3;
    C[0,1] = ct3*st1*st2+ct1*st3;
    C[0,2] = st1*st3-ct1*ct3*st2;
    C[1,0] = -ct2*st3;
    C[1,1] = ct1*ct3-st1*st2*st3;
    C[1,2] = ct3*st1+ct1*st2*st3;
    C[2,0] = st2;
    C[2,1] = -ct2*st1;
    C[2,2] = ct1*ct2;

    return C;

def Euler1232EP(e):
    """
    Euler1232EP(E)

    	Q = Euler1232EP(E) translates the 123 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    c1 = math.cos(e[0,0]/2);
    s1 = math.sin(e[0,0]/2);
    c2 = math.cos(e[1,0]/2);
    s2 = math.sin(e[1,0]/2);
    c3 = math.cos(e[2,0]/2);
    s3 = math.sin(e[2,0]/2);

    q0 = c1*c2*c3-s1*s2*s3;
    q1 = s1*c2*c3+c1*s2*s3;
    q2 = c1*s2*c3-s1*c2*s3;
    q3 = c1*c2*s3+s1*s2*c3;

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler1232Gibbs(e):
    """
    Euler1232Gibbs(E)

    	Q = Euler1232Gibbs(E) translates the (1-2-3) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler1232EP(e));

def Euler1232MRP(e):
    """
    Euler1232MRP(E)

    	Q = Euler1232MRP(E) translates the (1-2-3) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler1232EP(e));

def Euler1232PRV(e):
    """
    Euler1232PRV(E)

    	Q = Euler1232PRV(E) translates the (1-2-3) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler1232EP(e));

def Euler1312C(q):
    """
    Euler1312C

    	C = Euler1312C(Q) returns the direction cosine
    	matrix in terms of the 1-3-1 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct2;
    C[0,1] = ct1*st2;
    C[0,2] = st1*st2;
    C[1,0] = -ct3*st2;
    C[1,1] = ct1*ct2*ct3-st1*st3;
    C[1,2] = ct2*ct3*st1+ct1*st3;
    C[2,0] = st2*st3;
    C[2,1] = -ct3*st1-ct1*ct2*st3;
    C[2,2] = ct1*ct3-ct2*st1*st3;

    return C;

def Euler1312EP(e):
    """
    Euler1312EP(E)

    	Q = Euler1312EP(E) translates the 131 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    e1 = e[0,0]/2;
    e2 = e[1,0]/2;
    e3 = e[2,0]/2;

    q0 = math.cos(e2)*math.cos(e1+e3);
    q1 = math.cos(e2)*math.sin(e1+e3);
    q2 = math.sin(e2)*math.sin(-e1+e3);
    q3 = math.sin(e2)*math.cos(-e1+e3);

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler1312Gibbs(e):
    """
    Euler1312Gibbs(E)

    	Q = Euler1312Gibbs(E) translates the (1-3-1) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler1312EP(e));

def Euler1312MRP(e):
    """
    Euler1312MRP(E)

    	Q = Euler1312MRP(E) translates the (1-3-1) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler1312EP(e));

def Euler1312PRV(e):
    """
    Euler1312PRV(E)

    	Q = Euler1312PRV(E) translates the (1-3-1) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler1312EP(e));

def Euler1322C(q):
    """
    Euler1322C

    	C = Euler1322C(Q) returns the direction cosine
    	matrix in terms of the 1-3-2 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct2*ct3;
    C[0,1] = ct1*ct3*st2+st1*st3;
    C[0,2] = ct3*st1*st2-ct1*st3;
    C[1,0] = -st2;
    C[1,1] = ct1*ct2;
    C[1,2] = ct2*st1;
    C[2,0] = ct2*st3;
    C[2,1] = -ct3*st1+ct1*st2*st3;
    C[2,2] = ct1*ct3+st1*st2*st3;

    return C;

def Euler1322EP(e):
    """
    Euler1322EP(E)

    	Q = Euler1322EP(E) translates the 132 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    c1 = math.cos(e[0,0]/2);
    s1 = math.sin(e[0,0]/2);
    c2 = math.cos(e[1,0]/2);
    s2 = math.sin(e[1,0]/2);
    c3 = math.cos(e[2,0]/2);
    s3 = math.sin(e[2,0]/2);

    q0 = c1*c2*c3+s1*s2*s3;
    q1 = s1*c2*c3-c1*s2*s3;
    q2 = c1*c2*s3-s1*s2*c3;
    q3 = c1*s2*c3+s1*c2*s3;

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler1322Gibbs(e):
    """
    Euler1322Gibbs(E)

    	Q = Euler1322Gibbs(E) translates the (1-3-2) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler1322EP(e));

def Euler1322MRP(e):
    """
    Euler1322MRP(E)

    	Q = Euler1322MRP(E) translates the (1-3-2) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler1322EP(e));

def Euler1322PRV(e):
    """
    Euler1322PRV(E)

    	Q = Euler1322PRV(E) translates the (1-3-2) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler1322EP(e));

def Euler2122C(q):
    """
    Euler2122C

    	C = Euler2122C(Q) returns the direction cosine
    	matrix in terms of the 2-1-2 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct1*ct3-ct2*st1*st3;
    C[0,1] = st2*st3;
    C[0,2] = -ct3*st1-ct1*ct2*st3;
    C[1,0] = st1*st2;
    C[1,1] = ct2;
    C[1,2] = ct1*st2;
    C[2,0] = ct2*ct3*st1+ct1*st3;
    C[2,1] = -ct3*st2;
    C[2,2] = ct1*ct2*ct3-st1*st3;

    return C;

def Euler2132C(q):
    """
    Euler2132C

    	C = Euler2132C(Q) returns the direction cosine
    	matrix in terms of the 2-1-3 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct1*ct3+st1*st2*st3;
    C[0,1] = ct2*st3;
    C[0,2] = -ct3*st1+ct1*st2*st3;
    C[1,0] = ct3*st1*st2-ct1*st3;
    C[1,1] = ct2*ct3;
    C[1,2] = ct1*ct3*st2 + st1*st3;
    C[2,0] = ct2*st1;
    C[2,1] = -st2;
    C[2,2] = ct1*ct2;

    return C;

def Euler2312C(q):
    """
    Euler2312C

    	C = Euler2312C(Q) returns the direction cosine
    	matrix in terms of the 2-3-1 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct1*ct2;
    C[0,1] = st2;
    C[0,2] = -ct2*st1;
    C[1,0] = -ct1*ct3*st2+st1*st3;
    C[1,1] = ct2*ct3;
    C[1,2] = ct3*st1*st2+ct1*st3;
    C[2,0] = ct3*st1+ct1*st2*st3;
    C[2,1] = -ct2*st3;
    C[2,2] = ct1*ct3-st1*st2*st3;

    return C;

def Euler2322C(q):
    """
    Euler2322C

    	C = Euler2322C(Q) returns the direction cosine
    	matrix in terms of the 2-3-2 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct1*ct2*ct3-st1*st3;
    C[0,1] = ct3*st2;
    C[0,2] = -ct2*ct3*st1-ct1*st3;
    C[1,0] = -ct1*st2;
    C[1,1] = ct2;
    C[1,2] = st1*st2;
    C[2,0] = ct3*st1+ct1*ct2*st3;
    C[2,1] = st2*st3;
    C[2,2] = ct1*ct3-ct2*st1*st3;

    return C;

def Euler3122C(q):
    """
    Euler3122C

    	C = Euler3122C(Q) returns the direction cosine
    	matrix in terms of the 1-2-3 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct1*ct3-st1*st2*st3;
    C[0,1] = ct3*st1+ct1*st2*st3;
    C[0,2] = -ct2*st3;
    C[1,0] = -ct2*st1;
    C[1,1] = ct1*ct2;
    C[1,2] = st2;
    C[2,0] = ct3*st1*st2+ct1*st3;
    C[2,1] = st1*st3-ct1*ct3*st2;
    C[2,2] = ct2*ct3;

    return C;

def Euler3132C(q):
    """
    Euler3132C

    	C = Euler3132C(Q) returns the direction cosine
    	matrix in terms of the 3-1-3 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct3*ct1-st3*ct2*st1;
    C[0,1] = ct3*st1+st3*ct2*ct1;
    C[0,2] = st3*st2;
    C[1,0] = -st3*ct1-ct3*ct2*st1;
    C[1,1] = -st3*st1+ct3*ct2*ct1;
    C[1,2] = ct3*st2;
    C[2,0] = st2*st1;
    C[2,1] = -st2*ct1;
    C[2,2] = ct2;

    return C;

def Euler3212C(q):
    """
    Euler3212C

    	C = Euler3212C(Q) returns the direction cosine
    	matrix in terms of the 3-2-1 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct2*ct1;
    C[0,1] = ct2*st1;
    C[0,2] = -st2;
    C[1,0] = st3*st2*ct1-ct3*st1;
    C[1,1] = st3*st2*st1+ct3*ct1;
    C[1,2] = st3*ct2;
    C[2,0] = ct3*st2*ct1+st3*st1;
    C[2,1] = ct3*st2*st1-st3*ct1;
    C[2,2] = ct3*ct2;

    return C;

def Euler3232C(q):
    """
    Euler3232C

    	C = Euler3232C(Q) returns the direction cosine
    	matrix in terms of the 3-2-3 Euler angles.
    	Input Q must be a 3x1 vector of Euler angles.
    """

    st1 = math.sin(q[0,0]);
    ct1 = math.cos(q[0,0]);
    st2 = math.sin(q[1,0]);
    ct2 = math.cos(q[1,0]);
    st3 = math.sin(q[2,0]);
    ct3 = math.cos(q[2,0]);

    C = np.matrix("1. 0. 0.;0. 1. 0.;0. 0. 1.");
    C[0,0] = ct1*ct2*ct3-st1*st3;
    C[0,1] = ct2*ct3*st1+ct1*st3;
    C[0,2] = -ct3*st2;
    C[1,0] = -ct3*st1-ct1*ct2*st3;
    C[1,1] = ct1*ct3-ct2*st1*st3;
    C[1,2] = st2*st3;
    C[2,0] = ct1*st2;
    C[2,1] = st1*st2;
    C[2,2] = ct2;

    return C;

def Euler2122EP(e):
    """
    Euler2122EP(E)

    	Q = Euler2122EP(E) translates the 212 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    e1 = e[0,0]/2;
    e2 = e[1,0]/2;
    e3 = e[2,0]/2;

    q0 = math.cos(e2)*math.cos(e1+e3);
    q1 = math.sin(e2)*math.cos(-e1+e3);
    q2 = math.cos(e2)*math.sin(e1+e3);
    q3 = math.sin(e2)*math.sin(-e1+e3);

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler2132EP(e):
    """
    Euler2132EP(E)

    	Q = Euler2132EP(E) translates the 213 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    c1 = math.cos(e[0,0]/2);
    s1 = math.sin(e[0,0]/2);
    c2 = math.cos(e[1,0]/2);
    s2 = math.sin(e[1,0]/2);
    c3 = math.cos(e[2,0]/2);
    s3 = math.sin(e[2,0]/2);

    q0 = c1*c2*c3+s1*s2*s3;
    q1 = c1*s2*c3+s1*c2*s3;
    q2 = s1*c2*c3-c1*s2*s3;
    q3 = c1*c2*s3-s1*s2*c3;

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler2312EP(e):
    """
    Euler2312EP(E)

    	Q = Euler2312EP(E) translates the 231 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    c1 = math.cos(e[0,0]/2);
    s1 = math.sin(e[0,0]/2);
    c2 = math.cos(e[1,0]/2);
    s2 = math.sin(e[1,0]/2);
    c3 = math.cos(e[2,0]/2);
    s3 = math.sin(e[2,0]/2);

    q0 = c1*c2*c3-s1*s2*s3;
    q1 = c1*c2*s3+s1*s2*c3;
    q2 = s1*c2*c3+c1*s2*s3;
    q3 = c1*s2*c3-s1*c2*s3;

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler2322EP(e):
    """
    Euler2322EP(E)

    	Q = Euler2322EP(E) translates the 232 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    e1 = e[0,0]/2;
    e2 = e[1,0]/2;
    e3 = e[2,0]/2;

    q0 = math.cos(e2)*math.cos(e1+e3);
    q1 = math.sin(e2)*math.sin(e1-e3);
    q2 = math.cos(e2)*math.sin(e1+e3);
    q3 = math.sin(e2)*math.cos(e1-e3);

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler3122EP(e):
    """
    Euler3122EP(E)

    	Q = Euler3122EP(E) translates the 312 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    c1 = math.cos(e[0,0]/2);
    s1 = math.sin(e[0,0]/2);
    c2 = math.cos(e[1,0]/2);
    s2 = math.sin(e[1,0]/2);
    c3 = math.cos(e[2,0]/2);
    s3 = math.sin(e[2,0]/2);

    q0 = c1*c2*c3-s1*s2*s3;
    q1 = c1*s2*c3-s1*c2*s3;
    q2 = c1*c2*s3+s1*s2*c3;
    q3 = s1*c2*c3+c1*s2*s3;

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler3132EP(e):
    """
    Euler3132EP(E)

    	Q = Euler3132EP(E) translates the 313 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    e1 = e[0,0]/2;
    e2 = e[1,0]/2;
    e3 = e[2,0]/2;

    q0 = math.cos(e2)*math.cos(e1+e3);
    q1 = math.sin(e2)*math.cos(e1-e3);
    q2 = math.sin(e2)*math.sin(e1-e3);
    q3 = math.cos(e2)*math.sin(e1+e3);

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler3212EP(e):
    """
    Euler3212EP(E)

    	Q = Euler3212EP(E) translates the 321 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    c1 = math.cos(e[0,0]/2);
    s1 = math.sin(e[0,0]/2);
    c2 = math.cos(e[1,0]/2);
    s2 = math.sin(e[1,0]/2);
    c3 = math.cos(e[2,0]/2);
    s3 = math.sin(e[2,0]/2);

    q0 = c1*c2*c3+s1*s2*s3;
    q1 = c1*c2*s3-s1*s2*c3;
    q2 = c1*s2*c3+s1*c2*s3;
    q3 = s1*c2*c3-c1*s2*s3;

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler3232EP(e):
    """
    Euler3232EP(E)

    	Q = Euler3232EP(E) translates the 323 Euler angle
    	vector E into the Euler parameter vector Q.
    """

    e1 = e[0,0]/2;
    e2 = e[1,0]/2;
    e3 = e[2,0]/2;

    q0 = math.cos(e2)*math.cos(e1+e3);
    q1 = math.sin(e2)*math.sin(-e1+e3);
    q2 = math.sin(e2)*math.cos(-e1+e3);
    q3 = math.cos(e2)*math.sin(e1+e3);

    return np.matrix([[q0],[q1],[q2],[q3]]);

def Euler2122Gibbs(e):
    """
    Euler2122Gibbs(E)

    	Q = Euler2122Gibbs(E) translates the (2-1-2) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler2122EP(e));

def Euler2122MRP(e):
    """
    Euler2122MRP(E)

    	Q = Euler2122MRP(E) translates the (2-1-2) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler2122EP(e));

def Euler2122PRV(e):
    """
    Euler2122PRV(E)
    
    	Q = Euler2122PRV(E) translates the (2-1-2) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler2122EP(e));

def Euler2132Gibbs(e):
    """
    Euler2132Gibbs(E)
    
    	Q = Euler2132Gibbs(E) translates the (2-1-3) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler2132EP(e));

def Euler2132MRP(e):
    """
    Euler2132MRP(E)
    
    	Q = Euler2132MRP(E) translates the (2-1-3) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler2132EP(e));

def Euler2132PRV(e):
    """
    Euler2132PRV(E)
    
    	Q = Euler2132PRV(E) translates the (2-1-3) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler2132EP(e));

def Euler2312Gibbs(e):
    """
    Euler2312Gibbs(E)
    
    	Q = Euler2312Gibbs(E) translates the (2-3-1) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler2312EP(e));

def Euler2312MRP(e):
    """
    Euler2312MRP(E)
    
    	Q = Euler2312MRP(E) translates the (2-3-1) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler2312EP(e));

def Euler2312PRV(e):
    """
    Euler2312PRV(E)
    
    	Q = Euler2312PRV(E) translates the (2-3-1) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler2312EP(e));

def Euler2322Gibbs(e):
    """
    Euler2322Gibbs(E)
    
    	Q = Euler2322Gibbs(E) translates the (2-3-2) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler2322EP(e));

def Euler2322MRP(e):
    """
    Euler2322MRP(E)
    
    	Q = Euler2322MRP(E) translates the (2-3-2) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler2322EP(e));

def Euler2322PRV(e):
    """
    Euler2322PRV(E)
    
    	Q = Euler2322PRV(E) translates the (2-3-2) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler2322EP(e));

def Euler3122Gibbs(e):
    """
    Euler3122Gibbs(E)
    
    	Q = Euler3122Gibbs(E) translates the (3-1-2) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler3122EP(e));

def Euler3122MRP(e):
    """
    Euler3122MRP(E)
    
    	Q = Euler3122MRP(E) translates the (3-1-2) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler3122EP(e));

def Euler3122PRV(e):
    """
    Euler3122PRV(E)
    
    	Q = Euler3122PRV(E) translates the (3-1-2) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler3122EP(e));

def Euler3132Gibbs(e):
    """
    Euler3132Gibbs(E)
    
    	Q = Euler3132Gibbs(E) translates the (3-1-3) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler3132EP(e));

def Euler3132MRP(e):
    """
    Euler3132MRP(E)
    
    	Q = Euler3132MRP(E) translates the (3-1-3) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler3132EP(e));

def Euler3132PRV(e):
    """
    Euler3132PRV(E)
    
    	Q = Euler3132PRV(E) translates the (3-1-3) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler3132EP(e));

def Euler3212Gibbs(e):
    """
    Euler3212Gibbs(E)
    
    	Q = Euler3212Gibbs(E) translates the (3-2-1) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler3212EP(e));

def Euler3212MRP(e):
    """
    Euler3212MRP(E)
    
    	Q = Euler3212MRP(E) translates the (3-2-1) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler3212EP(e));

def Euler3212PRV(e):
    """
     Euler3212PRV(E)
    
    	Q = Euler3212PRV(E) translates the (3-2-1) Euler
    	angle vector E into the principal rotation vector Q.
    """

    return EP2PRV(Euler3212EP(e));

def Euler3232Gibbs(e):
    """
    Euler3232Gibbs(E)
    
    	Q = Euler3232Gibbs(E) translates the (3-2-3) Euler
    	angle vector E into the Gibbs vector Q.
    """

    return EP2Gibbs(Euler3232EP(e));

def Euler3232MRP(e):
    """
    Euler3232MRP(E)
    
    	Q = Euler3232MRP(E) translates the (3-2-3) Euler
    	angle vector E into the MRP vector Q.
    """

    return EP2MRP(Euler3232EP(e));

def Euler3232PRV(e):
    """
    Euler3232PRV(E)
    
    	Q = Euler3232PRV(E) translates the (3-2-3) Euler
    	angle vector Q1 into the principal rotation vector Q.
    """

    return EP2PRV(Euler3232EP(e));



