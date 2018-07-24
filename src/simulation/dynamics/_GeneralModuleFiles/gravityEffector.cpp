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


#include "gravityEffector.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/avsEigenMRP.h"
#include "utilities/linearAlgebra.h"

SphericalHarmonics::SphericalHarmonics()
{
    this->radEquator = 0.0;
    this->maxDeg = 0;
    this->muBody = 0.0;
    return;
}

SphericalHarmonics::~SphericalHarmonics()
{
    return;
}

/*
@brief Computes the term (2 - d_l), where d_l is the kronecker delta.
*/
double SphericalHarmonics::getK(const unsigned int degree)
{
    return ((degree == 0) ? 1.0 : 2.0);
}

bool SphericalHarmonics::initializeParameters()
{
    bool paramsDone = false;
    
    //! - If coefficients haven't been loaded, quit and return failure
    if(cBar.size() == 0 || sBar.size() == 0)
    {
        return paramsDone;
    }
    
    for(unsigned int i = 0; i <= maxDeg + 1; i++)
    {
        std::vector<double> aRow, n1Row, n2Row;
        aRow.resize(i+1, 0.0);
        // Diagonal elements of A_bar
        if (i == 0)
        {
             aRow[i] = 1.0;
        }
        else
        {
            aRow[i] = sqrt(double((2*i+1)*getK(i))/(2*i*getK(i-1))) * aBar[i-1][i-1];
        }
        n1Row.resize(i+1, 0.0);
        n2Row.resize(i+1, 0.0);
        for (unsigned int m = 0; m <= i; m++)
        {
            if (i >= m + 2)
            {
                n1Row[m] = sqrt(double((2*i+1)*(2*i-1))/((i-m)*(i+m)));
                n2Row[m] = sqrt(double((i+m-1)*(2*i+1)*(i-m-1))/((i+m)*(i-m)*(2*i-3)));
            
            }
        }
        n1.push_back(n1Row);
        n2.push_back(n2Row);
        aBar.push_back(aRow);
    }
    
    for (unsigned int l = 0; l <= maxDeg; l++) // up to _maxDegree-1
    {
        std::vector<double> nq1Row, nq2Row;
        nq1Row.resize(l+1, 0.0);
        nq2Row.resize(l+1, 0.0);
        for (unsigned int m = 0; m <= l; m++)
        {
            if (m < l)
            {
                nq1Row[m] = sqrt(double((l-m)*getK(m)*(l+m+1))/getK(m+1));
            }
            nq2Row[m] = sqrt(double((l+m+2)*(l+m+1)*(2*l+1)*getK(m))/((2*l+3)*getK(m+1)));
        }
        nQuot1.push_back(nq1Row);
        nQuot2.push_back(nq2Row);
    }
    paramsDone = true;
    
    return paramsDone;
}

///---------------------------------Main Interface----------------------------///
/*!
 @brief Use to compute the field in position pos, given in a body frame.
 @param[in] pos Position in which the field is to be computed.
 @param[in] degree used to compute the field.
 @param[out] acc Vector including the computed field.
 @param[in] include_zero_degree Boolean that determines whether the zero-degree term is included.
 */
Eigen::Vector3d SphericalHarmonics::computeField(const Eigen::Vector3d pos_Pfix, unsigned int degree,
    bool include_zero_degree)
{
    double x = pos_Pfix[0];
    double y = pos_Pfix[1];
    double z = pos_Pfix[2];
    double r, s, t, u;
    double order;
    double rho;
    double a1, a2, a3, a4, sum_a1, sum_a2, sum_a3, sum_a4;
    std::vector<double> rE, iM, rhol;
    Eigen::Vector3d acc;
    acc.fill(0.0);
    
    // Change of variables: direction cosines
    r = sqrt(x*x + y*y + z*z);
    s = x/r;
    t = y/r;
    u = z/r;
    
    // maximum degree!
    if (degree > maxDeg)
        degree = maxDeg;
    
    order = degree;
    
    for (unsigned int l = 1; l <= degree+1; l++)
    {
        //Diagonal terms are computed in initialize()
        // Low diagonal terms
        aBar[l][l-1] = sqrt(double((2*l)*getK(l-1))/getK(l)) * aBar[l][l] * u;
    }
    
    // Lower terms of A_bar
    for (unsigned int m = 0; m <= order+1; m++)
    {
        for(unsigned int l = m + 2; l <= degree+1; l++)
        {
            aBar[l][m] = u * n1[l][m] * aBar[l-1][m] - n2[l][m] * aBar[l-2][m];

        }
        
        // Computation of real and imaginary parts of (2+j*t)^m
        if (m == 0)
        {
            rE.push_back(1.0);
            iM.push_back(0.0);
        }
        else
        {
            rE.push_back(s * rE[m-1] - t * iM[m-1]);
            iM.push_back(s * iM[m-1] + t * rE[m-1]);
        }

        
    }
    
    rho = radEquator/r;
    rhol.resize(degree+2, 0.0);
    rhol[0] = muBody/r;
    rhol[1] = rhol[0]*rho;

    
    // Degree 0
    
    // Gravity field and potential of degree l = 0
    // Gravity components
    a1 = 0.0;
    a2 = 0.0;
    a3 = 0.0;
    a4 = 0.0;
    
    if (include_zero_degree == true)
    {
        a4 = -rhol[1]/radEquator; // * this->_Nquot_2[0][0] * this->_A_bar[1][1]; //This is 1, so it's not included!
    }
    
    for (unsigned int l = 1; l <= degree; l++) // does not include l = maxDegree
    {
        rhol[l+1] =  rho * rhol[l]; // rho_l computed

        sum_a1 = 0.0;
        sum_a2 = 0.0;
        sum_a3 = 0.0;
        sum_a4 = 0.0;
        
        for(unsigned int m = 0; m <= l; m++)
        {
            double D, E, F;
            D = cBar[l][m] * rE[m] + sBar[l][m] * iM[m];
            if (m == 0)
            {
                E = 0.0;
                F = 0.0;
            }
            else
            {
                E = cBar[l][m] * rE[m-1] + sBar[l][m] * iM[m-1];
                F = sBar[l][m] * rE[m-1] - cBar[l][m] * iM[m-1];
            }
            
            //            if (l < degree)   // Gravity contains up to max_degree-1 harmonics
            //            {
            sum_a1 = sum_a1 + m * aBar[l][m] * E;
            sum_a2 = sum_a2 + m * aBar[l][m] * F;
            if (m < l)
            {
                sum_a3 = sum_a3 + nQuot1[l][m] * aBar[l][m+1] * D;
            }
            sum_a4 = sum_a4 + nQuot2[l][m] * aBar[l+1][m+1] * D;
            //            }
            
        }
        
        //        if (l < degree)   // Gravity contains up to max_degree-1 harmonics
        //        {
        a1 = a1 + rhol[l+1]/radEquator * sum_a1;
        a2 = a2 + rhol[l+1]/radEquator * sum_a2;
        a3 = a3 + rhol[l+1]/radEquator * sum_a3;
        a4 = a4 - rhol[l+1]/radEquator * sum_a4;
        //        }
    }
    
    acc[0] = a1 + s * a4;
    acc[1] = a2 + t * a4;
    acc[2] = a3 + u * a4;
    
    return acc;
}

bool SphericalHarmonics::harmReady()
{
    bool harmGood = true;

    harmGood = harmGood && cBar.size() > 0;
    harmGood = harmGood && sBar.size() > 0;
    harmGood = harmGood && aBar.size()> 0;
    
    return harmGood;
}

/*--------------------------------------------------------------------------------------------------*/
// GravBodyData implementation

/*!
 @brief Set parameters for a gravity body
 */
GravBodyData::GravBodyData()
{
    this->useSphericalHarmParams = false;
    this->isCentralBody = false;
    this->isDisplayBody = false;
    this->mu = 0;                      //!< [m3/s^2] central body gravitational param
    this->ephemTime = 0;               //!< [s]      Ephemeris time for the body in question
    this->ephIntTime = 0;              //!< [s]      Integration time associated with the ephem data
    this->radEquator = 0;              //!< [m]      Equatorial radius for the body
    this->spherHarm.maxDeg = 0;
    // Default these values to zero just in case they don't get populated
    this->localPlanet.J2000Current = 0.0;
    v3SetZero(this->localPlanet.PositionVector);
    v3SetZero(this->localPlanet.VelocityVector);
    m33SetIdentity(this->localPlanet.J20002Pfix);
    m33SetZero(this->localPlanet.J20002Pfix_dot);
    return;
}

/*!
 @brief Destructor.
 */
GravBodyData::~GravBodyData()
{
    
}

void GravBodyData::initBody(uint64_t moduleID)
{
    bool spherFound;
    spherFound = this->spherHarm.initializeParameters();
    this->bodyMsgID = SystemMessaging::GetInstance()->subscribeToMessage(
                    this->bodyInMsgName, sizeof(SpicePlanetStateSimMsg), moduleID);
    this->mu = spherFound ? this->spherHarm.muBody : this->mu;
    this->radEquator = spherFound ? this->spherHarm.radEquator : this->radEquator;
    return;
}

Eigen::Vector3d GravBodyData::computeGravityInertial(Eigen::Vector3d r_I,
    uint64_t simTimeNanos)
{
    Eigen::Vector3d gravOut;
    
    double rMag = r_I.norm();
    gravOut  = -r_I*this->mu/(rMag*rMag*rMag);
    
    if(this->spherHarm.harmReady() && this->useSphericalHarmParams)
    {
        double dt = ((int64_t) simTimeNanos - (int64_t) this->localHeader.WriteClockNanos)*NANO2SEC;
        Eigen::Matrix3d dcm_PfixN = Eigen::Map<Eigen::Matrix3d>
            (&(this->localPlanet.J20002Pfix[0][0]), 3, 3);
        Eigen::Matrix3d dcm_PfixN_dot = Eigen::Map<Eigen::Matrix3d>
        (&(this->localPlanet.J20002Pfix_dot[0][0]), 3, 3);
        dcm_PfixN += dcm_PfixN_dot * dt;
        dcm_PfixN.transposeInPlace();
        Eigen::Vector3d r_Pfix = dcm_PfixN*r_I;
        Eigen::Vector3d gravPert_Pfix = this->spherHarm.computeField(r_Pfix,
            this->spherHarm.maxDeg, false);
        gravOut += dcm_PfixN.transpose() * gravPert_Pfix;
    }
    
    return(gravOut);
}

double GravBodyData::computePotentialEnergy(Eigen::Vector3d r_I)
{
    double gravPotentialEnergyOut  = -this->mu/r_I.norm();
    return gravPotentialEnergyOut;
}

void GravBodyData::loadEphemeris(uint64_t moduleID)
{
    SystemMessaging::GetInstance()->ReadMessage(this->bodyMsgID, &this->localHeader,
        sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t *>(&this->localPlanet));
    return;
}

GravityEffector::GravityEffector()
{
    this->centralBody = nullptr;
    this->systemTimeCorrPropName = "systemTime";
    this->vehicleGravityPropName = "g_N";
    this->centralBodyOutMsgName = "central_body_spice";
    this->centralBodyOutMsgId = -1;
    this->inertialPositionPropName = "r_BN_N";
    this->inertialVelocityPropName = "v_BN_N";
    return;
}

GravityEffector::~GravityEffector()
{
    return;
}

void GravityEffector::SelfInit()
{
    if (this->centralBody) {
        this->centralBodyOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->centralBodyOutMsgName, sizeof(SpicePlanetStateSimMsg), 2, "SpicePlanetStateSimMsg", this->moduleID);
    }
    return;
}

void GravityEffector::CrossInit()
{
    //! Begin method steps
    //! - For each gravity body in the data vector, find message ID
    std::vector<GravBodyData *>::iterator it;
    for(it = this->gravBodies.begin(); it != this->gravBodies.end(); it++)
    {
        (*it)->initBody(this->moduleID);
    }
}

void GravityEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! Begin method steps
    //! - Updates the grav body planet ephemerides
    std::vector<GravBodyData *>::iterator it;
    for(it = this->gravBodies.begin(); it != this->gravBodies.end(); it++)
    {
        (*it)->loadEphemeris(this->moduleID);
        if((*it)->isCentralBody){
            this->centralBody = (*it);
        }
    }
    this->writeOutputMessages(CurrentSimNanos);
    return;
}

void GravityEffector::writeOutputMessages(uint64_t currentSimNanos)
{
    if (this->centralBodyOutMsgId > 0) {
        SystemMessaging::GetInstance()->WriteMessage(this->centralBodyOutMsgId, currentSimNanos, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*> (&this->centralBody->localPlanet), this->moduleID);
    }
    return;
}

void GravityEffector::prependSpacecraftNameToStates()
{
    this->inertialPositionPropName = this->nameOfSpacecraftAttachedTo + "_" + this->inertialPositionPropName;
    this->inertialVelocityPropName = this->nameOfSpacecraftAttachedTo + "_" +this->inertialVelocityPropName;
    this->vehicleGravityPropName = this->nameOfSpacecraftAttachedTo + "_" + this->vehicleGravityPropName;

    return;
}

void GravityEffector::registerProperties(DynParamManager& statesIn)
{
    Eigen::Vector3d gravInit;
    gravInit.fill(0.0);
    this->gravProperty = statesIn.createProperty(this->vehicleGravityPropName, gravInit);
    this->inertialPositionProperty = statesIn.createProperty(this->inertialPositionPropName, gravInit);
    this->inertialVelocityProperty = statesIn.createProperty(this->inertialVelocityPropName, gravInit);
}

void GravityEffector::linkInStates(DynParamManager& statesIn)
{
    this->timeCorr = statesIn.getPropertyReference(this->systemTimeCorrPropName);
}

/*!Calculate gravitational acceleration of s/c wrt inertial (no central body) or wrt central body
@params r_cF_N is position of center of mass of s/c wrt frame it is stored/integrated in in spacecraft
@params rDot_cF_N is the derivative of above
*/
void GravityEffector::computeGravityField(Eigen::Vector3d r_cF_N, Eigen::Vector3d rDot_cF_N)
{
    std::vector<GravBodyData *>::iterator it;
    uint64_t systemClock = this->timeCorr->data()[0];
    Eigen::Vector3d r_cN_N;          //position of s/c CoM wrt N
    Eigen::Vector3d r_CN_N;          //inertial position of central body if there is one. Big C is central body. Little c is CoM of s/c
    Eigen::Vector3d r_PN_N;          //position of Planet being queried wrt N
    Eigen::Vector3d r_cP_N;          //position of s/c CoM wrt planet in N
    Eigen::Vector3d rDotDot_cF_N;    //acceleration of CoM of s/c wrt Frame in which it is stored/integrated in spacecraft
    rDotDot_cF_N.fill(0.0);
    Eigen::Vector3d rDotDot_cN_N_P;  //acceleration of c wrt N in N, due to P
    
    if (this->centralBody){   //Evaluates true if there is a central body, false otherwise
        r_CN_N = getEulerSteppedGravBodyPosition(this->centralBody);
        r_cN_N = r_cF_N + r_CN_N; //shift s/c to be wrt inertial frame origin if it isn't already
    }else{
        r_cN_N = r_cF_N;
    }
    
    for(it = this->gravBodies.begin(); it != this->gravBodies.end(); it++)
    {
        r_PN_N = getEulerSteppedGravBodyPosition(*it);
        r_cP_N = r_cN_N - r_PN_N;
        if(this->centralBody)   //Evaluates true if there is a central body, false otherwise
        {
            if(this->centralBody != (*it))
            {
                rDotDot_cF_N += (*it)->computeGravityInertial(r_PN_N - r_CN_N, systemClock); //subtract accel of central body due to other bodies to get RELATIVE accel of s/c. See Vallado on "Three-body and n-body Equations"
            }
        }
        rDotDot_cN_N_P = (*it)->computeGravityInertial(r_cP_N, systemClock); //acc of s/c wrt N CoM in Frame used for s/c dynamics
        rDotDot_cF_N += rDotDot_cN_N_P;
    }
    
    *this->gravProperty = rDotDot_cF_N;
}
/*!Calculate gravitational acceleration of s/c wrt inertial (no central body) or wrt central body
 @params r_BF_N is position of body frame of s/c wrt frame it is stored/integrated in in spacecraft
 @params rDot_BF_N is the derivative of above
 */
void GravityEffector::updateInertialPosAndVel(Eigen::Vector3d r_BF_N, Eigen::Vector3d rDot_BF_N)
{
    // Here we add the central body inertial position and velocities to the central-body-relative
    // position and velicities which are propogated relative to the central body
    if(this->centralBody)   //Evaluates true if there is a central body, false otherwise
    {
        Eigen::Vector3d r_CN_N = getEulerSteppedGravBodyPosition(this->centralBody);
        *this->inertialPositionProperty = r_CN_N + r_BF_N;
        *this->inertialVelocityProperty = Eigen::Map<Eigen::MatrixXd>(&(this->centralBody->localPlanet.VelocityVector[0]), 3, 1) + rDot_BF_N;
    } else {
        *this->inertialPositionProperty = r_BF_N;
        *this->inertialVelocityProperty = rDot_BF_N;
    }
}
/*!Calculate gravitational acceleration of s/c wrt inertial (no central body) or wrt central body
 @params r_cF_N is position of center of mass of s/c wrt frame it is stored/integrated in in spacecraft
 @params rDot_cF_N is the derivative of above
 */
Eigen::Vector3d GravityEffector::getEulerSteppedGravBodyPosition(GravBodyData *bodyData)
{
    uint64_t systemClock = this->timeCorr->data()[0];
    double dt = (systemClock - bodyData->localHeader.WriteClockNanos)*NANO2SEC;
    Eigen::Vector3d r_PN_N = Eigen::Map<Eigen::MatrixXd>
    (&(bodyData->localPlanet.PositionVector[0]), 3, 1);
    r_PN_N += Eigen::Map<Eigen::Vector3d>
    (&(bodyData->localPlanet.VelocityVector[0]), 3, 1)*dt;
    return r_PN_N;
}

void GravityEffector::updateEnergyContributions(Eigen::Vector3d r_cF_N, double & orbPotEnergyContr)
{
    Eigen::Vector3d r_CN_N;  // C is central body. position of C wrt N in N
    Eigen::Vector3d r_PN_N;  // P is planet being queried. position of planet wrt N in N
    Eigen::Vector3d r_cP_N;  // c is s/c CoM. position of c wrt P in N
    Eigen::Vector3d r_cN_N;  // position c wrt N in N
    
    if(this->centralBody){   //Evaluates true if there is a central body, false otherwise
        r_CN_N = getEulerSteppedGravBodyPosition(this->centralBody);
        r_cN_N = r_cF_N + r_CN_N; //shift s/c to be wrt inertial frame origin if it isn't already
    }else{
        r_cN_N = r_cF_N;
    }
    
    std::vector<GravBodyData *>::iterator it;
    for(it = this->gravBodies.begin(); it != this->gravBodies.end(); it++)
    {
        r_PN_N = getEulerSteppedGravBodyPosition(*it);
        r_cP_N = r_cN_N - r_PN_N;
        
        if(this->centralBody)   //Evaluates true if there is a central body, false otherwise
        {
            if(this->centralBody != (*it))
            {
                orbPotEnergyContr += (*it)->computePotentialEnergy(r_PN_N - r_CN_N); //potential of central body w/in current planet field. leads to relative potential energy solution
            }
        }
        orbPotEnergyContr = (*it)->computePotentialEnergy(r_cP_N); //Potential w/in current planet field
    }
    return;
}

void GravityEffector::setGravBodies(std::vector<GravBodyData *> gravBodies)
{
    this->gravBodies = gravBodies;
    return;
}

void GravityEffector::addGravBody(GravBodyData* gravBody)
{
    this->gravBodies.push_back(gravBody);
    return;
}
