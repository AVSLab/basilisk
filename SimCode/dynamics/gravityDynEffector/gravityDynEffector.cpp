/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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


#include "gravityDynEffector.h"
#include "utilities/simMacros.h"

SphericalHarmonics::SphericalHarmonics()
{
    this->radEquator = 0.0;
    this->maxDeg = 0;
    this->muBody = 0.0;
}

SphericalHarmonics::~SphericalHarmonics()
{
    
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
    
    for(unsigned int i = 0; i < maxDeg + 1; i++)
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
    
    for (unsigned int l = 0; l < maxDeg; l++) // up to _maxDegree-1
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
    
    for (unsigned int l = 1; l <= degree; l++)
    {
        //Diagonal terms are computed in initialize()
        // Low diagonal terms
        aBar[l][l-1] = sqrt(double((2*l)*getK(l-1))/getK(l)) * aBar[l][l] * u;
    }
    
    // Lower terms of A_bar
    for (unsigned int m = 0; m <= order; m++)
    {
        for(unsigned int l = m + 2; l <= degree; l++)
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
    rhol.resize(degree+1, 0.0);
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
    
    for (unsigned int l = 1; l < degree; l++) // does not include l = maxDegree
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
 @brief Use this constructor to use the class as the old structure. Should be deprecated soon.
 */
GravBodyData::GravBodyData()
{
    useSphericalHarmParams = false;
    isCentralBody = false;
    isDisplayBody = false;
    mu=0;                      //!< [m3/s^2] central body gravitational param
    ephemTime=0;               //!< [s]      Ephemeris time for the body in question
    ephIntTime=0;              //!< [s]      Integration time associated with the ephem data
    radEquator=0;              //!< [m]      Equatorial radius for the body
    spherHarm.maxDeg = 0;
}


/*!
 @brief Copy constructor.
 */
//GravBodyData::GravBodyData(const GravBodyData& gravBody)
//{
//    isCentralBody = gravBody.isCentralBody;
//    isDisplayBody = gravBody.isDisplayBody;
//    useSphericalHarmParams = gravBody.useSphericalHarmParams;
//
//    posFromEphem = gravBody.posFromEphem;
//    velFromEphem = gravBody.velFromEphem;
//    j20002Pfix = gravBody.j20002Pfix;
//    j20002Pfix_dot = gravBody.j20002Pfix_dot;
//    mu = gravBody.mu;
//    ephemTime = gravBody.ephemTime;
//    ephIntTime = gravBody.ephIntTime;
//    radEquator = gravBody.radEquator;
//    bodyMsgName = gravBody.bodyMsgName;
//    outputMsgName = gravBody.outputMsgName;
//    planetEphemName = gravBody.planetEphemName;
//    outputMsgID = gravBody.outputMsgID;
//    BodyMsgID = gravBody.BodyMsgID;
//    spherHarm = gravBody.spherHarm;
//    return;
//}

/*!
 @brief Destructor.
 */
GravBodyData::~GravBodyData()
{
    
}

void GravBodyData::initBody(uint64_t moduleID)
{
    bool spherFound;
    spherFound = spherHarm.initializeParameters();
    bodyMsgID = SystemMessaging::GetInstance()->subscribeToMessage(
                    bodyMsgName, sizeof(SpicePlanetState), moduleID);
    mu = spherFound ? spherHarm.muBody : mu;
    radEquator = spherFound ? spherHarm.radEquator : radEquator;
    
}

Eigen::Vector3d GravBodyData::computeGravityInertial(Eigen::Vector3d r_I,
    uint64_t simTimeNanos)
{
    Eigen::Vector3d gravOut;
    
    double rMag = r_I.norm();
    gravOut  = -r_I*mu/(rMag*rMag*rMag);
    
    if(spherHarm.harmReady() && useSphericalHarmParams)
    {
        double dt = (simTimeNanos - localHeader.WriteClockNanos)*NANO2SEC;
        Eigen::Matrix3d dcm_PfixN = Eigen::Map<Eigen::Matrix3d>
            (&(localPlanet.J20002Pfix[0][0]), 3, 3);
        Eigen::Matrix3d dcm_PfixN_dot = Eigen::Map<Eigen::Matrix3d>
        (&(localPlanet.J20002Pfix_dot[0][0]), 3, 3);
        dcm_PfixN += dcm_PfixN_dot * dt;
        Eigen::Vector3d r_Pfix = dcm_PfixN*r_I;
        Eigen::Vector3d gravPert_Pfix = spherHarm.computeField(r_Pfix,
            spherHarm.maxDeg, false);
        gravOut += dcm_PfixN.transpose() * gravPert_Pfix;
        
    }
    
    return(gravOut);
}

void GravBodyData::loadEphemeris(uint64_t moduleID)
{
    SystemMessaging::GetInstance()->ReadMessage(bodyMsgID, &localHeader,
        sizeof(SpicePlanetState), reinterpret_cast<uint8_t *>(&localPlanet));
}

///*!
// @brief Operator = overloaded.
// */
//GravBodyData& GravBodyData::operator=(const GravBodyData& gravBody)
//{
////    if (this == &gravBody) {
////        return *this;
////    }
////    
////    this->IsCentralBody = gravBody.IsCentralBody;
////    this->IsDisplayBody = gravBody.IsDisplayBody;
////    this->UseJParams = gravBody.UseJParams;
////    this->UseSphericalHarmParams = gravBody.UseSphericalHarmParams;
////    this->JParams = gravBody.JParams;
////    
////    for(unsigned int i = 0; i < 3; i++){
////        this->PosFromEphem[i] = gravBody.PosFromEphem[i];
////        this->VelFromEphem[i] = gravBody.VelFromEphem[i];
////        
////        this->posRelDisplay[i] = gravBody.posRelDisplay[i];
////        this->velRelDisplay[i] = gravBody.posRelDisplay[i];
////        
////        for(unsigned int j = 0; j < 3; j++) {
////            this->J20002Pfix[i][j] = gravBody.J20002Pfix[i][j];
////            this->J20002Pfix_dot[i][j] = gravBody.J20002Pfix_dot[i][j];
////        }
////    }
////    
////    this->mu = gravBody.mu;
////    this->ephemTime = gravBody.ephemTime;
////    this->ephIntTime = gravBody.ephIntTime;
////    this->radEquator = gravBody.radEquator;
////    this->BodyMsgName = gravBody.BodyMsgName;
////    this->outputMsgName = gravBody.outputMsgName;
////    this->planetEphemName = gravBody.planetEphemName;
////    this->outputMsgID = gravBody.outputMsgID;
////    this->BodyMsgID = gravBody.BodyMsgID;
////    
////    if (gravBody._coeff_loader != nullptr) {
////        this->_coeff_loader = new coeffLoaderCSV(*(gravBody._coeff_loader));
////    }
////    else
////    this->_coeff_loader = nullptr;
////    
////    if (gravBody._spherHarm != nullptr) {
////        this->_spherHarm = new sphericalHarmonics(*(gravBody._spherHarm));
////        this->_spherHarm->setCoefficientLoader(this->_coeff_loader);
////    }
////    else
////    this->_spherHarm = nullptr;
//    
//    return *this;
//}

GravityDynEffector::GravityDynEffector()
{
    centralBody = nullptr;
    vehiclePositionStateName = "hubPosition";
    systemTimeCorrPropName = "systemTime";
    vehicleGravityPropName = "g_N";
    return;
}


GravityDynEffector::~GravityDynEffector()
{
    return;
}

void GravityDynEffector::registerProperties(DynParamManager& statesIn)
{
    Eigen::Vector3d gravInit;
    gravInit.fill(0.0);
    gravProperty = statesIn.createProperty(vehicleGravityPropName, gravInit);
}

void GravityDynEffector::linkInStates(DynParamManager& statesIn)
{
    posState = statesIn.getStateObject(vehiclePositionStateName);
    timeCorr = statesIn.getPropertyReference(systemTimeCorrPropName);
}

void GravityDynEffector::computeGravityField()
{
    std::vector<GravBodyData *>::iterator it;
    uint64_t systemClock = timeCorr->data()[0];
    Eigen::Vector3d gravOut;
    gravOut.fill(0.0);
    for(it = gravBodies.begin(); it != gravBodies.end(); it++)
    {
        Eigen::Vector3d posRelBody_N;
        posRelBody_N = posState->getState();
        Eigen::Vector3d mappedPos;
        mappedPos.fill(0.0);
        double dt;
        dt = (systemClock - (*it)->localHeader.WriteClockNanos)*NANO2SEC;
        mappedPos = Eigen::Map<Eigen::MatrixXd>
            (&((*it)->localPlanet.PositionVector[0]), 3, 1);
        mappedPos += Eigen::Map<Eigen::Vector3d>
            (&((*it)->localPlanet.VelocityVector[0]), 3, 1)*dt;
        posRelBody_N -= mappedPos;
        if(centralBody)
        {
            dt = (systemClock - centralBody->localHeader.WriteClockNanos)*NANO2SEC;
            Eigen::Vector3d centralPos = Eigen::Map<Eigen::MatrixXd>
            (&(centralBody->localPlanet.PositionVector[0]), 3, 1);
            centralPos += Eigen::Map<Eigen::Vector3d>
            (&(centralBody->localPlanet.VelocityVector[0]), 3, 1)*dt;
            posRelBody_N += centralPos;
            if(centralBody != (*it))
            {
                Eigen::Vector3d frmGrav =
                    (*it)->computeGravityInertial(centralPos-mappedPos , systemClock);
                gravOut += frmGrav;
            }
        }
        
        Eigen::Vector3d bodyGrav = (*it)->computeGravityInertial(posRelBody_N,
            systemClock);
        gravOut += bodyGrav;
    }
    *gravProperty = gravOut;
}

void GravityDynEffector::SelfInit()
{
    
}

void GravityDynEffector::CrossInit()
{
    //! Begin method steps
    //! - For each gravity body in the data vector, find message ID
    //! - If message ID is not found, alert the user and disable message
    std::vector<GravBodyData *>::iterator it;
    for(it = gravBodies.begin(); it != gravBodies.end(); it++)
    {
        (*it)->initBody(moduleID);
    }
    
    
}

void GravityDynEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! Begin method steps
    //! - For each gravity body in the data vector, find message ID
    //! - If message ID is not found, alert the user and disable message
    std::vector<GravBodyData *>::iterator it;
    for(it = gravBodies.begin(); it != gravBodies.end(); it++)
    {
        (*it)->loadEphemeris(moduleID);
        if((*it)->isCentralBody)
        {
            centralBody = (*it);
        }
    }
}
