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
#include "dynamics/deprecatedCode/SixDofEOM/six_dof_eom.h"
#include "environment/spice/spice_interface.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include <cstring>
#include <iostream>
#include <math.h>
#include "architecture/messaging/system_messaging.h"

/*--------------------------------------------------------------------------------------------------*/
// GravityBodyData implementation

/*!
 @brief Use this constructor to use the class as the old structure. Should be deprecated soon.
 */
GravityBodyData::GravityBodyData()
{
    this->UseSphericalHarmParams = false;
    this->_coeff_loader = nullptr;
    this->_spherHarm = nullptr;
	IsCentralBody = false;
	IsDisplayBody = false;
	UseJParams = false;
	UseSphericalHarmParams = false;
	memset(PosFromEphem, 0x0, 3 * sizeof(double));
	memset(VelFromEphem, 0x0, 3 * sizeof(double));
	memset(J20002Pfix, 0x0, 9*sizeof(double));        //!<          Transformation matrix from J2000 to planet-fixed
	memset(J20002Pfix_dot, 0x0, 9 * sizeof(double));    //!<          Derivative of the transformation matrix from J2000 to planet-fixed
	memset(posRelDisplay, 0x0, 3 * sizeof(double));        //!< [m]      Position of planet relative to display frame
	memset(velRelDisplay, 0x0, 3 * sizeof(double));        //!< [m]      Velocity of planet relative to display frame
	mu=0;                      //!< [m3/s^2] central body gravitational param
	ephemTime=0;               //!< [s]      Ephemeris time for the body in question
	ephIntTime=0;              //!< [s]      Integration time associated with the ephem data
	radEquator=0;              //!< [m]      Equatorial radius for the body
	ephemTimeSimNanos=0;
}

/*!
 @brief Constructor used only for bodies with an associated spherical harmonics model 
 @param[in] sphHarm_filename Filename of the coefficients file.
 @param[in] max_degree Maximum degree that the model will use. If it's larger than the maximum degree contained in the file, max_degree will be truncated by the loader.
 @param[in] mu Gravitational parameter.
 @param[in] reference_radius Radius of reference with which the coefficients were estimated. Usually, a mean radius at Equator, but it could be other number. It should be given along with the coefficients file.
 */
GravityBodyData::GravityBodyData(const std::string& sphHarm_filename, const unsigned int max_degree, const double mu_in, const double reference_radius)
{
	IsCentralBody = false;
	IsDisplayBody = false;
	UseJParams = false;
	UseSphericalHarmParams = false;
	memset(PosFromEphem, 0x0, 3 * sizeof(double));
	memset(VelFromEphem, 0x0, 3 * sizeof(double));
	memset(J20002Pfix, 0x0, 9 * sizeof(double));        //!<          Transformation matrix from J2000 to planet-fixed
	memset(J20002Pfix_dot, 0x0, 9 * sizeof(double));    //!<          Derivative of the transformation matrix from J2000 to planet-fixed
	memset(posRelDisplay, 0x0, 3 * sizeof(double));        //!< [m]      Position of planet relative to display frame
	memset(velRelDisplay, 0x0, 3 * sizeof(double));        //!< [m]      Velocity of planet relative to display frame
	mu = mu_in;                      //!< [m3/s^2] central body gravitational param
	ephemTime = 0;               //!< [s]      Ephemeris time for the body in question
	ephIntTime = 0;              //!< [s]      Integration time associated with the ephem data
	radEquator = reference_radius;              //!< [m]      Equatorial radius for the body
	ephemTimeSimNanos = 0;
    this->UseSphericalHarmParams = true;
    this->UseJParams = false;
    
    //if (file_format.compare("CSV") == 0) { //CSV are the only possible files by now
    this->_coeff_loader = new coeffLoaderCSV();
    //}
    
    this->_spherHarm = new sphericalHarmonics(this->_coeff_loader, sphHarm_filename, max_degree, mu, reference_radius);
    
    this->mu = mu_in;
    this->radEquator = reference_radius;
    
    return;
}

/*!
 @brief Copy constructor.
 */
GravityBodyData::GravityBodyData(const GravityBodyData& gravBody)
{
    this->IsCentralBody = gravBody.IsCentralBody;
    this->IsDisplayBody = gravBody.IsDisplayBody;
    this->UseJParams = gravBody.UseJParams;
    this->UseSphericalHarmParams = gravBody.UseSphericalHarmParams;
    this->JParams = gravBody.JParams;

    for(unsigned int i = 0; i < 3; i++){
        this->PosFromEphem[i] = gravBody.PosFromEphem[i];
        this->VelFromEphem[i] = gravBody.VelFromEphem[i];
        
        this->posRelDisplay[i] = gravBody.posRelDisplay[i];
        this->velRelDisplay[i] = gravBody.posRelDisplay[i];
        
        for(unsigned int j = 0; j < 3; j++) {
            this->J20002Pfix[i][j] = gravBody.J20002Pfix[i][j];
            this->J20002Pfix_dot[i][j] = gravBody.J20002Pfix_dot[i][j];
        }
    }
    
    this->mu = gravBody.mu;
    this->ephemTime = gravBody.ephemTime;
    this->ephIntTime = gravBody.ephIntTime;
    this->radEquator = gravBody.radEquator;
    this->BodyMsgName = gravBody.BodyMsgName;
    this->outputMsgName = gravBody.outputMsgName;
    this->planetEphemName = gravBody.planetEphemName;
    this->outputMsgID = gravBody.outputMsgID;
    this->BodyMsgID = gravBody.BodyMsgID;
    
    if (gravBody._coeff_loader != nullptr) {
        this->_coeff_loader = new coeffLoaderCSV(*(gravBody._coeff_loader));
    }
    else
        this->_coeff_loader = nullptr;
    
    if (gravBody._spherHarm != nullptr) {
        this->_spherHarm = new sphericalHarmonics(*(gravBody._spherHarm));
        this->_spherHarm->setCoefficientLoader(this->_coeff_loader);
    }
    else
        this->_spherHarm = nullptr;
    
    return;
}

/*! 
 @brief Destructor.
 */
GravityBodyData::~GravityBodyData()
{
    if (this->_spherHarm != nullptr)
        delete this->_spherHarm;
    
    if (this->_coeff_loader != nullptr)
        delete this->_coeff_loader;
}

/*!
 @brief Operator = overloaded.
 */
GravityBodyData& GravityBodyData::operator=(const GravityBodyData& gravBody)
{
    if (this == &gravBody) {
        return *this;
    }
    
    this->IsCentralBody = gravBody.IsCentralBody;
    this->IsDisplayBody = gravBody.IsDisplayBody;
    this->UseJParams = gravBody.UseJParams;
    this->UseSphericalHarmParams = gravBody.UseSphericalHarmParams;
    this->JParams = gravBody.JParams;
    
    for(unsigned int i = 0; i < 3; i++){
        this->PosFromEphem[i] = gravBody.PosFromEphem[i];
        this->VelFromEphem[i] = gravBody.VelFromEphem[i];
        
        this->posRelDisplay[i] = gravBody.posRelDisplay[i];
        this->velRelDisplay[i] = gravBody.posRelDisplay[i];
        
        for(unsigned int j = 0; j < 3; j++) {
            this->J20002Pfix[i][j] = gravBody.J20002Pfix[i][j];
            this->J20002Pfix_dot[i][j] = gravBody.J20002Pfix_dot[i][j];
        }
    }
    
    this->mu = gravBody.mu;
    this->ephemTime = gravBody.ephemTime;
    this->ephIntTime = gravBody.ephIntTime;
    this->radEquator = gravBody.radEquator;
    this->BodyMsgName = gravBody.BodyMsgName;
    this->outputMsgName = gravBody.outputMsgName;
    this->planetEphemName = gravBody.planetEphemName;
    this->outputMsgID = gravBody.outputMsgID;
    this->BodyMsgID = gravBody.BodyMsgID;
    
    if (gravBody._coeff_loader != nullptr) {
        this->_coeff_loader = new coeffLoaderCSV(*(gravBody._coeff_loader));
    }
    else
        this->_coeff_loader = nullptr;
    
    if (gravBody._spherHarm != nullptr) {
        this->_spherHarm = new sphericalHarmonics(*(gravBody._spherHarm));
        this->_spherHarm->setCoefficientLoader(this->_coeff_loader);
    }
    else
        this->_spherHarm = nullptr;
    
    return *this;
}

/*!
 @brief Returns the sphericalHarmonics object.
 */
sphericalHarmonics* GravityBodyData::getSphericalHarmonicsModel(void)
{
    return this->_spherHarm;
}

/*--------------------------------------------------------------------------------------------------*/


/*! This is the constructor for SixDofEOM.  It initializes a few variables*/
SixDofEOM::SixDofEOM()
    : outputStateMessage("inertial_state_output")
    , outputMassPropsMsg("spacecraft_mass_props")
    , centralBodyOutMsgName("central_body_data")
    , centralBodyOutMsgId(-1)
{
    this->CallCounts = 0;
    this->RWACount = reactWheels.size();
    this->numHRB = hingedRigidBodies.size(); //! Get number of hinged rigid bodies
    this->OutputBufferCount = 2;
    this->MRPSwitchCount = 0;
    this->useTranslation = false;
    this->useRotation    = false;
    // By default gravity will be on, but is only enabled if translation is on as well
    this->useGravity     = true;
    this->XState = nullptr;

    /* initialize some spacecraft states to default values.  The user should always override these values
     with the desired values.  These defaults are set to avoid crashes if the dynamic mode doesn't set or update these */
    m33SetIdentity(this->baseI);
    v3SetZero(this->baseCoM);
    v3SetZero(this->sigma_BN);
    v3SetZero(this->omega_BN_B);
    v3Set(1.0, 0.0, 0.0, this->r_BN_N);
    v3Set(1.0, 0.0, 0.0, this->v_BN_N);
    /* default to an idendity BS DCM */
    m33SetIdentity(this->T_str2Bdy);
    this->T_Str2BdyInit.assign(9, 0);
    this->T_Str2BdyInit[0] = this->T_Str2BdyInit[4] = this->T_Str2BdyInit[8] = 1.0;

    this->Integrator = new rk4Integrator(this); // Default integrator: RK4
    this->DefaultIntegrator = true;
    return;
}

/*! Destructor.*/
SixDofEOM::~SixDofEOM()
{
    if (XState != nullptr) { // There was a memory leak. This array was not being released (Manuel)
        delete[] XState;
    }
    
    if (DefaultIntegrator == true && Integrator != nullptr) {
        delete Integrator;
    }
    return;
}

/*! This method exists to add a new gravitational body in to the simulation to
    be used to effect the spacecraft dynamics.  
    @return void
    @param newGravityBody A pointer to the gravitational body that is being added
*/
void SixDofEOM::AddGravityBody(GravityBodyData *newGravityBody)
{
    gravData.push_back(*newGravityBody);
}

/*! This method exists to attach an effector to the vehicle's dynamics.  The
 effector should be a derived class of the DynEffector abstract class and it
 should include a ComputeDynamics call which is operated by dynamics.
 @return void
 @param NewEffector The effector that we are adding to dynamics
 */
void SixDofEOM::addBodyEffector(DynEffector *newBodyEffector)
{
    this->bodyEffectors.push_back(newBodyEffector);
}

/*! This method exists to attach a thrust set effector to the vehicle's dynamics.
    The effector should be a derived class of the DynEffector abstract class and it
    should include a ComputeDynamics call which is operated by dynamics.
    @return void
    @param newThrusterSet The effector that we are adding to dynamics
*/
void SixDofEOM::addThrusterSet(ThrusterDynamics *newThrusterSet)
{
    this->thrusters.push_back(newThrusterSet);
}

/*! This method exists to attach an effector to the vehicle's dynamics.  The
effector should be a derived class of the DynEffector abstract class and it
should include a ComputeDynamics call which is operated by dynamics.
@return void
@param newReactionWheelSet The effector that we are adding to dynamics
*/
void SixDofEOM::addReactionWheelSet(ReactionWheelDynamics *newReactionWheelSet)
{
    std::vector<ReactionWheelConfigData>::iterator it;
    for(it= newReactionWheelSet->ReactionWheelData.begin(); it != newReactionWheelSet->ReactionWheelData.end(); it++)
    {
        reactWheels.push_back(&(*it));
    }
}

/*! This method exists to attach an effector to the vehicle's dynamics.  The
 effector should be a derived class of the DynEffector abstract class and it
 should include a ComputeDynamics call which is operated by dynamics.
 @return void
 @param newHingedRigidEffector The effector that we are adding to dynamics
 */
void SixDofEOM::addHingedRigidBodySet(HingedRigidBodies *newHingedRigidEffector)
{
    hingedRigidBodies.push_back(newHingedRigidEffector);
}

/*! This method exists to attach an effector to the vehicle's dynamics.  The
 effector should be a derived class of the DynEffector abstract class and it
 should include a ComputeDynamics call which is operated by dynamics.
 @return void
 @param newFuelTank The effector that we are adding to dynamics
 */
void SixDofEOM::addFuelTank(FuelTank *newFuelTank)
{
    fuelTanks.push_back(newFuelTank);
}

/*!
 * @brief This method changes the integrator in use (Default integrator: RK4)
 * @param Pointer to an integrator object.
 */
void SixDofEOM::setIntegrator(integrator *NewIntegrator)
{
    if (DefaultIntegrator == true && Integrator != nullptr)
        delete Integrator;
    
    Integrator = NewIntegrator;
    DefaultIntegrator = false;
    
    return;
}

/*! This method creates an output message for each planetary body that computes
    the planet's ephemeris information in the display reference frame.  Note that 
    the underlying assumption is that the display reference frame is always 
    oriented the same as the ECI axes and is always rotating the same as the 
    ECI axes
    @return void
*/
void SixDofEOM::initPlanetStateMessages()
{
 
    std::vector<GravityBodyData>::iterator it;
    for(it = gravData.begin(); it != gravData.end(); it++)
    {
        it->outputMsgID = -1;
        if(it->outputMsgName.size() > 0)
        {
            it->outputMsgID= SystemMessaging::GetInstance()->CreateNewMessage(
                it->outputMsgName, sizeof(SpicePlanetState), OutputBufferCount,
                "SpicePlanetState",moduleID);
        }
    }
}

/*! This method initializes the state of the dynamics system to the init 
    values passed in by the user.  There is potentially some value in removing 
    the Init intermediary variables but they exist for now.
    @return void
*/
void SixDofEOM::SelfInit()
{
    //! Begin method steps
    //! - Zero out initial states prior to copying in init values
	this->RWACount = 0;
    this->numRWJitter = 0;
    std::vector<ReactionWheelConfigData *>::iterator rwIt;

    for (rwIt = reactWheels.begin(); rwIt != reactWheels.end(); rwIt++)
    {
        this->RWACount++;
        if ((*rwIt)->usingRWJitter) {this->numRWJitter++;}
    }

    this->numHRB = 0;
    std::vector<HingedRigidBodies *>::iterator itSP;
    std::vector<HingedRigidBodyConfigData>::iterator HRBIt;
    for (itSP = hingedRigidBodies.begin(); itSP != hingedRigidBodies.end(); itSP++)
    {
        this->numHRB = (*itSP)->hingedRigidBodyData.size();
    }

    this->numFSP = 0;
    std::vector<FuelTank *>::iterator itFT;
    std::vector<FuelSloshParticleConfigData>::iterator FSPIt;
    for (itFT = fuelTanks.begin(); itFT != fuelTanks.end(); itFT++)
    {
        this->numFSP = (*itFT)->fuelSloshParticlesData.size();
    }

    this->NStates = 0;
    if(this->useTranslation) this->NStates += 6;
    if(this->useRotation)    this->NStates += 6;
    this->NStates += this->RWACount + this->numRWJitter;
    this->NStates += this->numHRB*2;
    this->NStates += this->numFSP*2;
    if(this->NStates==0) {
        std::cerr << "ERROR: The simulation state vector is of size 0!";
    }

    this->XState = new double[this->NStates]; /* pos/vel/att/rate + rwa omegas + hinged dynamics (theta/thetadot) + fuel slosh (rho/rhoDot)*/
    memset(this->XState, 0x0, (this->NStates)*sizeof(double));
    timePrev = 0.0;
    
    //! - Ensure that all init states were appropriately set by the caller
    if(this->useTranslation){
        if(PositionInit.size() != 3 || VelocityInit.size() != 3)
        {
            std::cerr << "Your initial translational states didn't match up with right sizes\n";
            std::cerr << "Position: "<< PositionInit.size() << std::endl;
            std::cerr << "Velocity: "<< VelocityInit.size() << std::endl;
            return;
        }

    }
    if(this->useRotation){
        if(AttitudeInit.size() != 3 || AttRateInit.size() != 3 ||
           baseInertiaInit.size() != 9 || baseCoMInit.size() != 3 ||
           T_Str2BdyInit.size() != 9)
        {
            std::cerr << "Your initial rotational states didn't match up with right sizes\n";
            std::cerr << "Attitude: "<< AttitudeInit.size() << std::endl;
            std::cerr << "Att-rate: "<< AttRateInit.size() << std::endl;
            std::cerr << "Inertia: "<< baseInertiaInit.size() << std::endl;
            std::cerr << "CoM: "<< baseCoMInit.size() << std::endl;
            std::cerr << "Str2Bdy: "<< T_Str2BdyInit.size() << std::endl;
            return;
        }
    }

    //! - Zero out the accumulated DV (always starts at zero)
    memset(this->AccumDVBdy, 0x0, 3*sizeof(double));
    
    //! - For remaining variables, grab iterators for each vector and assign internals
    std::vector<double>::iterator PosIt = PositionInit.begin();
    std::vector<double>::iterator VelIt = VelocityInit.begin();
    std::vector<double>::iterator AttIt = AttitudeInit.begin();
    std::vector<double>::iterator RateIt= AttRateInit.begin();
    std::vector<double>::iterator InertiaIt= baseInertiaInit.begin();
    std::vector<double>::iterator CoMIt= baseCoMInit.begin();
    std::vector<double>::iterator Str2BdyIt= T_Str2BdyInit.begin();


    /* set the simulation specific initial conditions */
    for(uint32_t i=0; i<3; i++)
    {
        uint32_t c = 0;
        if (this->useTranslation){
            this->XState[i] = *PosIt++;
            this->XState[i+3] = *VelIt++;
            c += 6;
        }
        if (this->useRotation){
            this->XState[i+c]   = *AttIt++;
            this->XState[i+c+3] = *RateIt++;
            this->baseCoM[i]    = *CoMIt++;
            for(uint32_t j=0; j<3; j++)
            {
                this->baseI[i][j] = *InertiaIt++;
                this->T_str2Bdy[i][j] = *Str2BdyIt++;
            }

        }
    }

    uint32_t rwCount = 0;
    uint32_t rwJitterCount = 0;
    for (rwIt = reactWheels.begin(); rwIt != reactWheels.end(); rwIt++)
    {
        this->XState[this->useTranslation*6 + this->useRotation*6 + rwCount] = (*rwIt)->Omega;
        if ((*rwIt)->usingRWJitter) {    // set initial RW angle to zero
            this->XState[this->useTranslation*6 + this->useRotation*6 + this->RWACount + rwJitterCount] = 0.0;
            rwJitterCount++;
        }
        rwCount++;
    }

    uint32_t HRBIterator = 0;
    for (itSP=hingedRigidBodies.begin(); itSP != hingedRigidBodies.end(); itSP++)
    {
        for (HRBIt = (*itSP)->hingedRigidBodyData.begin();
             HRBIt != (*itSP)->hingedRigidBodyData.end(); HRBIt++)
        {
            this->XState[this->useTranslation*6 + this->useRotation*6 + this->RWACount + this->numRWJitter + HRBIterator] = HRBIt->theta;
            this->XState[this->useTranslation*6 + this->useRotation*6 + this->RWACount + this->numRWJitter + this->numHRB + HRBIterator] = HRBIt->thetaDot;
            HRBIterator++;
        }
    }

    uint32_t fspIterator = 0;
    for (itFT = fuelTanks.begin(); itFT != fuelTanks.end(); itFT++)
    {
        for (FSPIt = (*itFT)->fuelSloshParticlesData.begin();
             FSPIt != (*itFT)->fuelSloshParticlesData.end(); FSPIt++)
        {
            this->XState[this->useTranslation*6 + this->useRotation*6 + this->RWACount + this->numRWJitter + this->numHRB*2 + fspIterator] = FSPIt->rho;
            this->XState[this->useTranslation*6 + this->useRotation*6 + this->RWACount + this->numRWJitter + this->numHRB*2 + this->numFSP + fspIterator] = FSPIt->rhoDot;
            fspIterator++;
        }
    }
    
    //! - Write output messages for other modules that use the dynamics state in cross-init
    this->StateOutMsgID = SystemMessaging::GetInstance()->
        CreateNewMessage(this->outputStateMessage, sizeof(OutputStateData),
        OutputBufferCount, "OutputStateData", moduleID);
    
    this->MassPropsMsgID = SystemMessaging::GetInstance()->
        CreateNewMessage(this->outputMassPropsMsg, sizeof(MassPropsData),
        OutputBufferCount, "MassPropsData", moduleID);
    
    this->centralBodyOutMsgId = SystemMessaging::GetInstance()->
        CreateNewMessage(this->centralBodyOutMsgName, sizeof(SpicePlanetState),
        OutputBufferCount, "SpicePlanetState", moduleID);
    
    initPlanetStateMessages();
    
    //! - Call computeOutputs to ensure that the outputs are available post-init
    computeCompositeProperties();
    computeOutputs();
    WriteOutputMessages(0);
    
}

/*! This method links up all gravitational bodies with thei appropriate ephemeris
    source data.  It alerts the user if any of the gravitational bodies fail to 
    link correctly.
    @return void
*/
void SixDofEOM::CrossInit()
{
    //! Begin method steps
    //! - For each gravity body in the data vector, find message ID
    //! - If message ID is not found, alert the user and disable message
    std::vector<GravityBodyData>::iterator it;
    for(it = gravData.begin(); it != gravData.end(); it++)
    {
        it->BodyMsgID = SystemMessaging::GetInstance()->subscribeToMessage(
            it->BodyMsgName, sizeof(SpicePlanetState), moduleID);
        if(it->BodyMsgID < 0)
        {
            std::cerr << "WARNING: Did not find a valid message with name: ";
            std::cerr << it->BodyMsgName << " :" << __FILE__ <<std::endl;
            std::cerr << "Disabling body gravity." <<std::endl;
        }
    }
    
}

/*! This method reads in all of the gravity body state messages and saves off 
    the position of the body used to compute gravitational forces and to set 
    the output state correctly.
    @return void
*/
void SixDofEOM::ReadInputs()
{
    SpicePlanetState LocalPlanet;
    SingleMessageHeader LocalHeader;
    std::vector<GravityBodyData>::iterator it;
    
    //! Begin method steps
    //! - Loop through all valid gravity bodies and grab the ephem data
    for(it = gravData.begin(); it != gravData.end(); it++)
    {
        if(it->BodyMsgID >= 0)
        {
            SystemMessaging::GetInstance()->ReadMessage(it->BodyMsgID, &LocalHeader,
                                                        sizeof(SpicePlanetState), reinterpret_cast<uint8_t*> (&LocalPlanet), moduleID);
            memcpy(it->PosFromEphem, LocalPlanet.PositionVector, 3*sizeof(double));
            memcpy(it->VelFromEphem, LocalPlanet.VelocityVector, 3*sizeof(double));
            memcpy(it->J20002Pfix, LocalPlanet.J20002Pfix, 9*sizeof(double));
            memcpy(it->J20002Pfix_dot, LocalPlanet.J20002Pfix_dot, 9*sizeof(double));
            it->ephemTime = LocalPlanet.J2000Current;
            it->planetEphemName = LocalPlanet.PlanetName;
            it->ephemTimeSimNanos = LocalHeader.WriteClockNanos;
        }
    }
}


/*! This method is used to compute the non-spherical gravitational perturbation 
    from a planet.  It is designed to use the J2-J6 parameters and determine the 
    gravitational acceleration caused by those harmonics.
    @return void
    @param gravBody The data associated with the gravitational body
    @param r_N The current position vector of the vehicle relative to body
    @param perturbAccel_N The computed perturbation vector output from function
*/
void SixDofEOM::jPerturb(GravityBodyData *gravBody, double r_N[3],
    double perturbAccel_N[3])
{

    double temp[3];
    double temp2[3];
    double planetPos[3];
    
    m33MultV3(gravBody->J20002Pfix, r_N, planetPos);
    double rmag = v3Norm(planetPos);
    double x = planetPos[0];
    double y = planetPos[1];
    double z = planetPos[2];

    memset(perturbAccel_N, 0x0, 3*sizeof(double));
    /* Calculating the total acceleration based on user input */
    if(gravBody->JParams.size() > 0)
    {
        std::vector<double>::iterator it = gravBody->JParams.begin();
        perturbAccel_N[0] = 5.0*z*z/(rmag*rmag) - 1.0;
        perturbAccel_N[1] = perturbAccel_N[0];
        perturbAccel_N[2] = perturbAccel_N[0] - 2.0;
        perturbAccel_N[0] *= x;
        perturbAccel_N[1] *= y;
        perturbAccel_N[2] *= z;
        v3Scale(3.0/2.0*gravBody->mu * (*it) *
                gravBody->radEquator*gravBody->radEquator / (rmag*rmag*rmag*rmag*rmag), perturbAccel_N, perturbAccel_N );
    }
    if(gravBody->JParams.size()> 1)
    {
        std::vector<double>::iterator it = gravBody->JParams.begin()+1;
        v3Set(5.0 * (7.0 * pow(z / rmag, 3.0) - 3.0 * (z / rmag)) * (x / rmag),
              5.0 * (7.0 * pow(z / rmag, 3.0) - 3.0 * (z / rmag)) * (y / rmag),
              -3.0 * (10.0 * pow(z / rmag, 2.0) - (35.0 / 3.0)*pow(z / rmag, 4.0) - 1.0), temp);
        v3Scale(1.0 / 2.0 * (*it) * (gravBody->mu / pow(rmag, 2.0))*pow(gravBody->radEquator / rmag, 3.0), temp, temp2);
        v3Add(perturbAccel_N, temp2, perturbAccel_N);
    }
    if(gravBody->JParams.size()> 2) {
        std::vector<double>::iterator it = gravBody->JParams.begin()+2;
        v3Set((3.0 - 42.0 * pow(z / rmag, 2.0) + 63.0 * pow(z / rmag, 4.0)) * (x / rmag),
              (3.0 - 42.0 * pow(z / rmag, 2.0) + 63.0 * pow(z / rmag, 4.0)) * (y / rmag),
              (15.0 - 70.0 * pow(z / rmag, 2) + 63.0 * pow(z / rmag, 4.0)) * (z / rmag), temp);
        v3Scale(5.0 / 8.0 * (*it) * (gravBody->mu / pow(rmag, 2.0))*pow(gravBody->radEquator / rmag, 4.0), temp, temp2);
        v3Add(perturbAccel_N, temp2, perturbAccel_N);
    }
    if(gravBody->JParams.size()> 3) {
    std::vector<double>::iterator it = gravBody->JParams.begin()+3;
        v3Set(3.0 * (35.0 * (z / rmag) - 210.0 * pow(z / rmag, 3.0) + 231.0 * pow(z / rmag, 5.0)) * (x / rmag),
              3.0 * (35.0 * (z / rmag) - 210.0 * pow(z / rmag, 3.0) + 231.0 * pow(z / rmag, 5.0)) * (y / rmag),
              -(15.0 - 315.0 * pow(z / rmag, 2.0) + 945.0 * pow(z / rmag, 4.0) - 693.0 * pow(z / rmag, 6.0)), temp);
        v3Scale(1.0 / 8.0 * (*it) * (gravBody->mu / pow(rmag, 2.0))*pow(gravBody->radEquator / rmag, 5.0), temp, temp2);
        v3Add(perturbAccel_N, temp2, perturbAccel_N);
    }
    if(gravBody->JParams.size()> 4) {
    std::vector<double>::iterator it = gravBody->JParams.begin()+4;
        v3Set((35.0 - 945.0 * pow(z / rmag, 2) + 3465.0 * pow(z / rmag, 4.0) - 3003.0 * pow(z / rmag, 6.0)) * (x / rmag),
              (35.0 - 945.0 * pow(z / rmag, 2.0) + 3465.0 * pow(z / rmag, 4.0) - 3003.0 * pow(z / rmag, 6.0)) * (y / rmag),
              -(3003.0 * pow(z / rmag, 6.0) - 4851.0 * pow(z / rmag, 4.0) + 2205.0 * pow(z / rmag, 2.0) - 245.0) * (z / rmag), temp);
        v3Scale(-1.0 / 16.0 * (*it) * (gravBody->mu / pow(rmag, 2.0))*pow(gravBody->radEquator / rmag, 6.0), temp, temp2);
        v3Add(perturbAccel_N, temp2, perturbAccel_N);
    }
    m33tMultV3(gravBody->J20002Pfix, perturbAccel_N, perturbAccel_N);
}

/*! This method is used to compute the composite mass properties of the vehicle 
    for the current time.  It takes the base properties and the properties of 
    all dyn effectors and computes the current composite properties.  It's hard 
    to do this "right" because we aren't propagating the mass properties states 
    but this is a good way to start
    @return void
*/
void SixDofEOM::computeCompositeProperties()
{

    double scaledCoM[3];
    double localCoM[3];
    double identMatrix[3][3];
    double diracMatrix[3][3], outerMatrix[3][3];
    double CoMDiff[3], CoMDiffNormSquare;
    double objInertia[3][3];

    memset(scaledCoM, 0x0, 3*sizeof(double));
    this->compMass = this->baseMass;
    v3Scale(this->baseMass, this->baseCoM, scaledCoM);

    std::vector<ThrusterDynamics*>::iterator it;
    for(it=thrusters.begin(); it != thrusters.end(); it++)
    {
        DynEffector *TheEff = *it;
        v3Scale(TheEff->objProps.Mass, TheEff->objProps.CoM, localCoM);
        v3Add(scaledCoM, localCoM, scaledCoM);
        this->compMass += TheEff->objProps.Mass;
    }
    
    //! - Divide summation by total mass to get center of mass.
    v3Scale(1.0/this->compMass, scaledCoM, this->compCoM);
    
    //! - Compute the parallel axis theorem effects for the base body inertia tensor
    m33SetIdentity(identMatrix);
    v3Subtract(this->baseCoM, this->compCoM, CoMDiff);
    CoMDiffNormSquare = v3Norm(CoMDiff);
    CoMDiffNormSquare *= CoMDiffNormSquare;
    m33Scale(CoMDiffNormSquare, identMatrix, diracMatrix);
    v3OuterProduct(CoMDiff, CoMDiff, outerMatrix);
    m33Subtract(diracMatrix, outerMatrix, objInertia);
    m33Add(objInertia, this->baseI, this->compI);
    
    /*! - For each child body, compute parallel axis theorem effectos for inertia tensor
     and add that overall inertia back to obtain the composite inertia tensor.*/
    for(it=thrusters.begin(); it != thrusters.end(); it++)
    {
        DynEffector *TheEff = *it;
        v3Subtract(TheEff->objProps.CoM, this->compCoM, CoMDiff);
        CoMDiffNormSquare = v3Norm(CoMDiff);
        CoMDiffNormSquare *= CoMDiffNormSquare;
        m33Scale(CoMDiffNormSquare, identMatrix, diracMatrix);
        v3OuterProduct(CoMDiff, CoMDiff, outerMatrix);
        m33Subtract(diracMatrix, outerMatrix, objInertia);
        m33Scale(TheEff->objProps.Mass, objInertia, objInertia);
        vAdd(TheEff->objProps.InertiaTensor, 9, objInertia, objInertia);
        m33Add(this->compI, objInertia, this->compI);
    }
    //! - Compute inertia inverse based off the computed inertia tensor
    m33Inverse(this->compI, this->compIinv);
    
}

/*! This method computes the total gravity field using a point mass, a J-model, spherical harmonics, and third body effects.
    @param[in] t Current time.
    @param[in] r_BN_N Current position in inertial frame.
    @param[in] BN Current orientation of the body frame relative to inertial (given by a DCM).
    @param[out] g_N Gravity field in inertial frame.
 */
void SixDofEOM::computeGravity(double t, double r_BN_N[3], double BN[3][3], double *g_N)
{
    double rmag;
    double perturbAccel_N[3];
    
    //! - Get current position magnitude and compute the 2-body gravitational accels
    rmag = v3Norm(r_BN_N);
    v3Scale(-centralBody->mu / rmag / rmag / rmag, r_BN_N, g_N);
    
    /* compute the gravitational zonal harmonics or the spherical harmonics (never both)*/
    if(centralBody->UseJParams)
    {
        jPerturb(centralBody, r_BN_N, perturbAccel_N);
        v3Add(perturbAccel_N, g_N, g_N);
    }
    else if (centralBody->UseSphericalHarmParams)
    {
        unsigned int max_degree = centralBody->getSphericalHarmonicsModel()->getMaxDegree(); // Maximum degree to include
        double posPlanetFix[3]; // [m] Position in planet-fixed frame
        double gravField[3]; // [m/s^2] Gravity field in planet fixed frame
        
        double planetDt = t - centralBody->ephIntTime;
        double J2000PfixCurrent[3][3];
        
        m33Scale(planetDt, centralBody->J20002Pfix_dot, J2000PfixCurrent);
        m33Add(J2000PfixCurrent, centralBody->J20002Pfix, J2000PfixCurrent);
        m33MultV3(J2000PfixCurrent, r_BN_N, posPlanetFix); // r_E = [EN]*r_N
        centralBody->getSphericalHarmonicsModel()->computeField(posPlanetFix, max_degree, gravField, false);
        
        m33tMultV3(J2000PfixCurrent, gravField, perturbAccel_N); // [EN]^T * gravField
        
        v3Add(perturbAccel_N, g_N, g_N);
    }
    
    /*! - Zero the inertial accels and compute grav accel for all bodies other than central body.
     Gravity perturbation is acceleration on spacecraft+acceleration on central body.
     Ephemeris information is propagated by Euler's method in substeps*/
    v3SetZero(this->InertialAccels);
    std::vector<GravityBodyData>::iterator gravit;
    for(gravit = gravData.begin(); gravit != gravData.end(); gravit++)
    {
        double PlanetRelPos[3];
        double PlanetAccel[3];
        double posVelComp[3];
        
        if(gravit->IsCentralBody || gravit->BodyMsgID < 0)
        {
            continue;
        }
        v3Scale(t - centralBody->ephIntTime, centralBody->VelFromEphem,
                posVelComp);
        v3Add(r_BN_N, centralBody->PosFromEphem, PlanetRelPos);
        v3Add(PlanetRelPos, posVelComp, PlanetRelPos);
        v3Subtract(PlanetRelPos, gravit->PosFromEphem, PlanetRelPos);
        v3Scale(t - gravit->ephIntTime, gravit->VelFromEphem, posVelComp);
        v3Subtract(PlanetRelPos, posVelComp, PlanetRelPos);
        rmag = v3Norm(PlanetRelPos);
        v3Scale(-gravit->mu / rmag / rmag / rmag, PlanetRelPos, PlanetAccel);
        v3Add(this->InertialAccels, PlanetAccel, InertialAccels);
        v3Scale(t - gravit->ephIntTime, gravit->VelFromEphem, posVelComp);
        v3Subtract(centralBody->PosFromEphem, gravit->PosFromEphem, PlanetRelPos);
        v3Subtract(PlanetRelPos, posVelComp, PlanetRelPos);
        v3Scale(t - centralBody->ephIntTime, centralBody->VelFromEphem,
                posVelComp);
        v3Add(PlanetRelPos, posVelComp, PlanetRelPos);
        rmag = v3Norm(PlanetRelPos);
        v3Scale(gravit->mu/rmag/rmag/rmag, PlanetRelPos, PlanetAccel);
        v3Add(this->InertialAccels, PlanetAccel, this->InertialAccels);
    }
    
    //! - Add in inertial accelerations of the non-central bodies
    v3Add(this->InertialAccels, g_N, g_N);
}

/*! This method computes the state derivatives at runtime for the state integration.
    It handles all gravitational effects and nominal attitude motion itself.  
    All body effectors have their current force/torque computed via the 
    ComputeDynamics call that is inherited from the DynEffector abstract class.
    @return void
    @param t The current simulation time as double precision
    @param X The current state of the spacecraft
    @param dX The computed derivatives that we output to caller
*/
void SixDofEOM::equationsOfMotion(double t, double *X, double *dX)
{

    OutputStateData StateCurrent;
    MassPropsData MassProps;
    
    double r_BN_NLoc[3];        /* [m] position vector from N to B */
    double v_BN_NLoc[3];        /* [m/s] inertial time derivative of position vector from N to B */
    double sigma_BNLoc[3];      /* MRP from inertial to body frame */
    double omega_BN_BLoc[3];    /* [rad/s] angular velocity of body frame w/ respect to N */
    double B[3][3];             /* d(sigma)/dt = 1/4 B omega */
    double omegaTilde_BN_B[3][3];    /* tilde matrix of the omega B-frame components */
    double intermediateVector[3];               /* intermediate variables */
    double intermediateVector2[3];              /* intermediate variables */
    double intermediateMatrix[3][3];            /* intermediate variables */
    double intermediateMatrix2[3][3];           /* intermediate variables */
    double variableSumFuelSloshDynamics;        /* intermediate variables */
    double BN[3][3];            /* DCM from inertial to body frame */
    double LocalAccels_B[3];
    double LocalAccels_N[3];
    double extSumTorque_B[3];   /* net external torque acting on the body in B-frame components */
    std::vector<DynEffector*>::iterator effectorIt;
    int    i;
	double *Omegas;             /* pointer to the RW speeds */
    double *omegaDot_BN_B;      /* pointer to inertial body angular acceleration vector in B-frame components */
    double rwF_N[3];            /* simple RW jitter force in inertial frame */
    double rwA_N[3];            /* inertial simple RW jitter acceleration in inertial frame components */
    double *thetasSP;           /* pointer of theta values for hinged SP dynamics */
    double *thetaDotsSP;        /* pointer of time derivatives of thetas for hinged dynamics */
    double *thetaDDotsHRB;       /* pointer of 2nd time derivatives of thetas for hinged dynamics */
    double *rhosFS;             /* pointer of rho values for fuel slosh dynamics */
    double *rhoDotsFS;          /* pointer of time derivatives of rhos for fuel slosh dynamics */
    double *rhoDDotsFS;         /* pointer of 2nd time derivatives of rhos for fuel slosh dynamics */
    double rDDot_CN_N[3];       /* inertial accelerration of the center of mass of the sc in N frame */
    double g_B[3];              /* [m/s^2] gravity acceleration in the body frame */
    double gravityTorquePntB_B[3]; /* [N-m] torque due to gravity about point B in B frame components */
    double gravityTorqueHRBPntH_B[3]; /* [N-m] gravity torque on hinged rigid body about point H in B */
    double rDDot_CN_B[3];       /* inertial accelerration of the center of mass of the sc in B frame */
    double rDDot_BN_B[3];       /* inertial accelerration of r_BN in the body frame */
    double *matrixA; /* Matrix A needed for hinged SP dynamics */
    double *matrixE; /* Matrix E needed for hinged SP dynamics */
    double *matrixF; /* Matrix F needed for hinged SP dynamics */
    double *vectorV; /* Vector V needed for hinged SP dynamics */
    double *matrixR; /* Matrix R needed for hinged SP dynamics */
    double *matrixG; /* Matrix G needed for fuel slosh and hinged SP dynamics together */
    double *matrixN; /* Matrix N needed for fuel slosh and hinged SP dynamics together */
    double *matrixO; /* Matrix O needed for fuel slosh and hinged SP dynamics together */
    double *vectorQ; /* Vector Q needed for fuel slosh and hinged SP dynamics together */
    double *matrixT; /* Matrix T needed for fuel slosh and hinged SP dynamics together */
    double *matrixX; /* Matrix X needed for fuel slosh and hinged SP dynamics together */
    double *intermediateMatrixFuelSlosh; /* Matrix needed for fuel slosh */
    double *intermediateMatrixFuelSlosh2; /* Matrix needed for fuel slosh */
    double *intermediateVectorFuelSlosh; /* Vector needed for fuel slosh and hinged SP */
    double *intermediateVectorFuelSlosh2; /* Vector needed for fuel slosh and hinged SP */
    double tauRHS[3]; /* Right hand side of omegaDot equation */
    double ILHS[3][3]; /* Left hand side of omegaDot equaion */
    double mSC; /* Mass of the space craft including hinged rigid bodies and fuel slosh */
    double ISCPntB_B[3][3]; /* Inertia of the spacecraft about point B in B frame comp. including flexing SPs */
    double IPrimeSCPntB_B[3][3]; /* body derivative of ISCPntB_B */
    double c_B[3]; /* vector c in B frame components needed for SP dynamics */
    double cTilde_B[3][3]; /* Tilde matrix of c_B */
    double cPrime_B[3]; /* body time derivative of c_B */
    double vectorSumHingeDynamics[3];           /* intermediate variables */
    double vectorSum2HingeDynamics[3];          /* intermediate variables */
    double vectorSum3FuelSloshDynamics[3];      /* intermediate variables */
    double vectorSum4FuelSloshDynamics[3];      /* intermediate variables */
    double vectorSum5FuelSloshDynamics[3];      /* intermediate variables */
    double matrixSumFuelSloshDynamics[3][3];    /* intermediate variables */

    //! Populate variable size matrices and arrays
    matrixA = new double[this->numHRB*this->numHRB];
    matrixE = new double[this->numHRB*this->numHRB];
    matrixF = new double[this->numHRB*3];
    vectorV = new double[this->numHRB];
    matrixR = new double[3*this->numHRB];
    matrixG = new double[this->numHRB*this->numFSP];
    matrixN = new double[this->numFSP*this->numFSP];
    matrixO = new double[this->numFSP*3];
    vectorQ = new double[this->numFSP*3];
    matrixT = new double[this->numFSP*this->numFSP];
    matrixX = new double[3*this->numFSP];
    intermediateMatrixFuelSlosh = new double[this->numFSP*3];
    intermediateMatrixFuelSlosh2 = new double[this->numHRB*3];
    intermediateVectorFuelSlosh = new double[this->numFSP];
    intermediateVectorFuelSlosh2 = new double[this->numHRB];

    //! Begin method steps
    //! - Set local state variables based on the input state
    i = 0;
    /* translate state vector */
    if(this->useTranslation){
        r_BN_NLoc[0] = X[i++];
        r_BN_NLoc[1] = X[i++];
        r_BN_NLoc[2] = X[i++];
        v_BN_NLoc[0] = X[i++];
        v_BN_NLoc[1] = X[i++];
        v_BN_NLoc[2] = X[i++];
        
        mSetIdentity(BN, 3, 3);
    }
    if(this->useRotation){
        //! - Extract state information into respective variables
        sigma_BNLoc[0] = X[i++];
        sigma_BNLoc[1] = X[i++];
        sigma_BNLoc[2] = X[i++];
        omega_BN_BLoc[0] = X[i++];
        omega_BN_BLoc[1] = X[i++];
        omega_BN_BLoc[2] = X[i++];
        Omegas = NULL;
        thetasSP = NULL;
        thetaDotsSP = NULL;
        thetaDDotsHRB = NULL;
        if (this->RWACount > 0)
        {
            Omegas = &X[i];
        }
        if (this->numHRB > 0) {
            if (!this->useTranslation) {
                std::cerr << "WARNING: Cannot have hinged rigid body hinged dynamics w/o translation" << std::endl;
            }
            thetasSP = &X[i + this->RWACount + this->numRWJitter];
            thetaDotsSP = &X[i + this->RWACount + this->numRWJitter + this->numHRB];
            thetaDDotsHRB = dX + i + this->RWACount + this->numRWJitter + this->numHRB;
        }
        if (this->numFSP > 0) {
            if (!this->useTranslation) {
                std::cerr << "WARNING: Cannot have fuel slosh dynamics w/o translation" << std::endl;
            }
            rhosFS = &X[i + this->RWACount + this->numRWJitter + this->numHRB*2];
            rhoDotsFS = &X[i + this->RWACount + this->numRWJitter + this->numHRB*2 + this->numFSP];
            rhoDDotsFS = dX + i + this->RWACount + this->numRWJitter + this->numHRB*2 + this->numFSP;
        }
        omegaDot_BN_B = dX + 3 + this->useTranslation*6;
        
        //! - Convert the current attitude to DCM for conversion in DynEffector loop
        MRP2C(sigma_BNLoc, BN);
    }

    /* zero the derivative vector */
    memset(dX, 0x0, this->NStates*sizeof(double));

    //! - Set the current composite mass properties for later use in file
    computeCompositeProperties();
    
    //! - Zero the conservative acceleration
    memset(ConservAccelBdy, 0x0, 3*sizeof(double));
    
    //! - Zero the non-conservative accel
    memset(NonConservAccelBdy, 0x0, 3*sizeof(double));

    /* loop over the body effectors to call their individual ComputeDynamics() function.
       This avoids this function being called twice below */
    for(effectorIt = this->bodyEffectors.begin(); effectorIt != this->bodyEffectors.end(); effectorIt++)
    {
        DynEffector *bodyEffector = *effectorIt;
        bodyEffector->ComputeDynamics(&MassProps, &StateCurrent, t);
    }


    if (this->useTranslation){
        //! - compute inertial velocity
        v3Copy(v_BN_NLoc, dX);

        if (this->useGravity){

            double g_N[3];
            computeGravity(t, r_BN_NLoc, BN, g_N);
            
            m33MultV3(BN, g_N, g_B);
            v3Add(g_B, ConservAccelBdy, ConservAccelBdy); // Accumulating the conservative acceleration in the body frame
            v3Add(g_N, dX+3, dX+3);
        } else {
            v3SetZero(g_B);
        }
        std::vector<ReactionWheelDynamics *>::iterator RWPackIt;
        if (this->useRotation){
            
            std::vector<ReactionWheelConfigData *>::iterator rwIt;
            for (rwIt = reactWheels.begin();
                 rwIt != reactWheels.end(); rwIt++)
            {
                // convert RW jitter force from B to N frame */
                m33tMultV3(BN, (*rwIt)->F_B, rwF_N);
                // add RW jitter force to net inertial acceleration
                v3Scale(1.0/this->compMass, rwF_N, rwA_N);
                v3Add(dX+3, rwA_N, dX+3);
            }
            
        }

        //! - Loop over the vector of body Effectors and compute net force given in inertial N frame
        //! - Scale the force/torque by the mass properties inverse to get accels
        for(effectorIt = this->bodyEffectors.begin(); effectorIt != this->bodyEffectors.end(); effectorIt++)
        {
            DynEffector *bodyEffector = *effectorIt;
            if(this->useTranslation){
                v3Scale(1.0/this->compMass, bodyEffector->GetBodyForces_N(), LocalAccels_N);
                v3Add(dX + 3, LocalAccels_N, dX + 3);
            }
        }

    }

    if(this->useRotation){
        //! - Zero the external torque
        v3SetZero(extSumTorque_B);

        //! - compute dsigma/dt (see Schaub and Junkins)
        BmatMRP(sigma_BNLoc, B);
        m33Scale(0.25, B, B);
        m33MultV3(B, omega_BN_BLoc, dX + this->useTranslation*6);

        //! - Copy out the current state for DynEffector calls
        memcpy(StateCurrent.r_BN_N, r_BN_NLoc, 3*sizeof(double));
        memcpy(StateCurrent.v_BN_N, v_BN_NLoc, 3*sizeof(double));
        memcpy(StateCurrent.sigma_BN, sigma_BNLoc, 3*sizeof(double));
        memcpy(StateCurrent.omega_BN_B, omega_BN_BLoc, 3*sizeof(double));
        memcpy(StateCurrent.dcm_BS, this->T_str2Bdy, 9*sizeof(double));
        
        //! - Copy out the current mass properties for DynEffector calls
        MassProps.Mass = this->compMass;
        memcpy(MassProps.CoM, this->compCoM, 3*sizeof(double));
        memcpy(MassProps.InertiaTensor, this->compI, 9*sizeof(double));
        memcpy(MassProps.T_str2Bdy, this->T_str2Bdy, 9*sizeof(double));
        
        //! - Loop over the vector of thrusters and compute body force/torque
        //! - Convert the B-frame body forces to inertial frame for inclusion in dynamics
        //! - Scale the force/torque by the mass properties inverse to get accels
        std::vector<ThrusterDynamics*>::iterator it;
        for(it = thrusters.begin(); it != thrusters.end(); it++)
        {
            ThrusterDynamics *TheEff = *it;
            TheEff->ComputeDynamics(&MassProps, &StateCurrent, t);
            if(this->useTranslation){
                v3Scale(1.0/this->compMass, TheEff->GetBodyForces_B(), LocalAccels_B);
                m33tMultV3(BN, LocalAccels_B, LocalAccels_N);
                v3Add(dX + 3, LocalAccels_N, dX + 3);
            }
            v3Add(extSumTorque_B, TheEff->GetBodyTorquesPntB_B(), extSumTorque_B);
        }

        //! - Loop over the vector of body Effectors and compute net force/torque
        //! - Convert the B-frame body forces to inertial frame for inclusion in dynamics
        //! - Scale the force/torque by the mass properties inverse to get accels
        for(effectorIt = this->bodyEffectors.begin(); effectorIt != this->bodyEffectors.end(); effectorIt++)
        {
            DynEffector *bodyEffector = *effectorIt;
            if(this->useTranslation){
                v3Scale(1.0/this->compMass, bodyEffector->GetBodyForces_B(), LocalAccels_B);
                m33tMultV3(BN, LocalAccels_B, LocalAccels_N);
                v3Add(dX + 3, LocalAccels_N, dX + 3);
            }
            v3Add(extSumTorque_B, bodyEffector->GetBodyTorquesPntB_B(), extSumTorque_B);
        }

        //! - Define some necessary tilde matrices
        v3Tilde(omega_BN_BLoc, omegaTilde_BN_B);

        //! - Copy computed inertia into ISCPntB_B
        m33Copy(this->compI, ISCPntB_B);

        /* This section is situations when the translational motion is coupled with the rotational motion
         see Allard, Diaz, and Schaub (flex and slosh paper) for details */
        uint32_t hrbCount = 0;
        std::vector<HingedRigidBodies *>::iterator HRBPackIt;
        std::vector<HingedRigidBodyConfigData>::iterator HRBIt;
        uint32_t fspCount = 0;
        std::vector<FuelTank *>::iterator itFT;
        std::vector<FuelSloshParticleConfigData>::iterator FSPIt;
        if (this->useTranslation) {
            //! - Copy acceleration center of mass acceleration
            v3Copy(dX + 3, rDDot_CN_N);

            //! Rotate rDDot_CN_N into the body frame
            m33MultV3(BN, rDDot_CN_N, rDDot_CN_B);

            //! - Add in changes to mass properties
            v3SetZero(c_B);
            v3SetZero(cPrime_B);
            m33SetZero(IPrimeSCPntB_B);
            mSC = this->compMass;
            //! - Loop through hinged rigid bodies
            for (HRBPackIt = hingedRigidBodies.begin(); HRBPackIt != hingedRigidBodies.end(); HRBPackIt++)
            {
                for (HRBIt = (*HRBPackIt)->hingedRigidBodyData.begin();
                     HRBIt != (*HRBPackIt)->hingedRigidBodyData.end(); HRBIt++)
                {
                    //! - Define tilde matrix of r_HB_B
                    v3Tilde(HRBIt->r_HB_B, HRBIt->rTilde_HB_B);

                    //! - Define DCM from hinge to S
                    Mi(thetasSP[hrbCount], 2, HRBIt->SH);

                    //! - Define DCM from body to S
                    mMultM(HRBIt->SH, 3, 3, HRBIt->HB, 3, 3, HRBIt->SB);

                    //! - Define unit direction vectors
                    v3Copy(HRBIt->SB[0], HRBIt->sHat1_B);
                    v3Copy(HRBIt->SB[1], HRBIt->sHat2_B);
                    v3Copy(HRBIt->SB[2], HRBIt->sHat3_B);
                    //! - Find center of mass of sc with respect to B (is scaled by 1/mSC outside of loop)

                    v3Scale(-HRBIt->d, HRBIt->sHat1_B, intermediateVector);
                    v3Add(HRBIt->r_HB_B, intermediateVector, HRBIt->r_SB_B);
                    v3Scale(HRBIt->mass, HRBIt->r_SB_B, intermediateVector);
                    v3Add(intermediateVector, c_B, c_B);
                    mSC += HRBIt->mass;

                    //! - Define tilde matrix of r_SB_B
                    v3Tilde(HRBIt->r_SB_B, HRBIt->rTilde_SB_B);

                    //! - Find body derivative of c_B (is scaled by 1/mSC outside of loop)
                    v3Scale(HRBIt->mass*HRBIt->d*thetaDotsSP[hrbCount], HRBIt->sHat3_B, intermediateVector);
                    v3Add(intermediateVector, cPrime_B, cPrime_B);

                    //! - Find omega_BN in S frame components
                    m33MultV3(HRBIt->SB, omega_BN_BLoc, HRBIt->omega_BN_S);

                    //! - Find inertia of the spacecraft including hinged rigid bodies
                    mMultM(HRBIt->IPntS_S, 3, 3, HRBIt->SB, 3, 3, intermediateMatrix);
                    m33tMultM33(HRBIt->SB, intermediateMatrix, intermediateMatrix);
                    m33MultM33t(HRBIt->rTilde_SB_B, HRBIt->rTilde_SB_B, intermediateMatrix2);
                    m33Scale(HRBIt->mass, intermediateMatrix2, intermediateMatrix2);
                    m33Add(intermediateMatrix2, intermediateMatrix, intermediateMatrix);
                    m33Add(ISCPntB_B, intermediateMatrix, ISCPntB_B);

                    //! - Find body derivative of ISCPntB_B
                    v3OuterProduct(HRBIt->sHat1_B, HRBIt->sHat3_B, intermediateMatrix);
                    v3OuterProduct(HRBIt->sHat3_B, HRBIt->sHat1_B, intermediateMatrix2);
                    m33Add(intermediateMatrix, intermediateMatrix2, intermediateMatrix);
                    m33Scale(thetaDotsSP[hrbCount]*(HRBIt->IPntS_S[8] - HRBIt->IPntS_S[0]), intermediateMatrix, intermediateMatrix);
                    m33Add(IPrimeSCPntB_B, intermediateMatrix, IPrimeSCPntB_B);
                    v3Scale(HRBIt->d*thetaDotsSP[hrbCount], HRBIt->sHat3_B, HRBIt->rPrime_SB_B);
                    v3Tilde(HRBIt->rPrime_SB_B, HRBIt->rPrimeTilde_SB_B);
                    m33MultM33(HRBIt->rTilde_SB_B, HRBIt->rPrimeTilde_SB_B, intermediateMatrix);
                    m33Scale(HRBIt->mass, intermediateMatrix, intermediateMatrix);
                    m33Subtract(IPrimeSCPntB_B, intermediateMatrix, IPrimeSCPntB_B);
                    m33MultM33(HRBIt->rPrimeTilde_SB_B, HRBIt->rTilde_SB_B, intermediateMatrix);
                    m33Scale(HRBIt->mass, intermediateMatrix, intermediateMatrix);
                    m33Subtract(IPrimeSCPntB_B, intermediateMatrix, IPrimeSCPntB_B);
                    hrbCount++;
                }
            }

            //! - Loop through fuel slosh particles
            for (itFT = fuelTanks.begin(); itFT != fuelTanks.end(); itFT++)
            {
                FuelTank *TheEff = *itFT;
                for (FSPIt = (*itFT)->fuelSloshParticlesData.begin();
                     FSPIt != (*itFT)->fuelSloshParticlesData.end(); FSPIt++)
                {
                    //! - Add fuel slosh masses into mass of the spacecraft
                    mSC += FSPIt->massFSP;

                    //! - Define position vector from point B to center of mass of slosh particle
                    v3Add(FSPIt->r_PT_B, TheEff->fuelTankData.r_TB_B, FSPIt->r_PB_B);
                    v3Scale(rhosFS[fspCount], FSPIt->pHat_B, intermediateVector);
                    v3Add(FSPIt->r_PB_B, intermediateVector, FSPIt->r_PcB_B);

                    //! - Define body derivative of r_PcB_B, rPrime_PcB_B
                    v3Scale(rhoDotsFS[fspCount], FSPIt->pHat_B, FSPIt->rPrime_PcB_B);

                    //! - add to c_B (still to be scaled by 1/mSC outside of loop)
                    v3Scale(FSPIt->massFSP, FSPIt->r_PcB_B, intermediateVector);
                    v3Add(c_B, intermediateVector, c_B);

                    //! - add to cPrime_B (still to be scaled by 1/mSC outside of loop)
                    v3Scale(FSPIt->massFSP, FSPIt->rPrime_PcB_B, intermediateVector);
                    v3Add(cPrime_B, intermediateVector, cPrime_B);

                    //! - add to inertia of the spacecraft to include the fuel slosh particles
                    //! - Define tilde matrix of r_PcB_B
                    v3Tilde(FSPIt->r_PcB_B, FSPIt->rTilde_PcB_B);
                    m33MultM33t(FSPIt->rTilde_PcB_B, FSPIt->rTilde_PcB_B, intermediateMatrix);
                    m33Scale(FSPIt->massFSP, intermediateMatrix, intermediateMatrix);
                    m33Add(ISCPntB_B, intermediateMatrix, ISCPntB_B);

                    //! - add to body derivative of inertia of the spacecraft to include the fuel slosh particles
                    //! - Define tilde matrix of rPrime_PcB_B
                    v3Tilde(FSPIt->rPrime_PcB_B, FSPIt->rPrimeTilde_PcB_B);
                    m33MultM33(FSPIt->rPrimeTilde_PcB_B, FSPIt->rTilde_PcB_B, intermediateMatrix);
                    m33MultM33(FSPIt->rTilde_PcB_B, FSPIt->rPrimeTilde_PcB_B, intermediateMatrix2);
                    m33Add(intermediateMatrix, intermediateMatrix2, intermediateMatrix);
                    m33Scale(FSPIt->massFSP, intermediateMatrix, intermediateMatrix);
                    m33Subtract(IPrimeSCPntB_B, intermediateMatrix, IPrimeSCPntB_B);
                    fspCount++;
                }
            }
            //! - Scale c_B and cPrime_B by 1/mSC
            v3Scale(1/mSC, c_B, c_B);
            v3Scale(1/mSC, cPrime_B, cPrime_B);

            //! - Find torque due to gravity about point B
            v3Cross(c_B, g_B, intermediateVector);
            v3Scale(mSC, intermediateVector, gravityTorquePntB_B);
            v3Add(extSumTorque_B, gravityTorquePntB_B, extSumTorque_B);

            //! - Define necessary tilde matrix
            v3Tilde(c_B, cTilde_B);

            //! - Define matrices needed for hinged and fuel slosh dynamics
            uint32_t hrbCounti = 0;
            uint32_t hrbCountk;
            uint32_t fspCountj = 0;
            v3SetZero(vectorSumHingeDynamics);
            v3SetZero(vectorSum2HingeDynamics);
            std::vector<HingedRigidBodyConfigData>::iterator HRBIti;
            std::vector<HingedRigidBodies *>::iterator HRBPackIti;
            std::vector<HingedRigidBodyConfigData>::iterator HRBItk;
            std::vector<HingedRigidBodies *>::iterator HRBPackItk;
            std::vector<FuelTank *>::iterator itFTj;
            std::vector<FuelSloshParticleConfigData>::iterator FSPItj;
            for (HRBPackIti = hingedRigidBodies.begin(); HRBPackIti != hingedRigidBodies.end(); HRBPackIti++)
            {
                for (HRBIti = (*HRBPackIti)->hingedRigidBodyData.begin();
                     HRBIti != (*HRBPackIti)->hingedRigidBodyData.end(); HRBIti++)
                {
                    fspCountj = 0;
                    for (itFTj = fuelTanks.begin(); itFTj != fuelTanks.end(); itFTj++)
                    {
                        for (FSPItj = (*itFTj)->fuelSloshParticlesData.begin();
                             FSPItj != (*itFTj)->fuelSloshParticlesData.end(); FSPItj++)
                        {
                            //! - Populate G matrix
                            v3Scale(FSPItj->massFSP, FSPItj->pHat_B, intermediateVector);
                            v3Scale(HRBIti->mass*HRBIti->d/mSC, HRBIti->sHat3_B, intermediateVector2);
                            matrixG[hrbCounti*this->numFSP + fspCountj] = v3Dot(intermediateVector, intermediateVector2);
                            fspCountj++;
                        }
                    }
                    hrbCountk = 0;
                    for (HRBPackItk = hingedRigidBodies.begin(); HRBPackItk != hingedRigidBodies.end(); HRBPackItk++)
                    {
                        for (HRBItk = (*HRBPackItk)->hingedRigidBodyData.begin();
                             HRBItk != (*HRBPackItk)->hingedRigidBodyData.end(); HRBItk++)
                        {
                            if (hrbCounti != hrbCountk) {
                                // - Define off diagonal elements of A
                                matrixA[hrbCounti*this->numHRB+hrbCountk] = - HRBIti->mass*HRBIti->d*HRBItk->mass*HRBItk->d/mSC*v3Dot(HRBIti->sHat3_B, HRBItk->sHat3_B);

                                // - Define vector that is needed for v vector
                                v3Scale(HRBItk->mass*HRBItk->d*thetaDotsSP[hrbCountk]*thetaDotsSP[hrbCountk], HRBItk->sHat1_B, intermediateVector);
                                v3Add(vectorSumHingeDynamics, intermediateVector, vectorSumHingeDynamics);
                            }
                            hrbCountk++;

                        }

                    }
                    //! - Define diagonal elements of A matrix
                    matrixA[hrbCounti*this->numHRB + hrbCounti] = HRBIti->IPntS_S[4] + (HRBIti->mass - HRBIti->mass*HRBIti->mass/mSC)*HRBIti->d*HRBIti->d;

                    //! - Define F matrix
                    m33Subtract(cTilde_B, HRBIti->rTilde_HB_B, intermediateMatrix);
                    v3tMultM33(HRBIti->sHat3_B, intermediateMatrix, intermediateVector);
                    v3Scale(HRBIti->mass*HRBIti->d, intermediateVector, intermediateVector);
                    v3Scale(HRBIti->IPntS_S[4] + HRBIti->mass*HRBIti->d*HRBIti->d, HRBIti->sHat2_B, intermediateVector2);
                    v3Add(intermediateVector, intermediateVector2, intermediateVector);
                    v3Scale(-1.0, intermediateVector, &matrixF[hrbCounti*3]);

                    //! - Define v vector
                    v3Scale(HRBIti->mass, g_B, intermediateVector);
                    v3Cross(HRBIti->sHat1_B, intermediateVector, intermediateVector);
                    v3Scale(-HRBIti->d, intermediateVector, gravityTorqueHRBPntH_B);
                    v3Scale(1.0/mSC, vectorSumHingeDynamics, vectorSumHingeDynamics);
                    m33MultV3(omegaTilde_BN_B, cPrime_B, intermediateVector);
                    v3Scale(2.0, intermediateVector, intermediateVector);
                    v3Subtract(rDDot_CN_B, intermediateVector, intermediateVector);
                    v3Subtract(c_B, HRBIti->r_HB_B, intermediateVector2);
                    m33MultV3(omegaTilde_BN_B, intermediateVector2, intermediateVector2);
                    m33MultV3(omegaTilde_BN_B, intermediateVector2, intermediateVector2);
                    v3Subtract(intermediateVector, intermediateVector2, intermediateVector);
                    v3Subtract(intermediateVector, vectorSumHingeDynamics, intermediateVector);
                    vectorV[hrbCounti] = - HRBIti->k*thetasSP[hrbCounti] - HRBIti->c*thetaDotsSP[hrbCounti] + v3Dot(HRBIti->sHat2_B, gravityTorqueHRBPntH_B)+ (HRBIti->IPntS_S[8] - HRBIti->IPntS_S[0] + HRBIti->mass*HRBIti->d*HRBIti->d)*HRBIti->omega_BN_S[2]*HRBIti->omega_BN_S[0] - HRBIti->mass*HRBIti->d*v3Dot(HRBIti->sHat3_B, intermediateVector);

                    //! - Define R matrix
                    m33Subtract(HRBIti->rTilde_SB_B, cTilde_B, intermediateMatrix);
                    m33MultV3(intermediateMatrix, HRBIti->sHat3_B, intermediateVector);
                    v3Scale(HRBIti->mass*HRBIti->d, intermediateVector, intermediateVector);
                    v3Scale(HRBIti->IPntS_S[4], HRBIti->sHat2_B, intermediateVector2);
                    v3Add(intermediateVector, intermediateVector2, &matrixR[hrbCounti*3]);

                    //! - Define a vector that will be used for tauRHS
                    m33MultV3(HRBIti->rTilde_SB_B, HRBIti->sHat3_B, intermediateVector);
                    v3Scale(HRBIti->mass*HRBIti->d, intermediateVector, intermediateVector);
                    v3Scale(HRBIti->IPntS_S[4], HRBIti->sHat2_B, intermediateVector2);
                    v3Add(intermediateVector, intermediateVector2, intermediateVector);
                    m33MultV3(omegaTilde_BN_B, intermediateVector, intermediateVector);
                    v3Scale(thetaDotsSP[hrbCounti], intermediateVector, intermediateVector);
                    m33Subtract(HRBIti->rTilde_SB_B, cTilde_B, intermediateMatrix);
                    m33MultV3(intermediateMatrix, HRBIti->sHat1_B, intermediateVector2);
                    v3Scale(HRBIti->mass*HRBIti->d*thetaDotsSP[hrbCounti]*thetaDotsSP[hrbCounti], intermediateVector2, intermediateVector2);
                    v3Add(intermediateVector, intermediateVector2, intermediateVector);
                    v3Add(intermediateVector, vectorSum2HingeDynamics, vectorSum2HingeDynamics);
                    hrbCounti++;
                }
            }

            //! - Find inverse of A which is the E matrix
            if (this->numHRB > 0) {
                //! - Find E matrix for hinged hinged rigid body dynamics
                if (this->numHRB == 1) {
                    matrixE[0] = 1.0/matrixA[0];
                }
                else {
                    mInverse(matrixA, this->numHRB, matrixE);
                }
            }

            //! - Find more matrices needed for hinged dynamics and fuel slosh
            fspCountj = 0;
            uint32_t fspCountl;
            std::vector<FuelTank *>::iterator itFTl;
            std::vector<FuelSloshParticleConfigData>::iterator FSPItl;
            v3SetZero(vectorSum5FuelSloshDynamics);
            for (itFTj = fuelTanks.begin(); itFTj != fuelTanks.end(); itFTj++)
            {
                for (FSPItj = (*itFTj)->fuelSloshParticlesData.begin();
                     FSPItj != (*itFTj)->fuelSloshParticlesData.end(); FSPItj++)
                {
                    fspCountl = 0;
                    for (itFTl = fuelTanks.begin(); itFTl != fuelTanks.end(); itFTl++)
                    {
                        for (FSPItl = (*itFTl)->fuelSloshParticlesData.begin();
                             FSPItl != (*itFTl)->fuelSloshParticlesData.end(); FSPItl++)
                        {
                            v3SetZero(vectorSum3FuelSloshDynamics);
                            hrbCounti = 0;
                            for (HRBPackIti = hingedRigidBodies.begin(); HRBPackIti != hingedRigidBodies.end(); HRBPackIti++)
                            {
                                for (HRBIti = (*HRBPackIti)->hingedRigidBodyData.begin();
                                     HRBIti != (*HRBPackIti)->hingedRigidBodyData.end(); HRBIti++)
                                {
                                    variableSumFuelSloshDynamics = 0;
                                    hrbCountk = 0;
                                    for (HRBPackItk = hingedRigidBodies.begin(); HRBPackItk != hingedRigidBodies.end(); HRBPackItk++)
                                    {
                                        for (HRBItk = (*HRBPackItk)->hingedRigidBodyData.begin();
                                             HRBItk != (*HRBPackItk)->hingedRigidBodyData.end(); HRBItk++)
                                        {
                                            //! - Value needed for off diagonal elements of N matrix
                                            variableSumFuelSloshDynamics += matrixE[hrbCounti*this->numHRB + hrbCountk]*matrixG[hrbCountk*this->numFSP + fspCountl];
                                            hrbCountk++;
                                        }
                                    }
                                    //! - Vector needed for off diagonal elements of N matrix
                                    v3Scale(HRBIti->mass*HRBIti->d*variableSumFuelSloshDynamics, HRBIti->sHat3_B, intermediateVector2);
                                    v3Add(vectorSum3FuelSloshDynamics, intermediateVector2, vectorSum3FuelSloshDynamics);
                                    hrbCounti++;
                                }
                            }
                            if (fspCountj != fspCountl){
                                //! - Populate off diagonal elements of N matrix
                                v3Scale(FSPItl->massFSP, FSPItl->pHat_B, intermediateVector);
                                if (this->numHRB > 0) {
                                    v3Add(vectorSum3FuelSloshDynamics, intermediateVector, intermediateVector);
                                }
                                matrixN[fspCountj*this->numFSP + fspCountl] = -FSPItj->massFSP/mSC*v3Dot(FSPItj->pHat_B, intermediateVector);
                            }
                            fspCountl++;
                        }
                    }
                    v3SetZero(vectorSum3FuelSloshDynamics);
                    v3SetZero(vectorSum4FuelSloshDynamics);
                    m33SetZero(matrixSumFuelSloshDynamics);
                    hrbCounti = 0;
                    for (HRBPackIti = hingedRigidBodies.begin(); HRBPackIti != hingedRigidBodies.end(); HRBPackIti++)
                    {
                        for (HRBIti = (*HRBPackIti)->hingedRigidBodyData.begin();
                             HRBIti != (*HRBPackIti)->hingedRigidBodyData.end(); HRBIti++)
                        {
                            variableSumFuelSloshDynamics = 0;
                            hrbCountk = 0;
                            for (HRBPackItk = hingedRigidBodies.begin(); HRBPackItk != hingedRigidBodies.end(); HRBPackItk++)
                            {
                                for (HRBItk = (*HRBPackItk)->hingedRigidBodyData.begin();
                                     HRBItk != (*HRBPackItk)->hingedRigidBodyData.end(); HRBItk++)
                                {
                                    //! - Value needed for diagonal elements of N matrix
                                    variableSumFuelSloshDynamics += matrixE[hrbCounti*this->numHRB + hrbCountk]*matrixG[hrbCountk*this->numFSP + fspCountj];
                                    hrbCountk++;
                                }
                            }
                            //! - Vector needed for diagonal elements of N matrix
                            v3Scale(HRBIti->mass*HRBIti->d*variableSumFuelSloshDynamics, HRBIti->sHat3_B, intermediateVector2);
                            v3Add(vectorSum3FuelSloshDynamics, intermediateVector2, vectorSum3FuelSloshDynamics);

                            //! - Matrix needed for O matrix
                            vtMultM(&matrixE[hrbCounti*this->numHRB], matrixF, this->numHRB, 3, intermediateVector2);
                            v3OuterProduct(HRBIti->sHat3_B, intermediateVector2, intermediateMatrix);
                            m33Scale(HRBIti->mass*HRBIti->d/mSC, intermediateMatrix, intermediateMatrix);
                            m33Add(matrixSumFuelSloshDynamics, intermediateMatrix, matrixSumFuelSloshDynamics);

                            //! - Vector needed for q vector
                            v3Scale(vDot(&matrixE[hrbCounti*this->numHRB], this->numHRB, vectorV), HRBIti->sHat3_B, intermediateVector);
                            v3Scale(thetaDotsSP[hrbCounti]*thetaDotsSP[hrbCounti], HRBIti->sHat1_B, intermediateVector2);
                            v3Add(intermediateVector, intermediateVector2, intermediateVector);
                            v3Scale(HRBIti->mass*HRBIti->d/mSC, intermediateVector, intermediateVector);
                            v3Add(vectorSum4FuelSloshDynamics, intermediateVector, vectorSum4FuelSloshDynamics);

                            hrbCounti++;
                        }
                    }
                    //! - Populate diagonal elements of N matrix
                    matrixN[fspCountj*this->numFSP + fspCountj] = FSPItj->massFSP - FSPItj->massFSP*FSPItj->massFSP/mSC - FSPItj->massFSP/mSC*v3Dot(FSPItj->pHat_B, vectorSum3FuelSloshDynamics);

                    //! - Populate O matrix
                    m33Subtract(cTilde_B, FSPItj->rTilde_PcB_B, intermediateMatrix);
                    if (this->numHRB > 0) {
                        m33Subtract(intermediateMatrix, matrixSumFuelSloshDynamics, intermediateMatrix);
                    }
                    v3tMultM33(FSPItj->pHat_B, intermediateMatrix, intermediateVector);
                    v3Scale(-FSPItj->massFSP, intermediateVector, &matrixO[fspCountj*3]);

                    //! - Populate q vector
                    v3Subtract(FSPItj->r_PcB_B, c_B, intermediateVector);
                    m33MultV3(omegaTilde_BN_B, intermediateVector, intermediateVector);
                    m33MultV3(omegaTilde_BN_B, intermediateVector, intermediateVector);
                    v3Add(rDDot_CN_B, intermediateVector, intermediateVector);
                    v3Subtract(FSPItj->rPrime_PcB_B, cPrime_B, intermediateVector2);
                    m33MultV3(omegaTilde_BN_B, intermediateVector2, intermediateVector2);
                    v3Scale(2, intermediateVector2, intermediateVector2);
                    v3Add(intermediateVector, intermediateVector2, intermediateVector);
                    v3Subtract(intermediateVector, g_B, intermediateVector);
                    if (this->numHRB > 0) {
                        v3Subtract(intermediateVector, vectorSum4FuelSloshDynamics, intermediateVector);
                    }
                    vectorQ[fspCountj] = -FSPItj->massFSP*v3Dot(FSPItj->pHat_B, intermediateVector) - FSPItj->k*rhosFS[fspCountj] - FSPItj->c*rhoDotsFS[fspCountj];

                    //! - Populate X matrix
                    m33Subtract(FSPItj->rTilde_PcB_B, cTilde_B, intermediateMatrix);
                    m33MultV3(intermediateMatrix, FSPItj->pHat_B, intermediateVector);
                    v3Scale(FSPItj->massFSP, intermediateVector, &matrixX[fspCountj*3]);

                    //! - Find vector needed for tauRHS
                    m33MultV3(FSPItj->rTilde_PcB_B, FSPItj->rPrime_PcB_B, intermediateVector);
                    m33MultV3(omegaTilde_BN_B, intermediateVector, intermediateVector);
                    v3Scale(FSPItj->massFSP, intermediateVector, intermediateVector);
                    v3Add(vectorSum5FuelSloshDynamics, intermediateVector, vectorSum5FuelSloshDynamics);
                    fspCountj++;
                }
            }
            //! - Find the inverse of the N matrix which is the T matrix
            if (this->numFSP > 0) {
                //! - Find T matrix for hinged hinged rigid body dynamics
                if (this->numFSP == 1) {
                    matrixT[0] = 1.0/matrixN[0];
                }
                else {
                    mInverse(matrixN, this->numFSP, matrixT);
                }
            }
        }

        /*! - compute domega/dt (see Schaub and Junkins) When the spacecraft is a rigid body with no reaction wheels, the equation is: I*omega = -omegaTilde*I*omega + L */
        /*! - Populate inertia for LHS of the equation (this is done with intention of adding RWs/Flex/Slosh */
        m33Copy(ISCPntB_B, ILHS);

        //! - Populate the RHS of the equation
        m33MultV3(ISCPntB_B, omega_BN_BLoc, intermediateVector);
        m33MultV3(omegaTilde_BN_B, intermediateVector, intermediateVector);
        v3Subtract(extSumTorque_B, intermediateVector, tauRHS);

        if (this->useTranslation) {
            if (this->numHRB > 0 || this->numFSP > 0) {
                //! - Modify ILHS
                m33MultM33(cTilde_B, cTilde_B, intermediateMatrix);
                m33Scale(mSC, intermediateMatrix, intermediateMatrix);
                m33Add(ILHS, intermediateMatrix, ILHS);

                //! - Modify tauRHS
                m33MultV3(omegaTilde_BN_B, cPrime_B, intermediateVector);
                v3Scale(2.0, intermediateVector, intermediateVector);
                v3Subtract(rDDot_CN_B, intermediateVector, intermediateVector);
                m33MultV3(omegaTilde_BN_B, c_B, intermediateVector2);
                m33MultV3(omegaTilde_BN_B, intermediateVector2, intermediateVector2);
                v3Subtract(intermediateVector, intermediateVector2, intermediateVector);
                m33MultV3(cTilde_B, intermediateVector, intermediateVector);
                v3Scale(mSC, intermediateVector, intermediateVector);
                v3Subtract(tauRHS, intermediateVector, tauRHS);
                m33MultV3(IPrimeSCPntB_B, omega_BN_BLoc, intermediateVector);
                v3Subtract(tauRHS, intermediateVector, tauRHS);
            }
            if (this->numHRB > 0) {
                //! - Modify tauRHS with vector calculated in hinged rigid body loop
                v3Subtract(tauRHS, vectorSum2HingeDynamics, tauRHS);
                hrbCount = 0;
                for (HRBPackIt = hingedRigidBodies.begin(); HRBPackIt != hingedRigidBodies.end(); HRBPackIt++)
                {
                    for (HRBIt = (*HRBPackIt)->hingedRigidBodyData.begin();
                         HRBIt != (*HRBPackIt)->hingedRigidBodyData.end(); HRBIt++)
                    {
                        //! - Modify ILHS with hinged dynamics
                        vtMultM(&matrixE[hrbCount*this->numHRB], matrixF, this->numHRB, 3, intermediateVector);
                        v3OuterProduct(&matrixR[hrbCount*3], intermediateVector, intermediateMatrix);
                        m33Add(ILHS, intermediateMatrix, ILHS);

                        //! - Modify tauRHS with hinged dynamics
                        v3Scale(vDot(&matrixE[hrbCount*this->numHRB], this->numHRB, vectorV), &matrixR[hrbCount*3], intermediateVector);
                        v3Subtract(tauRHS, intermediateVector, tauRHS);

                        if (this->numFSP) {
                            //! - Modify ILHS with cross coupling of hinged and fuel slosh dynamics
                            mMultM(matrixT, this->numFSP, this->numFSP, matrixO, this->numFSP, 3, intermediateMatrixFuelSlosh);
                            mMultM(matrixG, this->numHRB, this->numFSP, intermediateMatrixFuelSlosh, this->numFSP, 3, intermediateMatrixFuelSlosh2);
                            vtMultM(&matrixE[hrbCount*this->numHRB], intermediateMatrixFuelSlosh2, this->numHRB, 3, intermediateVector);
                            v3OuterProduct(&matrixR[hrbCount*3], intermediateVector, intermediateMatrix);
                            m33Add(ILHS, intermediateMatrix, ILHS);

                            //! - Modify tauRHS with cross coupling of hinged and fuel slosh dynamics
                            mMultV(matrixT, this->numFSP, this->numFSP, vectorQ, intermediateVectorFuelSlosh);
                            mMultV(matrixG, this->numHRB, this->numFSP, intermediateVectorFuelSlosh, intermediateVectorFuelSlosh2);
                            v3Scale(vDot(&matrixE[hrbCount*this->numHRB], this->numHRB, intermediateVectorFuelSlosh2), &matrixR[hrbCount*3], intermediateVector);
                            v3Subtract(tauRHS, intermediateVector, tauRHS);
                        }
                        hrbCount++;
                    }
                }
            }

            if (this->numFSP > 0) {
                //! - Modify tauRHS with vector calculated in fuel slosh loop
                v3Subtract(tauRHS, vectorSum5FuelSloshDynamics, tauRHS);
                fspCount = 0;
                for (itFT = fuelTanks.begin(); itFT != fuelTanks.end(); itFT++)
                {
                    for (FSPIt = (*itFT)->fuelSloshParticlesData.begin();
                         FSPIt != (*itFT)->fuelSloshParticlesData.end(); FSPIt++)
                    {
                        //! - Modify ILHS with fuel slosh motion
                        vtMultM(&matrixT[fspCount*this->numFSP], matrixO, this->numFSP, 3, intermediateVector);
                        v3OuterProduct(&matrixX[fspCount*3], intermediateVector, intermediateMatrix);
                        m33Add(ILHS, intermediateMatrix, ILHS);

                        //! - Modify tauRHS with fuel slosh motion
                        v3Scale(vDot(&matrixT[fspCount*this->numFSP], this->numFSP, vectorQ), &matrixX[fspCount*3], intermediateVector);
                        v3Subtract(tauRHS, intermediateVector, tauRHS);
                        fspCount++;
                    }
                }
            }
        }

        //! - Modify RHS and LHS of the equation with RWs
        uint32_t rwCount = 0;
        double rwTorque[3];
        double rwSumTorque[3];
        v3SetZero(rwSumTorque);
        v3SetZero(this->rwaGyroTorqueBdy);
        
        std::vector<ReactionWheelConfigData *>::iterator rwIt;
        for (rwIt = reactWheels.begin(); rwIt != reactWheels.end(); rwIt++)
        {
            //! - Modify inertia matrix of LHS of the omegaDot equation
            v3OuterProduct((*rwIt)->gsHat_B, (*rwIt)->gsHat_B, intermediateMatrix);
            m33Scale((*rwIt)->Js, intermediateMatrix, intermediateMatrix);
            m33Subtract(ILHS, intermediateMatrix, ILHS);
            /*! - Define hs of wheels (this is not the hs seen in Schaubs book) the inertia is being modified for the LHS of the equation which results in hs being modified in this way see Allard, Schaub (flex paper) for details */
            double hs = (*rwIt)->Js * Omegas[rwCount];
            
            //! - calculate RW gyro torque - omegaTilde*Gs*hs
            v3Scale(hs, (*rwIt)->gsHat_B, intermediateVector);
            v3Add(this->rwaGyroTorqueBdy, intermediateVector, this->rwaGyroTorqueBdy); /* omegaTilde is premultiplied outside of the loop */
            v3Scale((*rwIt)->u_current, (*rwIt)->gsHat_B, rwTorque);
            v3Subtract(rwSumTorque, rwTorque, rwSumTorque);         /* subtract [Gs]u */
            v3Add(rwSumTorque, (*rwIt)->tau_B, rwSumTorque);           /* add simple jitter torque */
            rwCount++;
        }
        

        //! - Complete rwaGyroTorque
        m33MultV3(omegaTilde_BN_B, this->rwaGyroTorqueBdy, this->rwaGyroTorqueBdy);
        v3Subtract(tauRHS, this->rwaGyroTorqueBdy, tauRHS);
        v3Add(tauRHS, rwSumTorque, tauRHS);  /* add RWs torque to RHS of equation */

        //! - Solve for d(w)/dt
        m33Inverse(ILHS, intermediateMatrix);
        m33MultV3(intermediateMatrix, tauRHS, omegaDot_BN_B);            /* d(w)/dt = [I_RW]^-1 . (RHS) */

        if (this->useTranslation) {
            //! - Back solve for the fuel slosh motion
            v3SetZero(vectorSum3FuelSloshDynamics);
            fspCount = 0;
            for (itFT = fuelTanks.begin(); itFT != fuelTanks.end(); itFT++)
            {
                for (FSPIt = (*itFT)->fuelSloshParticlesData.begin();
                     FSPIt != (*itFT)->fuelSloshParticlesData.end(); FSPIt++)
                {
                    //! - Set trivial derivative rhoDot = rhoDot
                    dX[this->useTranslation*6 + this->useRotation*6 + this->RWACount + this->numRWJitter + 2*this->numHRB + fspCount] = rhoDotsFS[fspCount];

                    //! - Solve for rhoDDot
                    vtMultM(&matrixT[fspCount*this->numFSP], matrixO, this->numFSP, 3, intermediateVector);
                    rhoDDotsFS[fspCount] = v3Dot(intermediateVector, omegaDot_BN_B) + vDot(&matrixT[fspCount*this->numFSP], this->numFSP, vectorQ);

                    //! - Solve for vector needed for translation
                    v3Scale(FSPIt->massFSP*rhoDDotsFS[fspCount]/mSC, FSPIt->pHat_B, intermediateVector);
                    v3Add(vectorSum3FuelSloshDynamics, intermediateVector, vectorSum3FuelSloshDynamics);

                    fspCount++;
                }
            }

            //! - Back solve for hinged rigid body motion
            v3SetZero(vectorSumHingeDynamics);
            v3SetZero(vectorSum2HingeDynamics);
            hrbCount = 0;
            for (HRBPackIt = hingedRigidBodies.begin(); HRBPackIt != hingedRigidBodies.end(); HRBPackIt++)
            {
                for (HRBIt = (*HRBPackIt)->hingedRigidBodyData.begin();
                     HRBIt != (*HRBPackIt)->hingedRigidBodyData.end(); HRBIt++)
                {
                    //! - Set trivial derivative thetaDot = thetaDot
                    dX[this->useTranslation*6 + this->useRotation*6 + this->RWACount + this->numRWJitter + hrbCount] = thetaDotsSP[hrbCount];

                    //! - Solve for thetaDDot
                    vtMultM(&matrixE[hrbCount*this->numHRB], matrixF, this->numHRB, 3, intermediateVector);
                    thetaDDotsHRB[hrbCount] = v3Dot(intermediateVector, omegaDot_BN_B) + vDot(&matrixE[hrbCount*this->numHRB], this->numHRB, vectorV);

                    //! - Add in fuel slosh motion into thetaDDot
                    if (this->numFSP > 0) {
                        mMultV(matrixG, this->numHRB, this->numFSP, rhoDDotsFS, intermediateVectorFuelSlosh2);
                        thetaDDotsHRB[hrbCount] += vDot(&matrixE[hrbCount*this->numHRB], this->numHRB, intermediateVectorFuelSlosh2);
                    }

                    //! - Solve for two vectors needed for translation
                    v3Scale(HRBIt->mass*HRBIt->d*thetaDDotsHRB[hrbCount]/mSC, HRBIt->sHat3_B, intermediateVector);
                    v3Add(intermediateVector, vectorSumHingeDynamics, vectorSumHingeDynamics);
                    v3Scale(HRBIt->mass*HRBIt->d*thetaDotsSP[hrbCount]*thetaDotsSP[hrbCount]/mSC, HRBIt->sHat1_B, intermediateVector);
                    v3Add(intermediateVector, vectorSum2HingeDynamics, vectorSum2HingeDynamics);
                    hrbCount++;
                }
            }

            //! - Back solve to find translational acceleration
            if (this->numHRB > 0 | this->numFSP > 0) {
                m33MultV3(omegaTilde_BN_B, cPrime_B, intermediateVector);
                v3Scale(2.0, intermediateVector, intermediateVector);
                v3Subtract(rDDot_CN_B, intermediateVector, rDDot_BN_B);
                m33MultV3(omegaTilde_BN_B, c_B, intermediateVector);
                m33MultV3(omegaTilde_BN_B, intermediateVector, intermediateVector);
                v3Subtract(rDDot_BN_B, intermediateVector, rDDot_BN_B);
                m33MultV3(cTilde_B, omegaDot_BN_B, intermediateVector);
                v3Add(rDDot_BN_B, intermediateVector, rDDot_BN_B);
                if (this->numHRB > 0) {
                    v3Subtract(rDDot_BN_B, vectorSumHingeDynamics, rDDot_BN_B);
                    v3Subtract(rDDot_BN_B, vectorSum2HingeDynamics, rDDot_BN_B);
                }
                if (this->numFSP > 0) {
                    v3Subtract(rDDot_BN_B, vectorSum3FuelSloshDynamics, rDDot_BN_B);
                }
                m33tMultV3(BN, rDDot_BN_B, dX + 3);
            }
        }

        /* RW motor torque equations to solve for d(Omega)/dt */
        /* if RW jitter is on, solve for wheel angle derivative */
        rwCount = 0;
        uint32_t rwJitterCount = 0;

        for (rwIt = reactWheels.begin(); rwIt != reactWheels.end(); rwIt++)
        {
            dX[this->useTranslation*6 + this->useRotation*6 + rwCount] = (*rwIt)->u_current / (*rwIt)->Js
            - v3Dot((*rwIt)->gsHat_B, omegaDot_BN_B);
            
            if ((*rwIt)->usingRWJitter) {
                /* compute d(theta)/dt for the RW angle */
                dX[this->useTranslation*6 + this->useRotation*6 + this->RWACount + rwJitterCount] = Omegas[rwCount];
                rwJitterCount++;
            }
            rwCount++;
        }
    }
    
    // Compute the total non-conservative acceleration of point B relative to N.
    // The non-conservative acceleration is what an accelerometer placed at point B would measure.
    if (this->useTranslation)
    {
        double totalAccel_B[3];
        m33MultV3(BN, dX+3, totalAccel_B);
        v3Subtract(totalAccel_B, ConservAccelBdy, NonConservAccelBdy);
    }

    //! - delete variable size matrices
    delete [] matrixA;
    delete [] matrixE;
    delete [] matrixF;
    delete [] vectorV;
    delete [] matrixR;
    delete [] matrixG;
    delete [] matrixN;
    delete [] matrixO;
    delete [] vectorQ;
    delete [] matrixT;
    delete [] matrixX;
    delete [] intermediateMatrixFuelSlosh2;
    delete [] intermediateMatrixFuelSlosh;
    delete [] intermediateVectorFuelSlosh;
    delete [] intermediateVectorFuelSlosh2;
}

/*! This method is used to integrate the state forward to the time specified.
    It is hardcoded to perform an RK4 against the current states.  We might want 
    to think about abstracting that out to allow different integrators in the 
    future.
    @return void
    @param CurrentTime The current simulation time in seconds with double precision
*/
void SixDofEOM::integrateState(double CurrentTime)
{
    
    double  *X = new double[NStates];         /* integration state space */
    double  *Xnext = new double[NStates];        /* integration state space */
    memset(X, 0x0, NStates*sizeof(double));
    memset(Xnext, 0x0, NStates*sizeof(double));

    double timeStep;
    double sMag;
    uint32_t CentralBodyCount = 0;

    double BN[3][3];                        /* DCM from inertial to body */
    double intermediateVector[3];           /* intermediate vector needed for calculation */
    double Omega;                           /* current wheel speeds of RWs */
    double omega_s;                         /* body rate about the g_s RW axis */
    double totScRotAngMom_B[3];             /* tot spacecraft angular momentum about its center of mass in the body frame */
    double totRwsKinEnergy;                 /* All RWs kinetic energy summed together */
    double totRwsRelAngMomentum_B[3];          /* All RWs angular momentum */
    double prevTotScRotEnergy;              /* The last kinetic energy calculation from time step before */
    double *attStates;                      /* pointer to the attitude state set in the overall state matrix */
    double rDot_BN_B[3];                    /* inertial derivative of position vector from N to B in the B frame */
    double totRotFuelSloshEnergy;           /* All slosh particle's translational, rotational and potential energy */
    double totRotFuelSloshAngMomentum_B[3]; /* All slosh particle's total angular momentum in the body frame */
    double SH[3][3];                        /* DCM from hinge frame to hinged rigid body frame for individual hinged rigid bodies */
    double SB[3][3];                        /* DCM from body frame to hinged rigid body frame for individual hinged rigid bodies */
    double sHat1_B[3];                      /* Unit vector for first axis of S frame in the body frame */
    double sHat2_B[3];                      /* Unit vector for second axis of S frame in the body frame */
    double sHat3_B[3];                      /* Unit vector for third axis of S frame in the body frame */
    double omega_BN_S[3];                   /* angular velocity of B w/ respect to N in the S frame */
    double omega_SN_S[3];                   /* angular velocity of S w/ respect to N in the S frame */
    double omega_SN_B[3];                   /* angular velocity of S w/ respect to N in the body frame */
    double r_ScH_B[3];                      /* position vector from H to Sc in the body frame */
    double rDot_ScB_B[3];                   /* inertial time derivative of position vector from B to Sc in B frame */
    double rDot_ScC_B[3];                   /* inertial time derivative of position vector from C to Sc in B frame */
    double totRotHingedRigidBodyEnergy;          /* All hinged rigid bodies translational, rotational and potential energy */
    double totRotHingedRigidBodyAngMomentum_B[3]; /* All hinged rigid bodies angular momentum in the body frame */
    double rDot_PcB_B[3];                   /* Inertial derivative of position vector from B to Pc in the body frame */
    double rDot_PcC_B[3];                   /* Inertial derivative of position vector from C to Pc in the body frame */
    double intermediateVector2[3];          /* intermediate vector */
    double rBN_B[3];                        /* position vector from N to B in the body frame */
    double c_B[3];                          /* center of mass of the spacecraft with respect to point B in the B frame components */
    double cPrime_B[3];                     /* body time derivative of c_B */
    double cDot_B[3];                       /* inertial time derivative of c_B */
    double mSC;                             /* total mass of the spacecraft */

    //! Begin method steps
    //! - Get the dt from the previous time to the current
    timeStep = CurrentTime - this->timePrev;
    
    //! - Initialize the local states and invert the inertia tensor
    memcpy(X, this->XState, this->NStates*sizeof(double));
    
    //! - Loop through gravitational bodies and find the central body to integrate around
    //GravityBodyData *CentralBody = NULL;
    std::vector<GravityBodyData>::iterator it;
    for(it = gravData.begin(); it != gravData.end(); it++)
    {
        it->ephIntTime = it->ephemTimeSimNanos * NANO2SEC;
        if(it->IsCentralBody)
        {
            this->centralBody = &(*it);
            CentralBodyCount++;
        }
    }
    
    //! - If we did not locate a central body, alert the user and prepare to explode.
    if(CentralBodyCount != 1)
    {
        std::cerr << "ERROR: I got a bad count on central bodies: " <<
        CentralBodyCount <<std::endl;
        return;
    }
    
    Integrator->integrate(CurrentTime, timeStep, X, Xnext, this->NStates);

    memcpy(this->XState, Xnext, this->NStates*sizeof(double));

    // Manuel really dislikes this part of the code and thinks we should rethink it. It's not clean whatsoever
    // The goal of the snippet is to compute the nonConservative delta v (LocalDV)

    if (this->useTranslation) {
        // The nonconservative delta v is computed assuming that the conservative acceleration is constant along a time step. Then: DV_cons = Cons_accel * dt. DV_noncons = DV_tot - DV_cons
        double DVtot[3];
        double DVconservative[3];
        double BNLoc[3][3];
        double BNLocnext[3][3];
        double* sigma_BN;
        double* sigma_BNnext;
        double v_B[3];
        double v_Bnext[3];
        double localDV[3];
        
        if (this->useRotation) {
            sigma_BN = X+6;
            sigma_BNnext = Xnext+6;
            
            MRP2C(sigma_BN, BNLoc);
            MRP2C(sigma_BNnext, BNLocnext);
        }
        else {
            mSetIdentity(BNLoc, 3, 3);
            mSetIdentity(BNLocnext, 3, 3);
        }
        
        
        m33MultV3(BNLocnext, X + 3, v_B);
        m33MultV3(BNLocnext, Xnext + 3, v_Bnext);
        
        v3Subtract(v_Bnext, v_B, DVtot);
        v3Scale(timeStep, ConservAccelBdy, DVconservative);
        v3Subtract(DVtot, DVconservative, localDV);
        
        v3Add(localDV, this->AccumDVBdy, this->AccumDVBdy);

        //! - Find translational energy and potential energy
        this->totScOrbitalEnergy = 1.0/2.0*this->compMass*v3Dot(&this->XState[3], &this->XState[3]);
        if (this->useGravity) {
            this->totScOrbitalEnergy -= this->compMass*this->centralBody->mu/v3Norm(&this->XState[0]);
        }

        //! - Find orbital angular momentum
        v3Cross(&this->XState[0], &this->XState[3], intermediateVector);
        v3Scale(this->compMass, intermediateVector, this->totScOrbitalAngMom_N);
        this->totScOrbitalAngMomMag = v3Norm(this->totScOrbitalAngMom_N);
    }
    
    //-------------------------------------------------------

    //! - MRPs get singular at 360 degrees.  If we are greater than 180, switch to shadow set
    if (this->useRotation){
        attStates = &this->XState[this->useTranslation*6];
        sMag =  v3Norm(attStates);
        if(sMag > 1.0) {
            v3Scale(-1.0 / sMag / sMag, attStates, attStates);
            this->MRPSwitchCount++;
        }


        /*! - Energy, Power, and angular Momentum Calculations,
            T = 1/2*omega^T*I*omega + sum_over_RWs(1/2*Js*(Omega_i)^2) - Schaub pg. 4.5.1, 
            H = I*omega + sum_over_RWs(g_s*J_s(omega_si + Omega)) - Schaub 4.5.1, 
            P = sum_over_external_torque(omega^T*L) + sum_over_RWs(u*Omega) - Schaub 4.5.2 */
        totRwsKinEnergy         = 0.0;
        this->scRotPower        = 0.0;
        v3SetZero(totRwsRelAngMomentum_B);
        v3SetZero(totScRotAngMom_B);
        v3SetZero(this->totScRotAngMom_N);

        //! Loop through Thrusters to get power
        std::vector<ThrusterDynamics*>::iterator itThruster;
        ThrusterDynamics *theEff;
        for(itThruster = thrusters.begin(); itThruster != thrusters.end(); itThruster++)
        {
            theEff = *itThruster;
            this->scRotPower += v3Dot(&attStates[3], theEff->GetBodyTorquesPntB_B()); /* omega^T*L */
        }
        
        //! - Loop through RWs to get energy, momentum and power information
        uint32_t rwCount = 0;
        uint32_t rwJitterCount = 0;

        std::vector<ReactionWheelConfigData *>::iterator rwIt;
        for (rwIt = reactWheels.begin(); rwIt != reactWheels.end(); rwIt++)
        {
            /* Gather values needed for energy and momentum calculations */
            Omega = this->XState[this->useTranslation*6 + this->useRotation*6 + rwCount];
            omega_s = v3Dot(&attStates[3], (*rwIt)->gsHat_B);
            
            /* RW energy */
            totRwsKinEnergy += 0.5*((*rwIt)->Js)*(Omega)*(Omega); /* 1/2*Js*(Omega_i)^2 */
            v3Scale((*rwIt)->Js, (*rwIt)->gsHat_B, intermediateVector);
            
            /* RW power */
            this->scRotPower += (*rwIt)->u_current*Omega; /* u*Omega */
            
            /* RW angular momentum */
            v3Scale(Omega, intermediateVector, intermediateVector);
            v3Add(totRwsRelAngMomentum_B, intermediateVector, totRwsRelAngMomentum_B);
            
            /* Set current reaction wheel speed and angle */
            (*rwIt)->Omega = Omega;
            if ((*rwIt)->usingRWJitter) {
                (*rwIt)->theta = this->XState[useTranslation*6 + useRotation*6 + this->RWACount + rwJitterCount];
                rwJitterCount++;
            }
            rwCount++;
        }

        //! - Define parameters needed for energy and momentum calculations
        MRP2C(&attStates[0], BN);
        m33MultV3(BN, &this->XState[3], rDot_BN_B);
        m33MultV3(BN, &this->XState[0], rBN_B);

        mSC = this->compMass;
        v3SetZero(c_B);
        v3SetZero(cPrime_B);
        v3SetZero(cDot_B);
        std::vector<HingedRigidBodies *>::iterator HRBPackIt;
        std::vector<HingedRigidBodyConfigData>::iterator HRBIt;
        uint32_t hrbCount = 0;
        for (HRBPackIt = hingedRigidBodies.begin(); HRBPackIt != hingedRigidBodies.end(); HRBPackIt++)
        {
            for (HRBIt = (*HRBPackIt)->hingedRigidBodyData.begin();
                 HRBIt != (*HRBPackIt)->hingedRigidBodyData.end(); HRBIt++)
            {
                //! - Copy out state vector for theta and thetaDot into hinged rigid body information
                HRBIt->theta = this->XState[this->useTranslation*6 + this->useRotation*6 + this->RWACount + this->numRWJitter + hrbCount];
                HRBIt->thetaDot = this->XState[this->useTranslation*6 + this->useRotation*6 + this->RWACount + this->numRWJitter + this->numHRB + hrbCount];

                //! - Find contributions of hinged rigid body information for c_B and cPrime_B for energy and momentum calculations
                mSC += HRBIt->mass;

                //! - Define DCM from hinge to S
                Mi(HRBIt->theta, 2, SH);

                //! - Define DCM from body to S
                mMultM(SH, 3, 3, HRBIt->HB, 3, 3, SB);

                //! - Define unit direction vectors
                v3Copy(SB[0], sHat1_B);
                v3Copy(SB[1], sHat2_B);
                v3Copy(SB[2], sHat3_B);

                //! - Find center of mass of sc with respect to B (is scaled by 1/mSC outside of loop)
                v3Scale(-HRBIt->d, sHat1_B, intermediateVector);
                v3Add(HRBIt->r_HB_B, intermediateVector, intermediateVector);
                v3Scale(HRBIt->mass, intermediateVector, intermediateVector);
                v3Add(intermediateVector, c_B, c_B);

                //! - Find body derivative of c_B (is scaled by 1/mSC outside of loop)
                v3Scale(HRBIt->mass*HRBIt->d*HRBIt->thetaDot, sHat3_B, intermediateVector);
                v3Add(intermediateVector, cPrime_B, cPrime_B);
                hrbCount++;
            }
        }

        uint32_t fspCount = 0;
        std::vector<FuelTank *>::iterator itFT;
        std::vector<FuelSloshParticleConfigData>::iterator FSPIt;
        for (itFT = fuelTanks.begin(); itFT != fuelTanks.end(); itFT++)
        {
            for (FSPIt = (*itFT)->fuelSloshParticlesData.begin();
                 FSPIt != (*itFT)->fuelSloshParticlesData.end(); FSPIt++)
            {
                //! - Copy out state vector for rho and rhoDot into fuel slosh information
                FSPIt->rho = this->XState[this->useTranslation*6 + this->useRotation*6 + this->RWACount + this->numRWJitter + 2*this->numHRB + fspCount];
                FSPIt->rhoDot = this->XState[this->useTranslation*6 + this->useRotation*6 + this->RWACount + this->numRWJitter + 2*this->numHRB + this->numFSP + fspCount];

                //! - Find contributions of hinged rigid body information for c_B and cPrime_B for energy and momentum calculations
                //! - Add fuel slosh masses into mass of the spacecraft
                mSC += FSPIt->massFSP;

                //! - add in contribution to c_B
                v3Scale(FSPIt->rho, FSPIt->pHat_B, intermediateVector);
                v3Add(FSPIt->r_PB_B, intermediateVector, intermediateVector);
                v3Scale(FSPIt->massFSP, intermediateVector, intermediateVector);
                v3Add(c_B, intermediateVector, c_B);

                //! - add to cPrime_B
                v3Scale(FSPIt->rhoDot, FSPIt->pHat_B, intermediateVector);
                v3Scale(FSPIt->massFSP, intermediateVector, intermediateVector);
                v3Add(cPrime_B, intermediateVector, cPrime_B);
                fspCount++;
            }
        }
        //! - Scale c_B and cPrime_B by 1/mSC
        v3Scale(1/mSC, c_B, c_B);
        v3Scale(1/mSC, cPrime_B, cPrime_B);

        //! - Find cDot_B
        v3Cross(&attStates[3], c_B, intermediateVector);
        v3Add(cPrime_B, intermediateVector, cDot_B);

        //! - Add in contribution of cDot_B to orbital energy
        this->totScOrbitalEnergy += mSC*v3Dot(rDot_BN_B, cDot_B) + 1.0/2.0*mSC*v3Dot(cDot_B, cDot_B);

        //! - Add in contribution of c_B and cDot_B to orbital angular momentum
        v3Cross(rBN_B, cDot_B, intermediateVector);
        v3Cross(c_B, rDot_BN_B, intermediateVector2);
        v3Add(intermediateVector, intermediateVector2, intermediateVector);
        v3Cross(c_B, cDot_B, intermediateVector2);
        v3Add(intermediateVector, intermediateVector2, intermediateVector);
        v3Scale(mSC, intermediateVector, intermediateVector);
        m33tMultV3(BN, intermediateVector, intermediateVector);
        v3Add(this->totScOrbitalAngMom_N, intermediateVector, this->totScOrbitalAngMom_N);

        totRotHingedRigidBodyEnergy = 0;
        v3SetZero(totRotHingedRigidBodyAngMomentum_B);
        hrbCount = 0;
        for (HRBPackIt = hingedRigidBodies.begin(); HRBPackIt != hingedRigidBodies.end(); HRBPackIt++)
        {
            for (HRBIt = (*HRBPackIt)->hingedRigidBodyData.begin();
                 HRBIt != (*HRBPackIt)->hingedRigidBodyData.end(); HRBIt++)
            {
                //! - Calculate energy for hinged rigid bodies
                //! - Define DCM from hinge to S
                Mi(HRBIt->theta, 2, SH);
                //! - Define DCM from body to S
                mMultM(SH, 3, 3, HRBIt->HB, 3, 3, SB);
                //! - Define unit direction vectors
                v3Copy(SB[0], sHat1_B);
                v3Copy(SB[1], sHat2_B);
                //! - map omega_BN_B to the S frame
                m33MultV3(SB, &attStates[3], omega_BN_S);
                v3Copy(omega_BN_S, omega_SN_S);
                //! - Find omega_SN_S
                omega_SN_S[1] += HRBIt->thetaDot;
                //! - Find rotational energy about Sc of hinged rigid bodies
                mMultV(HRBIt->IPntS_S, 3, 3, omega_SN_S, intermediateVector);
                totRotHingedRigidBodyEnergy += 1.0/2.0*v3Dot(omega_SN_S, intermediateVector);

                //! - Find rotational energy about C of the hinged rigid bodies
                //! - map omega_SN_S to the body frame
                m33tMultV3(SB, omega_SN_S, omega_SN_B);
                v3Scale(-HRBIt->d, sHat1_B, r_ScH_B);
                v3Cross(omega_SN_B, r_ScH_B, intermediateVector);
                v3Cross(&attStates[3], HRBIt->r_HB_B, intermediateVector2);
                v3Add(intermediateVector, intermediateVector2, rDot_ScB_B);
                v3Subtract(rDot_ScB_B, cDot_B, rDot_ScC_B);
                totRotHingedRigidBodyEnergy += 1.0/2.0*HRBIt->mass*v3Dot(rDot_ScC_B, rDot_ScC_B);
                //! - Potential energy of hinged rigid bodies
                totRotHingedRigidBodyEnergy += 1.0/2.0*HRBIt->k*HRBIt->theta*HRBIt->theta;

                //! - Find contribution from hinged rigid bodies to orbital energy
                this->totScOrbitalEnergy += 1.0/2.0*HRBIt->mass*v3Dot(rDot_BN_B, rDot_BN_B);

                //! - Find angular momentum of hinged rigid bodies about point C of the spacecraft
                mMultV(HRBIt->IPntS_S, 3, 3, omega_SN_S, intermediateVector);
                m33tMultV3(SB, intermediateVector, intermediateVector);
                v3Add(totRotHingedRigidBodyAngMomentum_B, intermediateVector, totRotHingedRigidBodyAngMomentum_B);
                v3Add(r_ScH_B, HRBIt->r_HB_B, intermediateVector);
                v3Subtract(intermediateVector, c_B, intermediateVector);
                v3Cross(intermediateVector, rDot_ScC_B, intermediateVector);
                v3Scale(HRBIt->mass, intermediateVector, intermediateVector);
                v3Add(totRotHingedRigidBodyAngMomentum_B, intermediateVector, totRotHingedRigidBodyAngMomentum_B);

                //! - Find contribution from hinged rigid bodies to orbital angular momentum
                v3Cross(&this->XState[0], &this->XState[3], intermediateVector);
                v3Scale(HRBIt->mass, intermediateVector, intermediateVector);
                v3Add(this->totScOrbitalAngMom_N, intermediateVector, this->totScOrbitalAngMom_N);
                hrbCount++;
            }
        }

        totRotFuelSloshEnergy = 0;
        v3SetZero(totRotFuelSloshAngMomentum_B);
        fspCount = 0;
        for (itFT = fuelTanks.begin(); itFT != fuelTanks.end(); itFT++)
        {
            for (FSPIt = (*itFT)->fuelSloshParticlesData.begin();
                 FSPIt != (*itFT)->fuelSloshParticlesData.end(); FSPIt++)
            {
                //! - Find fuel slosh rotation energy about point C
                v3Cross(&attStates[3], FSPIt->r_PB_B, rDot_PcB_B);
                v3Scale(FSPIt->rhoDot, FSPIt->pHat_B, intermediateVector);
                v3Add(rDot_PcB_B, intermediateVector, rDot_PcB_B);
                v3Scale(FSPIt->rho, FSPIt->pHat_B, intermediateVector);
                v3Cross(&attStates[3], intermediateVector, intermediateVector);
                v3Add(rDot_PcB_B, intermediateVector, rDot_PcB_B);
                v3Subtract(rDot_PcB_B, cDot_B, intermediateVector);
                totRotFuelSloshEnergy+= 1.0/2.0*FSPIt->massFSP*v3Dot(intermediateVector, intermediateVector);
                //! - Add in potential energy of the springs
                totRotFuelSloshEnergy+= 1.0/2.0*FSPIt->k*FSPIt->rho*FSPIt->rho;

                //! - Find contribution from fuel slosh to orbital energy
                this->totScOrbitalEnergy += 1.0/2.0*FSPIt->massFSP*v3Dot(rDot_BN_B, rDot_BN_B);

                //! - Find angular momentum of fuel slosh about point C
                v3Scale(FSPIt->rho, FSPIt->pHat_B, intermediateVector2);
                v3Add(FSPIt->r_PB_B, intermediateVector2, intermediateVector);
                v3Subtract(intermediateVector, c_B, intermediateVector);
                v3Subtract(rDot_PcB_B, cDot_B, rDot_PcC_B);
                v3Cross(intermediateVector, rDot_PcC_B, intermediateVector);
                v3Scale(FSPIt->massFSP, intermediateVector, intermediateVector);
                v3Add(totRotFuelSloshAngMomentum_B, intermediateVector, totRotFuelSloshAngMomentum_B);

                //! - Find contribution from fuel slosh to orbital angular momentum
                v3Cross(&this->XState[0], &this->XState[3], intermediateVector);
                v3Scale(FSPIt->massFSP, intermediateVector, intermediateVector);
                v3Add(this->totScOrbitalAngMom_N, intermediateVector, this->totScOrbitalAngMom_N);
                fspCount++;
            }
        }

        //! - Grab previous energy value for rate of change of energy
        prevTotScRotEnergy = this->totScRotEnergy;

        /* spacecraft angular momentum */
        m33MultV3(this->compI, &attStates[3], totScRotAngMom_B); /* I*omega */

        //! - 1/2*omega^T*I*omega - hubs rotational energy about its center of mass
        this->totScRotEnergy = 0.5*v3Dot(&attStates[3], totScRotAngMom_B);

        //! - Add in the hubs rotational energy about point C
        v3Scale(-1.0, cDot_B, intermediateVector);
        this->totScRotEnergy += 1.0/2.0*this->compMass*v3Dot(intermediateVector, intermediateVector);

        //! - Add in the hubs angular momentum about point C
        v3Scale(-1.0, c_B, intermediateVector);
        v3Scale(-1.0, cDot_B, intermediateVector2);
        v3Cross(intermediateVector, intermediateVector2, intermediateVector);
        v3Scale(this->compMass, intermediateVector, intermediateVector);
        v3Add(totScRotAngMom_B, intermediateVector, totScRotAngMom_B);

        //! - Add the reaction wheel, fuel slosh, and hinged dynamcis into rotational energy and momentum
        this->totScRotEnergy += totRwsKinEnergy; /* T from above */
        this->totScRotEnergy += totRotFuelSloshEnergy;
        this->totScRotEnergy += totRotHingedRigidBodyEnergy;
        v3Add(totRotFuelSloshAngMomentum_B, totScRotAngMom_B, totScRotAngMom_B);
        v3Add(totRotHingedRigidBodyAngMomentum_B, totScRotAngMom_B, totScRotAngMom_B);
        v3Add(totRwsRelAngMomentum_B, totScRotAngMom_B, totScRotAngMom_B); /* H from above */

        //! - Find angular momentum vector in inertial frame
        m33tMultV3(BN, totScRotAngMom_B, this->totScRotAngMom_N);

        //! - Find magnitude of rotational spacecraft angular momentum
        this->totScRotAngMomMag = v3Norm(this->totScRotAngMom_N);

        //! - Find magnitude of orbital spacecraft angular momentum
        this->totScOrbitalAngMomMag = v3Norm(this->totScOrbitalAngMom_N);

        //! - Calulate rate of change of energy
        this->scRotEnergyRate = (this->totScRotEnergy-prevTotScRotEnergy)/timeStep;
    }

    //! - Clear out local allocations and set time for next cycle
    this->timePrev = CurrentTime;
    delete[] X;
    delete[] Xnext;
}

/*! This method computes the output states based on the current integrated state.  
    @return void
*/
void SixDofEOM::computeOutputs()
{
    GravityBodyData *centralBody = NULL;
    GravityBodyData *displayBody = NULL;
    std::vector<GravityBodyData>::iterator it;
    double displayPos[3], displayVel[3];
    
    for(it = gravData.begin(); it != gravData.end(); it++)
    {
        if(it->IsCentralBody)
        {
            centralBody = &(*it);
        }
        if(it->IsDisplayBody)
        {
            displayBody = &(*it);
        }
    }

    if (this->useTranslation){
        memcpy(this->r_BN_N, &(this->XState[0]), 3*sizeof(double));
        memcpy(this->v_BN_N, &(this->XState[3]), 3*sizeof(double));
    } 
    if (this->useRotation){
        memcpy(this->sigma_BN, &(this->XState[this->useTranslation*6]), 3*sizeof(double));
        memcpy(this->omega_BN_B, &(this->XState[this->useTranslation*6+3]), 3*sizeof(double));
    }

    if(centralBody != NULL)
    {
        v3Add(this->r_BN_N, centralBody->PosFromEphem, this->r_BN_N);
        v3Add(this->v_BN_N, centralBody->VelFromEphem, this->v_BN_N);
    }
    
    memset(displayPos, 0x0, 3*sizeof(double));
    memset(displayVel, 0x0, 3*sizeof(double));
    if(displayBody != NULL)
    {
        v3Subtract(this->r_BN_N, displayBody->PosFromEphem, this->r_BN_N);
        v3Subtract(this->v_BN_N, displayBody->VelFromEphem, this->v_BN_N);
        v3Copy(displayBody->PosFromEphem, displayPos);
        v3Copy(displayBody->VelFromEphem, displayVel);
    }
    for(it = gravData.begin(); it != gravData.end(); it++)
    {
        v3Subtract(it->PosFromEphem, displayPos, it->posRelDisplay);
        v3Subtract(it->VelFromEphem, displayVel, it->velRelDisplay);
    }
    
}

/*! This method writes out the current state and mass properties to their
    appropriate output messages.  It creates local copies and puts that copied 
    data into the output stream.
    @return void
    @param CurrentClock The current simulation time in nanoseconds
*/
void SixDofEOM::WriteOutputMessages(uint64_t CurrentClock)
{
    SystemMessaging *messageSys = SystemMessaging::GetInstance();

    //! Begin method steps
    //! - If we have a valid state output message ID, copy over internals and write out
    if(StateOutMsgID >= 0)
    {
        OutputStateData stateOut;
        memcpy(stateOut.r_BN_N, this->r_BN_N, 3*sizeof(double));
        memcpy(stateOut.v_BN_N, this->v_BN_N, 3*sizeof(double));
        memcpy(stateOut.sigma_BN, this->sigma_BN, 3*sizeof(double));
        memcpy(stateOut.omega_BN_B, this->omega_BN_B, 3*sizeof(double));
        memcpy(stateOut.dcm_BS, this->T_str2Bdy, 9*sizeof(double));
        memcpy(stateOut.TotalAccumDVBdy, this->AccumDVBdy, 3*sizeof(double));
        stateOut.MRPSwitchCount = this->MRPSwitchCount;
        messageSys->WriteMessage(StateOutMsgID, CurrentClock,
            sizeof(OutputStateData), reinterpret_cast<uint8_t*> (&stateOut), moduleID);
    }
    
    //! - If we have a valid mass props output message ID, copy over internals and write out
    if(MassPropsMsgID >= 0)
    {
        
        MassPropsData massProps;
        massProps.Mass = this->compMass;
        memcpy(massProps.CoM, this->compCoM, 3*sizeof(double));
        memcpy(&(massProps.InertiaTensor[0]), &(this->compI[0][0]), 9*sizeof(double));
        memcpy(massProps.T_str2Bdy, this->T_str2Bdy, 9*sizeof(double));
        messageSys->WriteMessage(MassPropsMsgID, CurrentClock,
            sizeof(MassPropsData), reinterpret_cast<uint8_t*> (&massProps), moduleID);
    }
    
    SpicePlanetState tmpPlanet;
    std::vector<GravityBodyData>::iterator it;
    for(it = gravData.begin(); it != gravData.end(); it++)
    {
        // @TODO: Currently the Viz must be given the central body. For now we send the
        // below message to acheive this. However, long term the Viz will be updated
        // to be able to read the spice ephemeris and dynamically resolve the
        // display frame and reference frames and choose any gravity body as the
        // central display body.
        if (it->IsCentralBody)
        {
            v3Copy(it->posRelDisplay, tmpPlanet.PositionVector);
            v3Copy(it->velRelDisplay, tmpPlanet.VelocityVector);
            tmpPlanet.J2000Current = it->ephemTime;
            memset(tmpPlanet.PlanetName, 0x0, MAX_BODY_NAME_LENGTH*sizeof(char));
            memcpy(tmpPlanet.PlanetName, it->planetEphemName.c_str(),
                   it->planetEphemName.size()*sizeof(char));
            messageSys->WriteMessage(this->centralBodyOutMsgId, CurrentClock, sizeof(SpicePlanetState), reinterpret_cast<uint8_t*> (&tmpPlanet), moduleID);
        }
        
        if(it->outputMsgID < 0)
        {
            continue;
        }
        
        v3Copy(it->posRelDisplay, tmpPlanet.PositionVector);
        v3Copy(it->velRelDisplay, tmpPlanet.VelocityVector);
        tmpPlanet.J2000Current = it->ephemTime;
        memset(tmpPlanet.PlanetName, 0x0, MAX_BODY_NAME_LENGTH*sizeof(char));
        memcpy(tmpPlanet.PlanetName, it->planetEphemName.c_str(),
               it->planetEphemName.size()*sizeof(char));
        messageSys->WriteMessage(it->outputMsgID, CurrentClock, sizeof(SpicePlanetState),
            reinterpret_cast<uint8_t*> (&tmpPlanet), moduleID);
    }
}

/*! This method is the main entry point for dynamics.  It reads the inputs, 
    propagates the state, and then computes/writes the output messages.  Note that 
    the integration call converts the input time to seconds as a double precision 
    number
    @return void
    @param CurrentSimNanos The current simulation time in nanoseconds
*/
void SixDofEOM::UpdateState(uint64_t CurrentSimNanos)
{
    ReadInputs();
    integrateState(CurrentSimNanos*NANO2SEC);
    computeOutputs();
    WriteOutputMessages(CurrentSimNanos);
}
