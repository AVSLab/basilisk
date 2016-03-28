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
#include "dynamics/SixDofEOM/six_dof_eom.h"
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
	mu = 0;                      //!< [m3/s^2] central body gravitational param
	ephemTime = 0;               //!< [s]      Ephemeris time for the body in question
	ephIntTime = 0;              //!< [s]      Integration time associated with the ephem data
	radEquator = 0;              //!< [m]      Equatorial radius for the body
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
{
    CallCounts = 0;
    RWACount = reactWheels.size();
    OutputStateMessage = "inertial_state_output";
    OutputMassPropsMsg = "spacecraft_mass_props";
    OutputBufferCount = 2;
    MRPSwitchCount = 0;
    return;
}

/*! Destructor.  Nothing so far.*/
SixDofEOM::~SixDofEOM()
{
    return;
}

/*! This method exists to add a new gravitational body in to the simulation to 
    be used to effect the spacecraft dynamics.  
    @return void
    @param NewBody A pointer to the gravitational body that is being added
*/
void SixDofEOM::AddGravityBody(GravityBodyData *NewBody)
{
    GravData.push_back(*NewBody);
}

/*! This method exists to attach an effector to the vehicle's dynamics.  The 
    effector should be a derived class of the DynEffector abstract class and it 
    should include a ComputeDynamics call which is operated by dynamics.
    @return void
    @param NewEffector The effector that we are adding to dynamics
*/
void SixDofEOM::addThrusterSet(ThrusterDynamics *NewEffector)
{
    thrusters.push_back(NewEffector);
}

/*! This method exists to attach an effector to the vehicle's dynamics.  The
effector should be a derived class of the DynEffector abstract class and it
should include a ComputeDynamics call which is operated by dynamics.
@return void
@param NewEffector The effector that we are adding to dynamics
*/
void SixDofEOM::addReactionWheelSet(ReactionWheelDynamics *NewEffector)
{
	reactWheels.push_back(NewEffector);
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
    for(it = GravData.begin(); it != GravData.end(); it++)
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
	RWACount = 0;
	std::vector<ReactionWheelDynamics *>::iterator it;
	for (it = reactWheels.begin(); it != reactWheels.end(); it++)
	{
		std::vector<ReactionWheelConfigData>::iterator rwIt;
		for (rwIt = (*it)->ReactionWheelData.begin();
		rwIt != (*it)->ReactionWheelData.end(); rwIt++)
		{
			RWACount++;
		}
	}
    NStates = 12 + RWACount;
    XState = new double[NStates]; // pos/vel/att/rate + rwa omegas
    memset(XState, 0x0, (NStates)*sizeof(double));
    TimePrev = 0.0;
    
    //! - Ensure that all init states were appropriately set by the caller
    if(PositionInit.size() != 3 || VelocityInit.size() != 3 ||
       AttitudeInit.size() != 3 || AttRateInit.size() != 3 ||
       baseInertiaInit.size() != 9 || baseCoMInit.size() != 3 ||
       T_Str2BdyInit.size() != 9)
    {
        std::cerr << "Your initial conditions didn't match up with right sizes\n";
        std::cerr << "Position: "<< PositionInit.size() << std::endl;
        std::cerr << "Velocity: "<< VelocityInit.size() << std::endl;
        std::cerr << "Attitude: "<< AttitudeInit.size() << std::endl;
        std::cerr << "Att-rate: "<< AttRateInit.size() << std::endl;
        std::cerr << "Inertia: "<< baseInertiaInit.size() << std::endl;
        std::cerr << "CoM: "<< baseCoMInit.size() << std::endl;
        std::cerr << "Str2Bdy: "<< T_Str2BdyInit.size() << std::endl;
        return;
    }
    
    //! - Zero out the accumulated DV (always starts at zero)
    memset(AccumDVBdy, 0x0, 3*sizeof(double));
    
    //! - For remaining variables, grab iterators for each vector and assign internals
    std::vector<double>::iterator PosIt = PositionInit.begin();
    std::vector<double>::iterator VelIt = VelocityInit.begin();
    std::vector<double>::iterator AttIt = AttitudeInit.begin();
    std::vector<double>::iterator RateIt= AttRateInit.begin();
    std::vector<double>::iterator InertiaIt= baseInertiaInit.begin();
    std::vector<double>::iterator CoMIt= baseCoMInit.begin();
    std::vector<double>::iterator Str2BdyIt= T_Str2BdyInit.begin();
    for(uint32_t i=0; i<3; i++)
    {
        XState[i] = *PosIt++;
        XState[i+3] = *VelIt++;
        XState[i+6] = *AttIt++;
        XState[i+9] = *RateIt++;
        baseCoM[i] = *CoMIt++;
        for(uint32_t j=0; j<3; j++)
        {
            baseI[i][j] = *InertiaIt++;
            T_str2Bdy[i][j] = *Str2BdyIt++;
        }
    }

	uint32_t rwCount = 0;
	for (it=reactWheels.begin(); it != reactWheels.end(); it++)
	{
		std::vector<ReactionWheelConfigData>::iterator rwIt;
		for (rwIt = (*it)->ReactionWheelData.begin();
		  rwIt != (*it)->ReactionWheelData.end(); rwIt++)
		{
			XState[12 + rwCount] = rwIt->Omega;
			rwCount++;
		}
	}
    
    //! - Call computeOutputs to ensure that the outputs are available post-init
    computeOutputs();
    
    //! - Write output messages for other modules that use the dynamics state in cross-init
    StateOutMsgID = SystemMessaging::GetInstance()->
        CreateNewMessage(OutputStateMessage, sizeof(OutputStateData),
        OutputBufferCount, "OutputStateData", moduleID);
    
    MassPropsMsgID = SystemMessaging::GetInstance()->
        CreateNewMessage(OutputMassPropsMsg, sizeof(MassPropsData),
        OutputBufferCount, "MassPropsData", moduleID);
    
    initPlanetStateMessages();
    
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
    for(it = GravData.begin(); it != GravData.end(); it++)
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
    for(it = GravData.begin(); it != GravData.end(); it++)
    {
        if(it->BodyMsgID >= 0)
        {
            SystemMessaging::GetInstance()->ReadMessage(it->BodyMsgID, &LocalHeader,
                                                        sizeof(SpicePlanetState), reinterpret_cast<uint8_t*> (&LocalPlanet));
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
    @param perturbAccel The computed perturbation vector output from function
*/
void SixDofEOM::jPerturb(GravityBodyData *gravBody, double r_N[3],
    double perturbAccel[3])
{

    double temp[3];
    double temp2[3];
    double planetPos[3];
    
    m33MultV3(gravBody->J20002Pfix, r_N, planetPos);
    double rmag = v3Norm(planetPos);
    double x = planetPos[0];
    double y = planetPos[1];
    double z = planetPos[2];

    memset(perturbAccel, 0x0, 3*sizeof(double));
    /* Calculating the total acceleration based on user input */
    if(gravBody->JParams.size() > 0)
    {
        std::vector<double>::iterator it = gravBody->JParams.begin();
        perturbAccel[0] = 5.0*z*z/(rmag*rmag) - 1.0;
        perturbAccel[1] = perturbAccel[0];
        perturbAccel[2] = perturbAccel[0] - 2.0;
        perturbAccel[0] *= x;
        perturbAccel[1] *= y;
        perturbAccel[2] *= z;
        v3Scale(3.0/2.0*gravBody->mu * (*it) *
                gravBody->radEquator*gravBody->radEquator / (rmag*rmag*rmag*rmag*rmag), perturbAccel, perturbAccel );
    }
    if(gravBody->JParams.size()> 1)
    {
        std::vector<double>::iterator it = gravBody->JParams.begin()+1;
        v3Set(5.0 * (7.0 * pow(z / rmag, 3.0) - 3.0 * (z / rmag)) * (x / rmag),
              5.0 * (7.0 * pow(z / rmag, 3.0) - 3.0 * (z / rmag)) * (y / rmag),
              -3.0 * (10.0 * pow(z / rmag, 2.0) - (35.0 / 3.0)*pow(z / rmag, 4.0) - 1.0), temp);
        v3Scale(1.0 / 2.0 * (*it) * (gravBody->mu / pow(rmag, 2.0))*pow(gravBody->radEquator / rmag, 3.0), temp, temp2);
        v3Add(perturbAccel, temp2, perturbAccel);
    }
    if(gravBody->JParams.size()> 2) {
        std::vector<double>::iterator it = gravBody->JParams.begin()+2;
        v3Set((3.0 - 42.0 * pow(z / rmag, 2.0) + 63.0 * pow(z / rmag, 4.0)) * (x / rmag),
              (3.0 - 42.0 * pow(z / rmag, 2.0) + 63.0 * pow(z / rmag, 4.0)) * (y / rmag),
              (15.0 - 70.0 * pow(z / rmag, 2) + 63.0 * pow(z / rmag, 4.0)) * (z / rmag), temp);
        v3Scale(5.0 / 8.0 * (*it) * (gravBody->mu / pow(rmag, 2.0))*pow(gravBody->radEquator / rmag, 4.0), temp, temp2);
        v3Add(perturbAccel, temp2, perturbAccel);
    }
    if(gravBody->JParams.size()> 3) {
    std::vector<double>::iterator it = gravBody->JParams.begin()+3;
        v3Set(3.0 * (35.0 * (z / rmag) - 210.0 * pow(z / rmag, 3.0) + 231.0 * pow(z / rmag, 5.0)) * (x / rmag),
              3.0 * (35.0 * (z / rmag) - 210.0 * pow(z / rmag, 3.0) + 231.0 * pow(z / rmag, 5.0)) * (y / rmag),
              -(15.0 - 315.0 * pow(z / rmag, 2.0) + 945.0 * pow(z / rmag, 4.0) - 693.0 * pow(z / rmag, 6.0)), temp);
        v3Scale(1.0 / 8.0 * (*it) * (gravBody->mu / pow(rmag, 2.0))*pow(gravBody->radEquator / rmag, 5.0), temp, temp2);
        v3Add(perturbAccel, temp2, perturbAccel);
    }
    if(gravBody->JParams.size()> 4) {
    std::vector<double>::iterator it = gravBody->JParams.begin()+4;
        v3Set((35.0 - 945.0 * pow(z / rmag, 2) + 3465.0 * pow(z / rmag, 4.0) - 3003.0 * pow(z / rmag, 6.0)) * (x / rmag),
              (35.0 - 945.0 * pow(z / rmag, 2.0) + 3465.0 * pow(z / rmag, 4.0) - 3003.0 * pow(z / rmag, 6.0)) * (y / rmag),
              -(3003.0 * pow(z / rmag, 6.0) - 4851.0 * pow(z / rmag, 4.0) + 2205.0 * pow(z / rmag, 2.0) - 245.0) * (z / rmag), temp);
        v3Scale(-1.0 / 16.0 * (*it) * (gravBody->mu / pow(rmag, 2.0))*pow(gravBody->radEquator / rmag, 6.0), temp, temp2);
        v3Add(perturbAccel, temp2, perturbAccel);
    }
    m33tMultV3(gravBody->J20002Pfix, perturbAccel, perturbAccel);
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
    compMass = baseMass;
    v3Scale(baseMass, baseCoM, scaledCoM);

    std::vector<ThrusterDynamics*>::iterator it;
    for(it=thrusters.begin(); it != thrusters.end(); it++)
    {
        DynEffector *TheEff = *it;
        v3Scale(TheEff->objProps.Mass, TheEff->objProps.CoM, localCoM);
        v3Add(scaledCoM, localCoM, scaledCoM);
        compMass += TheEff->objProps.Mass;
    }
    
    //! - Divide summation by total mass to get center of mass.
    v3Scale(1.0/compMass, scaledCoM, compCoM);
    
    //! - Compute the parallel axis theorem effects for the base body inertia tensor
    m33SetIdentity(identMatrix);
    v3Subtract(baseCoM, compCoM, CoMDiff);
    CoMDiffNormSquare = v3Norm(CoMDiff);
    CoMDiffNormSquare *= CoMDiffNormSquare;
    m33Scale(CoMDiffNormSquare, identMatrix, diracMatrix);
    v3OuterProduct(CoMDiff, CoMDiff, outerMatrix);
    m33Subtract(diracMatrix, outerMatrix, objInertia);
    m33Add(objInertia, baseI, compI);
    
    /*! - For each child body, compute parallel axis theorem effectos for inertia tensor
     and add that overall inertia back to obtain the composite inertia tensor.*/
    for(it=thrusters.begin(); it != thrusters.end(); it++)
    {
        DynEffector *TheEff = *it;
        v3Subtract(TheEff->objProps.CoM, compCoM, CoMDiff);
        CoMDiffNormSquare = v3Norm(CoMDiff);
        CoMDiffNormSquare *= CoMDiffNormSquare;
        m33Scale(CoMDiffNormSquare, identMatrix, diracMatrix);
        v3OuterProduct(CoMDiff, CoMDiff, outerMatrix);
        m33Subtract(diracMatrix, outerMatrix, objInertia);
        m33Scale(TheEff->objProps.Mass, objInertia, objInertia);
        vAdd(TheEff->objProps.InertiaTensor, 9, objInertia, objInertia);
        m33Add(compI, objInertia, compI);
    }
    //! - Compute inertia inverse based off the computed inertia tensor
    m33Inverse(compI, compIinv);
    
}

/*! This method computes the state derivatives at runtime for the state integration.  
    It handles all gravitational effects and nominal attitude motion itself.  
    All body effectors have their current force/torque computed via the 
    ComputeDynamics call that is inherited from the DynEffector abstract class.
    @return void
    @param t The current simulation time as double precision
    @param X The current state of the spacecraft
    @param dX The computed derivatives that we output to caller
    @param CentralBody The gravitational data for the central body
*/
void SixDofEOM::equationsOfMotion(double t, double *X, double *dX,
                                  GravityBodyData *CentralBody)
{
    
    std::vector<ThrusterDynamics*>::iterator it;
    OutputStateData StateCurrent;
    MassPropsData MassProps;
    
    double r_NLoc[3];
    double rmag;
    double v_NLoc[3];
    double sigmaLoc[3];
    double omegaLoc[3];
    double B[3][3];             /* d(sigma)/dt = 1/4 B omega */
    double d2[3];               /* intermediate variables */
    double d3[3];
    double T_Irtl2Bdy[3][3];
    double LocalAccels[3];
    int    i;
    double PlanetRelPos[3];
    double PlanetAccel[3];
    double posVelComp[3];
    double perturbAccel[3];
	double *Omegas;
    
    //! Begin method steps
    
    //! - Set local state variables based on the input state
    i = 0;
    /* translate state vector */
    r_NLoc[0] = X[i++];
    r_NLoc[1] = X[i++];
    r_NLoc[2] = X[i++];
    v_NLoc[0] = X[i++];
    v_NLoc[1] = X[i++];
    v_NLoc[2] = X[i++];
    sigmaLoc[0] = X[i++];
    sigmaLoc[1] = X[i++];
    sigmaLoc[2] = X[i++];
    omegaLoc[0] = X[i++];
    omegaLoc[1] = X[i++];
    omegaLoc[2] = X[i++];
	Omegas = NULL;
	if (RWACount > 0)
	{
		Omegas = &X[i];
	}
 
    /* zero the derivative vector */
    memset(dX, 0x0, NStates*sizeof(double));
    //! - Set the current composite mass properties for later use in file
    computeCompositeProperties();

    //! - compute derivative of position (velocity)
    v3Copy(v_N, dX);

    //! - Get current position magnitude and compute the 2-body gravitational accels
    rmag = v3Norm(r_NLoc);
    v3Scale(-CentralBody->mu / rmag / rmag / rmag, r_NLoc, d2);
    v3Add(d2, dX+3, dX+3);

    /* compute the gravitational zonal harmonics or the spherical harmonics (never both)*/
    if(CentralBody->UseJParams)
    {
        jPerturb(CentralBody, r_NLoc, perturbAccel);
        v3Add(dX+3, perturbAccel, dX+3);
    }
    else if (CentralBody->UseSphericalHarmParams)
    {
        unsigned int max_degree = CentralBody->getSphericalHarmonicsModel()->getMaxDegree(); // Maximum degree to include
        double posPlanetFix[3]; // [m] Position in planet-fixed frame
        double gravField[3]; // [m/s^2] Gravity field in planet fixed frame

        double aux[3], aux1[3], aux2[3], aux3[3];
        double planetDt = t - CentralBody->ephIntTime;
        double J2000PfixCurrent[3][3];
        
        m33Scale(planetDt, CentralBody->J20002Pfix_dot, J2000PfixCurrent);
        m33Add(J2000PfixCurrent, CentralBody->J20002Pfix, J2000PfixCurrent);
        m33MultV3(J2000PfixCurrent, r_NLoc, posPlanetFix); // r_E = [EN]*r_N
        CentralBody->getSphericalHarmonicsModel()->computeField(posPlanetFix, max_degree, gravField, false);
        
        m33tMultV3(J2000PfixCurrent, gravField, aux1); // [EN]^T * gravField
        
        m33MultV3(CentralBody->J20002Pfix_dot, v_NLoc, aux2);  // [EN_dot] * v_N
        m33tMultV3(CentralBody->J20002Pfix, aux2, aux2);    // [EN]^T * [EN_dot] * v_N
        v3Scale(2.0, aux2, aux2);                           // 2 * [EN]^T * [EN_dot] * v_N
        
        m33MultV3(CentralBody->J20002Pfix_dot, r_NLoc, aux3);  // [EN_dot] * r_N
        m33tMultV3(CentralBody->J20002Pfix, aux3, aux3);    // [EN]^T * [EN_dot] * r_N
        m33MultV3(CentralBody->J20002Pfix_dot, aux3, aux3); // [EN_dot] * [EN]^T * [EN_dot] * r_N
        m33tMultV3(CentralBody->J20002Pfix, aux3, aux3);    // [EN]^T * [EN_dot] * [EN]^T * [EN_dot] * r_N
        
        v3SetZero(aux2);
        v3Subtract(aux1, aux2, aux);    // [EN]^T * gravField - 2 * [EN]^T * [EN_dot] * v_N
        
        // perturbAccel = [EN]^T * gravField - 2 * [EN]^T * [EN_dot] * v_N - [EN]^T * [EN_dot] * [EN]^T * [EN_dot] * r_N
        v3SetZero(aux3);
        v3Subtract(aux, aux3, perturbAccel);
        
        v3Add(dX+3, perturbAccel, dX+3);
        
//#ifdef _DEBUG
//        printf("Paste this into python terminal:\n");
//        printf("r_N = np.array([%.15e, %.15e, %.15e])\n", r_N[0], r_N[1], r_N[2]);
//        printf("v_N = np.array([%.15e, %.15e, %.15e])\n", v_N[0], v_N[1], v_N[2]);
//        printf("r_E = np.array([%.15e, %.15e, %.15e])\n", posPlanetFix[0], posPlanetFix[1], posPlanetFix[2]);
//        
//        printf("g_E = np.array([%.15e, %.15e, %.15e])\n", gravField[0], gravField[1], gravField[2]);
//        
//        printf("EN = np.array([");
//        for (unsigned int i = 0; i < 3; i++) {
//            printf("[%.15e, %.15e, %.15e]", CentralBody->J20002Pfix[i][0], CentralBody->J20002Pfix[i][1], CentralBody->J20002Pfix[i][2]);
//            if (i < 2)
//                printf(",");
//        }
//        printf("])\n");
//        
//        printf("EN_dot = np.array([");
//        for (unsigned int i = 0; i < 3; i++) {
//            printf("[%.15e, %.15e, %.15e]", CentralBody->J20002Pfix_dot[i][0], CentralBody->J20002Pfix_dot[i][1], CentralBody->J20002Pfix_dot[i][2]);
//            if (i < 2)
//                printf(",");
//        }
//        printf("])\n");
//        printf("g_N_computed = EN.T.dot(g_E)-2*EN.T.dot(EN_dot).dot(v_N)-EN.T.dot(EN_dot).dot(EN.T).dot(EN_dot).dot(r_N)\n");
//        printf("g_N = np.array([%.15e, %.15e, %.15e])\n", perturbAccel[0], perturbAccel[1], perturbAccel[2]);
//        printf("print g_N_computed - g_N\n");
//#endif
    }

    /*! - Zero the inertial accels and compute grav accel for all bodies other than central body.
     Gravity perturbation is acceleration on spacecraft+acceleration on central body.
     Ephemeris information is propagated by Euler's method in substeps*/
    v3SetZero(InertialAccels);
    std::vector<GravityBodyData>::iterator gravit;
    for(gravit = GravData.begin(); gravit != GravData.end(); gravit++)
    {
        if(gravit->IsCentralBody || gravit->BodyMsgID < 0)
        {
            continue;
        }
        v3Scale(t - CentralBody->ephIntTime, CentralBody->VelFromEphem,
            posVelComp);
        v3Add(r_NLoc, CentralBody->PosFromEphem, PlanetRelPos);
        v3Add(PlanetRelPos, posVelComp, PlanetRelPos);
        v3Subtract(PlanetRelPos, gravit->PosFromEphem, PlanetRelPos);
        v3Scale(t - gravit->ephIntTime, gravit->VelFromEphem, posVelComp);
        v3Subtract(PlanetRelPos, posVelComp, PlanetRelPos);
        rmag = v3Norm(PlanetRelPos);
        v3Scale(-gravit->mu / rmag / rmag / rmag, PlanetRelPos, PlanetAccel);
        v3Add(InertialAccels, PlanetAccel, InertialAccels);
        v3Scale(t - gravit->ephIntTime, gravit->VelFromEphem, posVelComp);
        v3Subtract(CentralBody->PosFromEphem, gravit->PosFromEphem, PlanetRelPos);
        v3Subtract(PlanetRelPos, posVelComp, PlanetRelPos);
        v3Scale(t - CentralBody->ephIntTime, CentralBody->VelFromEphem,
                posVelComp);
        v3Add(PlanetRelPos, posVelComp, PlanetRelPos);
        rmag = v3Norm(PlanetRelPos);
        v3Scale(gravit->mu/rmag/rmag/rmag, PlanetRelPos, PlanetAccel);
        v3Add(InertialAccels, PlanetAccel, InertialAccels);
    }
    //! - Add in inertial accelerations of the non-central bodies
    v3Add(dX+3, InertialAccels, dX+3);
    

    //! - compute dsigma/dt (see Schaub and Junkins)
    BmatMRP(sigmaLoc, B);
    m33Scale(0.25, B, B);
    m33MultV3(B, omegaLoc, dX + 6);
    

    //! - Convert the current attitude to DCM for conversion in DynEffector loop
    MRP2C(sigmaLoc, T_Irtl2Bdy);
    
    //! - Copy out the current state for DynEffector calls
    memcpy(StateCurrent.r_N, r_NLoc, 3*sizeof(double));
    memcpy(StateCurrent.v_N, v_NLoc, 3*sizeof(double));
    memcpy(StateCurrent.sigma, sigmaLoc, 3*sizeof(double));
    memcpy(StateCurrent.omega, omegaLoc, 3*sizeof(double));
    memcpy(StateCurrent.T_str2Bdy, T_str2Bdy, 9*sizeof(double));
    
    //! - Copy out the current mass properties for DynEffector calls
    MassProps.Mass = compMass;
    memcpy(MassProps.CoM, compCoM, 3*sizeof(double));
    memcpy(MassProps.InertiaTensor, compI, 9*sizeof(double));
    memcpy(MassProps.T_str2Bdy, T_str2Bdy, 9*sizeof(double));
    
    //! - Zero the non-conservative accel
    memset(NonConservAccelBdy, 0x0, 3*sizeof(double));
    //! - Loop over the vector of thrusters and compute body force/torque
    //! - Convert the body forces to inertial for inclusion in dynamics
    //! - Scale the force/torque by the mass properties inverse to get accels
    for(it=thrusters.begin(); it != thrusters.end(); it++)
    {
        ThrusterDynamics *TheEff = *it;
        TheEff->ComputeDynamics(&MassProps, &StateCurrent, t);
        v3Scale(1.0/compMass, TheEff->GetBodyForces(), LocalAccels);
        v3Add(LocalAccels, NonConservAccelBdy, NonConservAccelBdy);
        m33tMultV3(T_Irtl2Bdy, LocalAccels, LocalAccels);
        v3Add(dX+3, LocalAccels, dX+3);
        m33MultV3(compIinv, TheEff->GetBodyTorques(), LocalAccels);
        v3Add(dX+9, LocalAccels, dX+9);
        
    }

    //! - compute domega/dt (see Schaub and Junkins)
    v3Tilde(omega, B);                  /* [tilde(w)] */
    m33MultV3(compI, omega, d2);        /* [I]w */
	v3Copy(d2, d3);
 

	uint32_t rwCount = 0;
	std::vector<ReactionWheelDynamics *>::iterator RWPackIt;
	double rwTorque[3];
	double rwSumTorque[3];
	v3SetZero(rwSumTorque);
	double spinAxisBody[3];
	for (RWPackIt = reactWheels.begin(); RWPackIt != reactWheels.end(); RWPackIt++)
	{
		std::vector<ReactionWheelConfigData>::iterator rwIt;
		for (rwIt = (*RWPackIt)->ReactionWheelData.begin();
		rwIt != (*RWPackIt)->ReactionWheelData.end(); rwIt++)
		{
			m33MultV3(T_str2Bdy, rwIt->gsHat_S, spinAxisBody);
			double hs =  rwIt->Js * (v3Dot(omega, spinAxisBody) + Omegas[rwCount]);
			v3Scale(hs, spinAxisBody, d2);
			v3Add(d3, d2, d3);
			v3Scale(rwIt->u_current, spinAxisBody, rwTorque);
			v3Subtract(rwSumTorque, rwTorque, rwSumTorque);          /* subtract [Gs]u */
			rwCount++;
		}
	}
	m33MultV3(B, d3, d2);               /* [tilde(w)]([I]w + [Gs]hs) */
	v3Subtract(rwSumTorque, d2, d2);
    m33MultV3(compIinv, d2, d3);        /* d(w)/dt = [I_RW]^-1 . (RHS) */
    v3Add(dX+9, d3, dX+9);
	rwCount = 0;
	for (RWPackIt = reactWheels.begin(); RWPackIt != reactWheels.end(); RWPackIt++)
	{
		std::vector<ReactionWheelConfigData>::iterator rwIt;
		for (rwIt = (*RWPackIt)->ReactionWheelData.begin();
		rwIt != (*RWPackIt)->ReactionWheelData.end(); rwIt++)
		{
			m33MultV3(T_str2Bdy, rwIt->gsHat_S, spinAxisBody);
			dX[12 + rwCount] = rwIt->u_current / rwIt->Js
				- v3Dot(spinAxisBody, dX + 9);
			rwCount++;
		}
	}
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
    double  *X2 = new double[NStates];        /* integration state space */
    double  *k1 = new double[NStates];        /* intermediate RK results */
    double  *k2 = new double[NStates];
    double  *k3 = new double[NStates];
    double  *k4 = new double[NStates];
    memset(X, 0x0, NStates*sizeof(double));
    memset(X2, 0x0, NStates*sizeof(double));
    memset(k1, 0x0, NStates*sizeof(double));
    memset(k2, 0x0, NStates*sizeof(double));
    memset(k3, 0x0, NStates*sizeof(double));
    memset(k4, 0x0, NStates*sizeof(double));
    uint32_t i;
    double TimeStep;
    double sMag;
    uint32_t CentralBodyCount = 0;
    double LocalDV[3];
    double sigmaLoc[3];
    double BN[3][3];
    double gsHat_B[3];                       /* spin axis in body frame */
    double intermediate_vector[3];           /* intermediate vector needed for calculation */
    double omega_BLoc[3];                    /* local angular velocity vector in body frame */
    double rws_Js;                           /* Spin Axis Inertias of RWs */
    double Omegas_Loc;                       /* current wheel speeds of RWs */

    //! Begin method steps
    //! - Get the dt from the previous time to the current
    TimeStep = CurrentTime - TimePrev;
    
    //! - Initialize the local states and invert the inertia tensor
    memcpy(X, XState, NStates*sizeof(double));
    
    //! - Loop through gravitational bodies and find the central body to integrate around
    GravityBodyData *CentralBody = NULL;
    std::vector<GravityBodyData>::iterator it;
    for(it = GravData.begin(); it != GravData.end(); it++)
    {
        it->ephIntTime = it->ephemTimeSimNanos * 1.0E-9;
        if(it->IsCentralBody)
        {
            CentralBody = &(*it);
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
    
    //! - Perform RK4 steps.  Go ahead and look it up anywhere.  It's a standard thing
    equationsOfMotion(CurrentTime, X, k1, CentralBody);
    for(i = 0; i < NStates; i++) {
        X2[i] = X[i] + 0.5 * TimeStep * k1[i];
    }
    v3Scale(TimeStep/6.0, NonConservAccelBdy, LocalDV);
    v3Add(LocalDV, AccumDVBdy, AccumDVBdy);
    equationsOfMotion(CurrentTime + TimeStep * 0.5, X2, k2, CentralBody);
    for(i = 0; i < NStates; i++) {
        X2[i] = X[i] + 0.5 * TimeStep * k2[i];
    }
    v3Scale(TimeStep/3.0, NonConservAccelBdy, LocalDV);
    v3Add(LocalDV, AccumDVBdy, AccumDVBdy);
    equationsOfMotion(CurrentTime + TimeStep * 0.5, X2, k3, CentralBody);
    for(i = 0; i < NStates; i++) {
        X2[i] = X[i] + TimeStep * k3[i];
    }
    v3Scale(TimeStep/3.0, NonConservAccelBdy, LocalDV);
    v3Add(LocalDV, AccumDVBdy, AccumDVBdy);
    equationsOfMotion(CurrentTime + TimeStep, X2, k4, CentralBody);
    for(i = 0; i < NStates; i++) {
        X[i] += TimeStep / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    v3Scale(TimeStep/6.0, NonConservAccelBdy, LocalDV);
    v3Add(LocalDV, AccumDVBdy, AccumDVBdy);
    memcpy(XState, X, NStates*sizeof(double));
    
	uint32_t rwCount = 0;
    tot_rws_rel_kin_energy = 0.0;
    tot_rws_rel_ang_momentum_B[0] = 0.0;
    tot_rws_rel_ang_momentum_B[1] = 0.0;
    tot_rws_rel_ang_momentum_B[2] = 0.0;
	std::vector<ReactionWheelDynamics *>::iterator RWPackIt;
	for (RWPackIt = reactWheels.begin(); RWPackIt != reactWheels.end(); RWPackIt++)
	{
		std::vector<ReactionWheelConfigData>::iterator rwIt;
		for (rwIt = (*RWPackIt)->ReactionWheelData.begin();
		rwIt != (*RWPackIt)->ReactionWheelData.end(); rwIt++)
		{
            /* Gather values needed for energy and momentum calculations */
            rws_Js = rwIt->Js;
			m33MultV3(T_str2Bdy, rwIt->gsHat_S, gsHat_B);
            Omegas_Loc = XState[12 + rwCount];
            tot_rws_rel_kin_energy += 1.0/2.0*rws_Js*Omegas_Loc*Omegas_Loc;
            v3Scale(rws_Js, gsHat_B, intermediate_vector);
            v3Scale(Omegas_Loc, intermediate_vector, intermediate_vector);
            v3Add(tot_rws_rel_ang_momentum_B, intermediate_vector, tot_rws_rel_ang_momentum_B);
            /* Set current reaction wheel speed */
            rwIt->Omega = Omegas_Loc;
			rwCount++;
		}
	}
    
    //! - MRPs get singular at 360 degrees.  If we are greater than 180, switch to shadow
    sMag =  v3Norm(&XState[6]);
    if(sMag > 1.0) {
        v3Scale(-1.0 / sMag / sMag, &this->XState[6], &this->XState[6]);
        MRPSwitchCount++;
    }

    //! - Rotational Kinetic Energy and Momentum Calculations - this calculation assumes the spacecraft is a rigid body with RW's
    //! - Find rotational kinetic energy of spacecraft
    omega_BLoc[0] = XState[9];
    omega_BLoc[1] = XState[10];
    omega_BLoc[2] = XState[11];
    m33MultV3(compI, omega_BLoc, tot_sc_ang_momentum_B);
    tot_sc_rot_kin_energy = 1.0/2.0*v3Dot(omega_BLoc, tot_sc_ang_momentum_B);
    v3Add(tot_rws_rel_ang_momentum_B, tot_sc_ang_momentum_B, tot_sc_ang_momentum_B);
    tot_sc_ang_momentum = v3Dot(tot_sc_ang_momentum_B, tot_sc_ang_momentum_B);
    //! - Find angular momentum vector in inertial frame
    sigmaLoc[0] = XState[6];
    sigmaLoc[1] = XState[7];
    sigmaLoc[2] = XState[8];
    MRP2C(sigmaLoc, BN);
    m33tMultV3(BN, tot_sc_ang_momentum_B, tot_sc_ang_momentum_N);
    tot_sc_rot_kin_energy += tot_rws_rel_kin_energy;

    //! - Clear out local allocations and set time for next cycle
    TimePrev = CurrentTime;
    delete[] X;
    delete[] X2;
    delete[] k1;
    delete[] k2;
    delete[] k3;
    delete[] k4;
    
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
    
    for(it = GravData.begin(); it != GravData.end(); it++)
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

    memcpy(r_N, &(XState[0]), 3*sizeof(double));
    memcpy(v_N, &(XState[3]), 3*sizeof(double));
    memcpy(sigma, &(XState[6]), 3*sizeof(double));
    memcpy(omega, &(XState[9]), 3*sizeof(double));

    if(centralBody != NULL)
    {
        v3Add(r_N, centralBody->PosFromEphem, r_N);
        v3Add(v_N, centralBody->VelFromEphem, v_N);
    }
    
    memset(displayPos, 0x0, 3*sizeof(double));
    memset(displayVel, 0x0, 3*sizeof(double));
    if(displayBody != NULL)
    {
        v3Subtract(r_N, displayBody->PosFromEphem, r_N);
        v3Subtract(v_N, displayBody->VelFromEphem, v_N);
        v3Copy(displayBody->PosFromEphem, displayPos);
        v3Copy(displayBody->VelFromEphem, displayVel);
    }
    for(it = GravData.begin(); it != GravData.end(); it++)
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
    std::vector<GravityBodyData>::iterator it;
    SpicePlanetState localPlanet;
    //! Begin method steps
    //! - If we have a valid state output message ID, copy over internals and write out
    if(StateOutMsgID >= 0)
    {
        OutputStateData StateOut;
        memcpy(StateOut.r_N, r_N, 3*sizeof(double));
        memcpy(StateOut.v_N, v_N, 3*sizeof(double));
        memcpy(StateOut.sigma, sigma, 3*sizeof(double));
        memcpy(StateOut.omega, omega, 3*sizeof(double));
        memcpy(StateOut.T_str2Bdy, T_str2Bdy, 9*sizeof(double));
        memcpy(StateOut.TotalAccumDVBdy, AccumDVBdy, 3*sizeof(double));
        StateOut.MRPSwitchCount = MRPSwitchCount;
        SystemMessaging::GetInstance()->WriteMessage(StateOutMsgID, CurrentClock,
            sizeof(OutputStateData), reinterpret_cast<uint8_t*> (&StateOut), moduleID);
    }
    
    //! - If we have a valid mass props output message ID, copy over internals and write out
    if(MassPropsMsgID >= 0)
    {
        
        MassPropsData massProps;
        massProps.Mass = compMass;
        memcpy(massProps.CoM, compCoM, 3*sizeof(double));
        memcpy(&(massProps.InertiaTensor[0]), &(compI[0][0]), 9*sizeof(double));
        memcpy(massProps.T_str2Bdy, T_str2Bdy, 9*sizeof(double));
        SystemMessaging::GetInstance()->WriteMessage(MassPropsMsgID, CurrentClock,
            sizeof(MassPropsData), reinterpret_cast<uint8_t*> (&massProps), moduleID);
    }
    
    for(it = GravData.begin(); it != GravData.end(); it++)
    {
        if(it->outputMsgID < 0)
        {
            continue;
        }
        v3Copy(it->posRelDisplay, localPlanet.PositionVector);
        v3Copy(it->velRelDisplay, localPlanet.VelocityVector);
        localPlanet.J2000Current = it->ephemTime;
        memset(localPlanet.PlanetName, 0x0, MAX_BODY_NAME_LENGTH*sizeof(char));
        memcpy(localPlanet.PlanetName, it->planetEphemName.c_str(),
               it->planetEphemName.size()*sizeof(char));
        SystemMessaging::GetInstance()->
            WriteMessage(it->outputMsgID, CurrentClock, sizeof(SpicePlanetState),
            reinterpret_cast<uint8_t*> (&localPlanet), moduleID);
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
    integrateState(CurrentSimNanos*1.0E-9);
    computeOutputs();
    WriteOutputMessages(CurrentSimNanos);
}
