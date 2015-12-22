#include "dynamics/SixDofEOM/six_dof_eom.h"
#include "environment/spice/spice_interface.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include <cstring>
#include <iostream>
#include <math.h>
#include "architecture/messaging/system_messaging.h"

/*! This is the constructor for SixDofEOM.  It initializes a few variables*/
SixDofEOM::SixDofEOM()
{
    CallCounts = 0;
    RWACount = 0;
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
            it->ephemTime = LocalPlanet.J2000Current;
            it->planetEphemName = LocalPlanet.PlanetName;
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
 
    /* zero the derivative vector */
    memset(dX, 0x0, NStates*sizeof(double));
    //! - Set the current composite mass properties for later use in file
    computeCompositeProperties();

    //! - compute derivative of position (velocity)
    v3Copy(v_N, dX);

    //! - Get current position magnitude and compute the 2-body gravitational accels
    rmag = v3Norm(r_N);
    v3Scale(-CentralBody->mu / rmag / rmag / rmag, r_N, d2);
    v3Add(d2, dX+3, dX+3);

    /* compute the gravitational zonal harmonics */
    if(CentralBody->UseJParams)
    {
        jPerturb(CentralBody, r_N, perturbAccel);
        v3Add(dX+3, perturbAccel, dX+3);
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
        DynEffector *TheEff = *it;
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
    m33MultV3(B, d2, d3);               /* [tilde(w)]([I]w + [Gs]hs) */
    //    for(i = 0; i < NUM_RW; i++) {
    //        hs = this->rw[i].Js * (v3Dot(omega, this->rw[i].gs) + Omega[i]);
    //        v3Scale(hs, this->rw[i].gs, d3);
    //        v3Add(d3, d2, d2);
    //    }
    m33MultV3(compIinv, d3, d2);        /* d(w)/dt = [I_RW]^-1 . (RHS) */
    v3Add(dX+9, d2, dX+9);
    
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
        it->ephIntTime = CurrentTime;
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
        CentralBodyCount;
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
    
    
    //! - MRPs get singular at 360 degrees.  If we are greater than 180, switch to shadow
    sMag =  v3Norm(&XState[6]);
    if(sMag > 1.0) {
        v3Scale(-1.0 / sMag / sMag, &this->XState[6], &this->XState[6]);
        MRPSwitchCount++;
    }
    
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
