#include "dynamics/SixDofEOM/six_dof_eom.h"
#include "environment/spice/spice_interface.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include <cstring>
#include <iostream>
#include "architecture/messaging/system_messaging.h"

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

SixDofEOM::~SixDofEOM()
{
   return;
}

void SixDofEOM::AddGravityBody(GravityBodyData *NewBody) 
{
   GravData.push_back(*NewBody);
}

void SixDofEOM::AddBodyEffector(DynEffector *NewEffector) 
{
   BodyEffectors.push_back(NewEffector);
}

void SixDofEOM::SelfInit()
{
   NStates = 12 + RWACount;
   XState = new double[NStates]; // pos/vel/att/rate + rwa omegas
   memset(XState, 0x0, (NStates)*sizeof(double));
   TimePrev = 0;
   mass = MassInit;

   if(PositionInit.size() != 3 || VelocityInit.size() != 3 || 
      AttitudeInit.size() != 3 || AttRateInit.size() != 3 || 
      InertiaInit.size() != 9 || CoMInit.size() != 3 || 
      T_Str2BdyInit.size() != 9)
   {
      std::cerr << "Your initial conditions didn't match up with right sizes\n";
      std::cerr << "Position: "<< PositionInit.size() << std::endl;
      std::cerr << "Velocity: "<< VelocityInit.size() << std::endl;
      std::cerr << "Attitude: "<< AttitudeInit.size() << std::endl;
      std::cerr << "Att-rate: "<< AttRateInit.size() << std::endl;
      std::cerr << "Inertia: "<< InertiaInit.size() << std::endl;
      std::cerr << "CoM: "<< CoMInit.size() << std::endl;
      std::cerr << "Str2Bdy: "<< T_Str2BdyInit.size() << std::endl;
      return;
   }

   memset(AccumDVBdy, 0x0, 3*sizeof(double));

   std::vector<double>::iterator PosIt = PositionInit.begin();
   std::vector<double>::iterator VelIt = VelocityInit.begin();
   std::vector<double>::iterator AttIt = AttitudeInit.begin();
   std::vector<double>::iterator RateIt= AttRateInit.begin();
   std::vector<double>::iterator InertiaIt= InertiaInit.begin();
   std::vector<double>::iterator CoMIt= CoMInit.begin();
   std::vector<double>::iterator Str2BdyIt= T_Str2BdyInit.begin();

   for(uint32_t i=0; i<3; i++)
   {
      XState[i] = *PosIt++;
      XState[i+3] = *VelIt++;
      XState[i+6] = *AttIt++;
      XState[i+9] = *RateIt++;
      CoM[i] = *CoMIt++;
      for(uint32_t j=0; j<3; j++)
      {
         I[i][j] = *InertiaIt++;
         T_str2Bdy[i][j] = *Str2BdyIt++;
      }
   }
   computeOutputs();

   StateOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(
      OutputStateMessage, sizeof(OutputStateData), OutputBufferCount);

   MassPropsMsgID = SystemMessaging::GetInstance()->CreateNewMessage(
      OutputMassPropsMsg, sizeof(MassPropsData), OutputBufferCount);

}

void SixDofEOM::CrossInit()
{

    std::vector<GravityBodyData>::iterator it;
    for(it = GravData.begin(); it != GravData.end(); it++)
    {
       it->BodyMsgID = SystemMessaging::GetInstance()->FindMessageID(
          it->BodyMsgName);
       if(it->BodyMsgID < 0)
       {
          std::cerr << "WARNING: Did not find a valid message with name: ";
          std::cerr << it->BodyMsgName << " :" << __FILE__ <<std::endl;
          std::cerr << "Disabling body gravity." <<std::endl;
       }
    }

}

void SixDofEOM::ReadInputs()
{
    SpicePlanetState LocalPlanet;
    SingleMessageHeader LocalHeader;
    std::vector<GravityBodyData>::iterator it;

    for(it = GravData.begin(); it != GravData.end(); it++)
    {
       if(it->BodyMsgID > 0)
       { 
          SystemMessaging::GetInstance()->ReadMessage(it->BodyMsgID, &LocalHeader,
             sizeof(SpicePlanetState), reinterpret_cast<uint8_t*> (&LocalPlanet));
          memcpy(it->PosFromEphem, LocalPlanet.PositionVector, 3*sizeof(double));
          memcpy(it->VelFromEphem, LocalPlanet.VelocityVector, 3*sizeof(double));
       }
    }
}

void SixDofEOM::equationsOfMotion(double t, double *X, double *dX, 
   GravityBodyData *CentralBody)
{

   std::vector<DynEffector*>::iterator it; 
   OutputStateData StateCurrent;
   MassPropsData MassProps;

    double r_N[3];
    double rmag;
    double v_N[3];
    double accel[3];
    double sunvec[3];
    double sigma[3];
    double omega[3];
    //double Omega[RWACount];       /* RW speeds */
    //double appliedU[RWACount];    /* Actual applied motor torques (wheel-speed limited) */
    double hs;
    double B[3][3];             /* d(sigma)/dt = 1/4 B omega */
    double d2[3];               /* intermediate variables */
    double d3[3];
    double Lt[3];               /* Thrust torque */
    double Ft[3];               /* Thrust force */
    double T_Irtl2Bdy[3][3];
    double LocalAccels[3];
    int    i;
    int    j;

    i = 0;
    /* translate state vector */
    r_N[0] = X[i++];
    r_N[1] = X[i++];
    r_N[2] = X[i++];
    v_N[0] = X[i++];
    v_N[1] = X[i++];
    v_N[2] = X[i++];
    sigma[0] = X[i++];
    sigma[1] = X[i++];
    sigma[2] = X[i++];
    omega[0] = X[i++];
    omega[1] = X[i++];
    omega[2] = X[i++];
    //for(j = 0; j < RWACount; j++) {
    //    Omega[j] = X[i++];
    //}
    /* Check for torque limits due to wheel speed */
    //for(i = 0; i < NUM_RW; i++) {
    //    if(this->rw[i].state == COMPONENT_ON) {
    //        appliedU[i] = this->rw[i].u;
    //    } else {
    //        appliedU[i] = -this->rw[i].Js * this->rw[i].Omega * this->rwAll.frictionTimeConstant;
    //    }
    //}

    rmag = v3Norm(r_N);
    v3Scale(-CentralBody->mu / rmag / rmag / rmag, r_N, dX + 3);
    /// - Add in integrate-computed non-central bodies
    v3Add(dX+3, InertialAccels, dX+3);

    /* compute derivative of position */
    v3Copy(v_N, dX);

    /* compute dsigma/dt */
    BmatMRP(sigma, B);
    m33Scale(0.25, B, B);
    m33MultV3(B, omega, dX + 6);

    /* compute domega/dt */
    v3Tilde(omega, B);                  /* [tilde(w)] */
    m33MultV3(this->I, omega, d2);     /* [I]w */
    //for(i = 0; i < RWCount; i++) {
    //    hs = this->rw[i].Js * (v3Dot(omega, this->rw[i].gs) + Omega[i]);
    //    v3Scale(hs, this->rw[i].gs, d3);
    //    v3Add(d3, d2, d2);
    //}
    m33MultV3(B, d2, d3);               /* [tilde(w)]([I]w + [Gs]hs) */
    m33MultV3(Iinv, d3, dX + 9);  /* d(w)/dt = [I_RW]^-1 . (RHS) */

    memcpy(StateCurrent.r_N, r_N, 3*sizeof(double));
    memcpy(StateCurrent.v_N, v_N, 3*sizeof(double));
    memcpy(StateCurrent.sigma, sigma, 3*sizeof(double));
    memcpy(StateCurrent.omega, omega, 3*sizeof(double));
    memcpy(StateCurrent.T_str2Bdy, T_str2Bdy, 9*sizeof(double));

    MassProps.Mass = mass;
    memcpy(MassProps.CoM, CoM, 3*sizeof(double));
    memcpy(MassProps.InertiaTensor, I, 9*sizeof(double));
    memcpy(MassProps.T_str2Bdy, T_str2Bdy, 9*sizeof(double));

    MRP2C(sigma, T_Irtl2Bdy);

    memset(NonConservAccelBdy, 0x0, 3*sizeof(double));
    for(it=BodyEffectors.begin(); it != BodyEffectors.end(); it++)
    {
       DynEffector *TheEff = *it;
       TheEff->ComputeDynamics(&MassProps, &StateCurrent, t);
       v3Scale(1.0/mass, TheEff->GetBodyForces(), LocalAccels);
       v3Add(LocalAccels, NonConservAccelBdy, NonConservAccelBdy);
       m33tMultV3(T_Irtl2Bdy, LocalAccels, LocalAccels);
       v3Add(dX+3, LocalAccels, dX+3);
       m33MultV3(Iinv, TheEff->GetBodyTorques(), LocalAccels);
       v3Add(dX+9, LocalAccels, dX+9);
       
    }    

}

void SixDofEOM::integrateState(double CurrentTime)
{

    double  *X = new double[NStates];         /* integration state space */
    double  *X2 = new double[NStates];        /* integration state space */
    double  *k1 = new double[NStates];        /* intermediate RK results */
    double  *k2 = new double[NStates];
    double  *k3 = new double[NStates];
    double  *k4 = new double[NStates];
    uint32_t i;
    double TimeStep;
    double sMag;
    uint32_t CentralBodyCount = 0;
    double PlanetAccel[3];
    double PlanetRelPos[3];
    double LocalDV[3];
    double rmag;

    TimeStep = CurrentTime - TimePrev;
    GravityBodyData *CentralBody = NULL;

    memcpy(X, XState, NStates*sizeof(double));
    m33Inverse(this->I, Iinv);

    std::vector<GravityBodyData>::iterator it;
    for(it = GravData.begin(); it != GravData.end(); it++)
    {
       CentralBody = &(*it);
       if(it->IsCentralBody)
       {
          CentralBodyCount++;
          break;
       }
    }
   
    if(CentralBodyCount != 1)
    {
       std::cerr << "ERROR: I got a bad count on central bodies: " <<
          CentralBodyCount;
    }

    v3SetZero(InertialAccels);
    for(it = GravData.begin(); it != GravData.end(); it++)
    {
       if(it->IsCentralBody || it->BodyMsgID < 0)
       {
          continue;
       }
       v3Add(r_N, CentralBody->PosFromEphem, PlanetRelPos);
       v3Subtract(PlanetRelPos, it->PosFromEphem, PlanetRelPos);
       rmag = v3Norm(PlanetRelPos);
       v3Scale(-it->mu / rmag / rmag / rmag, PlanetRelPos, PlanetAccel);
       v3Add(InertialAccels, PlanetAccel, InertialAccels);
    }
    /* do RK4 evaluations */
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

    /* do MRP shadow set switching check */
    if((sMag = v3Norm(&XState[6])) > 1.0) {
        v3Scale(-1.0 / sMag / sMag, &this->XState[6], &this->XState[6]);
        MRPSwitchCount++;
    }

    TimePrev = CurrentTime;
	delete[] X;
	delete[]X2;
	delete[] k1;
	delete[] k2;
	delete[] k3;
	delete[] k4;

}

void SixDofEOM::computeOutputs()
{
   memcpy(r_N, &(XState[0]), 3*sizeof(double));
   memcpy(v_N, &(XState[3]), 3*sizeof(double));
   memcpy(sigma, &(XState[6]), 3*sizeof(double));
   memcpy(omega, &(XState[9]), 3*sizeof(double));
}

void SixDofEOM::WriteOutputMessages(uint64_t CurrentClock)
{
   if(StateOutMsgID < 0)
   {
      return;
   }
   OutputStateData StateOut;
   memcpy(StateOut.r_N, r_N, 3*sizeof(double));
   memcpy(StateOut.v_N, v_N, 3*sizeof(double));
   memcpy(StateOut.sigma, sigma, 3*sizeof(double));
   memcpy(StateOut.omega, omega, 3*sizeof(double));
   memcpy(StateOut.T_str2Bdy, T_str2Bdy, 9*sizeof(double));
   memcpy(StateOut.TotalAccumDVBdy, AccumDVBdy, 3*sizeof(double));
   StateOut.MRPSwitchCount = MRPSwitchCount;
   SystemMessaging::GetInstance()->WriteMessage(StateOutMsgID, CurrentClock,
      sizeof(OutputStateData), reinterpret_cast<uint8_t*> (&StateOut));

   MassPropsData massProps;
   massProps.Mass = mass;
   memcpy(massProps.CoM, CoM, 3*sizeof(double));
   memcpy(&(massProps.InertiaTensor[0][0]), &(I[0][0]), 9*sizeof(double));
   memcpy(massProps.T_str2Bdy, T_str2Bdy, 9*sizeof(double));
   SystemMessaging::GetInstance()->WriteMessage(MassPropsMsgID, CurrentClock,
      sizeof(MassPropsData), reinterpret_cast<uint8_t*> (&massProps));

}

void SixDofEOM::UpdateState(uint64_t CurrentSimNanos)
{
   ReadInputs();
   integrateState(CurrentSimNanos*1.0E-9);
   computeOutputs();
   WriteOutputMessages(CurrentSimNanos);
}
