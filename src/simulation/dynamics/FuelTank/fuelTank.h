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
#ifndef FUEL_TANK_H
#define FUEL_TANK_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/FuelTankMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "simulation/dynamics/Thrusters/thrusterDynamicEffector/thrusterDynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/fuelSlosh.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/Thrusters/thrusterStateEffector/thrusterStateEffector.h"

#include <math.h>
#include <vector>
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"


/*! Tank model class */
class FuelTankModel {
public:
    double propMassInit{};                              //!< [kg] Initial propellant mass in tank
    double maxFuelMass = 1.0;                           //!< [kg] maximum tank mass
    Eigen::Vector3d r_TcT_TInit;                        //!< [m] Initial position vector from B to tank point in B frame comp.
    Eigen::Matrix3d ITankPntT_T;                        //!< [kg m^2] Inertia of tank about pnt T in B frame comp.
    Eigen::Matrix3d IPrimeTankPntT_T;                   //!< [kg m^2/s] Derivative of inertia of tank about pnt T in B frame comp.
    Eigen::Vector3d r_TcT_T;                            //!< [m] position vector from B to tank point in B frame comp.
    Eigen::Vector3d rPrime_TcT_T;                       //!< [m/s] Derivative of position vector from B to tank point in B frame comp.
    Eigen::Vector3d rPPrime_TcT_T;                      //!< [m/s^2] Second derivative of position vector from B to tank point in B frame comp.
    virtual void computeTankProps(double mFuel) = 0;    //!< class method
    virtual void computeTankPropDerivs(double mFuel, double mDotFuel) = 0; //!< class method
    FuelTankModel() {
        this->r_TcT_TInit.setZero();
    }
    virtual ~FuelTankModel() = default;
};

/*! Tank constant volume class */
class FuelTankModelConstantVolume : public FuelTankModel {
public:
    double radiusTankInit{};                            //!< [m] Initial radius of the spherical tank

    FuelTankModelConstantVolume() = default;

    ~FuelTankModelConstantVolume() override = default;

    void computeTankProps(double mFuel) override {
        this->r_TcT_T = this->r_TcT_TInit;
        this->ITankPntT_T = 2.0 / 5.0 * mFuel * this->radiusTankInit * this->radiusTankInit * Eigen::Matrix3d::Identity();
    }

    void computeTankPropDerivs(double mFuel, double mDotFuel) override {
        this->IPrimeTankPntT_T = 2.0 / 5.0 * mDotFuel * this->radiusTankInit * this->radiusTankInit * Eigen::Matrix3d::Identity();
        this->rPrime_TcT_T.setZero();
        this->rPPrime_TcT_T.setZero();
    }
};

/*! Tank constant density class */
class FuelTankModelConstantDensity : public FuelTankModel {
public:
    double radiusTankInit{};                            //!< [m] Initial radius of the spherical tank
    double radiusTank{};                                //!< [m] Current radius of the spherical tank

    FuelTankModelConstantDensity() = default;

    ~FuelTankModelConstantDensity() override = default;

    void computeTankProps(double mFuel) override {
        this->radiusTank = std::pow(mFuel / this->propMassInit, 1.0 / 3.0) * this->radiusTankInit;
        this->r_TcT_T = this->r_TcT_TInit;
        this->ITankPntT_T = 2.0 / 5.0 * mFuel * this->radiusTank * this->radiusTank * Eigen::Matrix3d::Identity();
    }

    void computeTankPropDerivs(double mFuel, double mDotFuel) override {
        this->IPrimeTankPntT_T = 2.0 / 3.0 * mDotFuel * this->radiusTank * this->radiusTank * Eigen::Matrix3d::Identity();
        this->rPrime_TcT_T.setZero();
        this->rPPrime_TcT_T.setZero();
    }
};

/*! Tank model emptying class */
class FuelTankModelEmptying : public FuelTankModel {
public:
    double radiusTankInit{};                            //!< [m] Initial radius of the spherical tank
    double rhoFuel{};                                   //!< [kg/m^3] density of the fuel
    double thetaStar{};                                 //!< [rad] angle from vertical to top of fuel
    double thetaDotStar{};                              //!< [rad/s] derivative of angle from vertical to top of fuel
    double thetaDDotStar{};                             //!< [rad/s^2] second derivative of angle from vertical to top of fuel
    Eigen::Vector3d k3;                                 //!< -- Direction of fuel depletion

    FuelTankModelEmptying() = default;

    ~FuelTankModelEmptying() override = default;

    void computeTankProps(double mFuel) override {
        this->rhoFuel = this->propMassInit / (4.0 / 3.0 * M_PI * this->radiusTankInit * this->radiusTankInit * this->radiusTankInit);
        double rtank = this->radiusTankInit;
        double volume;
        double deltaRadiusK3;
        this->k3 << 0, 0, 1; //k3 is zhat

        if (mFuel != this->propMassInit) {
            double rhoFuel = this->rhoFuel;
            std::function<double(double)> f = [rhoFuel, rtank, mFuel](double thetaStar) -> double {
                return 2.0 / 3.0 * M_PI * rhoFuel * rtank * rtank * rtank *
                       (1 + 3.0 / 2.0 * cos(thetaStar) - 1.0 / 2.0 * pow(cos(thetaStar), 3)) - mFuel;
            };
            std::function<double(double)> fPrime = [rhoFuel, rtank](double thetaStar) -> double {
                return 2.0 / 3.0 * M_PI * rhoFuel * rtank * rtank * rtank *
                       (-3.0 / 2.0 * sin(thetaStar) + 3.0 / 2.0 * pow(cos(thetaStar), 2) * sin(thetaStar));
            };

            this->thetaStar = newtonRaphsonSolve(M_PI / 2.0, 1E-20, f, fPrime);
        } else {
            this->thetaStar = 0.0;
        }
        volume = 2.0 / 3.0 * M_PI * std::pow(this->radiusTankInit, 3) *
                 (1 + 3.0 / 2.0 * std::cos(this->thetaStar) - 1.0 / 2.0 * std::pow(std::cos(this->thetaStar), 3));
        if (volume != 0) {
            deltaRadiusK3 = M_PI * std::pow(this->radiusTankInit, 4) / (4.0 * volume) *
                            (2.0 * std::pow(std::cos(this->thetaStar), 2) - std::pow(std::cos(this->thetaStar), 4) - 1);
        } else {
            deltaRadiusK3 = -this->radiusTankInit;
        }

        this->r_TcT_T = this->r_TcT_TInit + deltaRadiusK3 * this->k3;
        this->ITankPntT_T.setZero();
        this->IPrimeTankPntT_T.setZero();
        this->ITankPntT_T(2, 2) = 2.0 / 5.0 * M_PI * this->rhoFuel * std::pow(this->radiusTankInit, 5) *
                            (2.0 / 3.0 + 1.0 / 4.0 * std::cos(this->thetaStar) * std::pow(std::sin(this->thetaStar), 4) -
                             1 / 12.0 * (std::cos(3 * this->thetaStar) - 9 * std::cos(this->thetaStar)));
        this->ITankPntT_T(0, 0) = this->ITankPntT_T(1, 1) = 2.0 / 5.0 * M_PI * this->rhoFuel * std::pow(this->radiusTankInit, 5) *
                                                (2.0 / 3.0 - 1.0 / 4.0 * std::pow(std::cos(this->thetaStar), 5) +
                                                 1 / 24.0 * (std::cos(3 * this->thetaStar) - 9 * std::cos(this->thetaStar)) +
                                                 5.0 / 4.0 * cos(this->thetaStar) +
                                                 1 / 8.0 * std::cos(this->thetaStar) * std::pow(std::sin(this->thetaStar), 4));
    }

    void computeTankPropDerivs(double mFuel, double mDotFuel) override {
        if (mFuel != this->propMassInit) {
            this->thetaDotStar = -mDotFuel / (M_PI * this->rhoFuel * std::pow(this->radiusTankInit, 3) * std::sin(this->thetaStar));
            this->thetaDDotStar = -3 * this->thetaDotStar * this->thetaDotStar * std::cos(this->thetaStar) /
                            std::sin(this->thetaStar); //This assumes that mddot = 0
        } else {
            this->thetaDotStar = 0.0;
            this->thetaDDotStar = 0.0;
        }
        this->IPrimeTankPntT_T(2, 2) = 2.0 / 5.0 * M_PI * this->rhoFuel * std::pow(this->radiusTankInit, 5) * this->thetaDotStar *
                                 (std::pow(std::cos(this->thetaStar), 2) * std::pow(std::sin(this->thetaStar), 3) -
                                  1.0 / 4.0 * std::pow(std::sin(this->thetaStar), 5) +
                                  1 / 4.0 * std::sin(3 * this->thetaStar) - 3.0 / 4.0 * std::sin(this->thetaStar));
        this->IPrimeTankPntT_T(0, 0) = this->IPrimeTankPntT_T(1, 1) =
                2.0 / 5.0 * M_PI * this->rhoFuel * std::pow(this->radiusTankInit, 5) * this->thetaDotStar *
                (5.0 / 4.0 * std::sin(this->thetaStar) * std::cos(this->thetaStar) - 5.0 / 4.0 * std::sin(this->thetaStar) -
                 1 / 8.0 * std::sin(3 * this->thetaStar) +
                 3.0 / 8.0 * sin(this->thetaStar) +
                 1 / 2.0 * std::pow(std::cos(this->thetaStar), 2) * std::pow(std::sin(this->thetaStar), 3) -
                 1 / 8.0 * std::pow(std::sin(this->thetaStar), 5));
        if (mFuel != 0) {
            this->rPrime_TcT_T = -M_PI * std::pow(this->radiusTankInit, 4) * this->rhoFuel / (4 * mFuel * mFuel) *
                           (4 * mFuel * this->thetaDotStar * std::pow(std::sin(this->thetaStar), 3) * std::cos(this->thetaStar) +
                            mDotFuel * (2 * std::pow(std::cos(this->thetaStar), 2) - std::pow(std::cos(this->thetaStar), 4) - 1)) *
                           this->k3;

            this->rPPrime_TcT_T = -M_PI * std::pow(this->radiusTankInit, 4) * this->rhoFuel / (2 * mFuel * mFuel * mFuel) *
                            (4 * mFuel * std::pow(std::sin(this->thetaStar), 3) * std::cos(this->thetaStar) *
                             (this->thetaDDotStar * mFuel - 2 * this->thetaDotStar * mDotFuel) -
                             4 * mFuel * mFuel * this->thetaDotStar * this->thetaDotStar * std::pow(std::sin(this->thetaStar), 2) *
                             (3 * std::pow(std::cos(this->thetaStar), 2) - std::pow(std::sin(this->thetaStar), 2)) +
                             (2 * std::pow(std::cos(this->thetaStar), 2) - std::pow(std::cos(this->thetaStar), 4) - 1) *
                             (-2 * mDotFuel * mDotFuel)) * this->k3;

        } else {
            this->rPrime_TcT_T.setZero();
            this->rPPrime_TcT_T.setZero();
        }
    }
};

/*! Tank model class for a uniform burn */
class FuelTankModelUniformBurn : public FuelTankModel {
public:
    double radiusTankInit{};                            //!< [m] Initial radius of the cylindrical tank
    double lengthTank{};                                //!< [m] Length of the tank

    FuelTankModelUniformBurn() = default;

    ~FuelTankModelUniformBurn() override = default;

    void computeTankProps(double mFuel) override {
        this->r_TcT_T = this->r_TcT_TInit;
        this->ITankPntT_T.setZero();
        this->ITankPntT_T(0, 0) = this->ITankPntT_T(1, 1) =
                mFuel * (this->radiusTankInit * this->radiusTankInit / 4.0 + this->lengthTank * this->lengthTank / 12.0);
        this->ITankPntT_T(2, 2) = mFuel * this->radiusTankInit * this->radiusTankInit / 2;
    }

    void computeTankPropDerivs(double mFuel, double mDotFuel) override {
        this->IPrimeTankPntT_T.setZero();
        this->IPrimeTankPntT_T(0, 0) = this->IPrimeTankPntT_T(1, 1) =
                mDotFuel * (this->radiusTankInit * this->radiusTankInit / 4.0 + this->lengthTank * this->lengthTank / 12.0);
        this->IPrimeTankPntT_T(2, 2) = mDotFuel * this->radiusTankInit * this->radiusTankInit / 2;
        this->rPrime_TcT_T.setZero();
        this->rPPrime_TcT_T.setZero();
    }
};

/*! Tank model class for a centrifugal burn */
class FuelTankModelCentrifugalBurn : public FuelTankModel {
public:
    double radiusTankInit{};                            //!< [m] Initial radius of the cylindrical tank
    double lengthTank{};                                //!< [m] Length of the tank
    double radiusInner{};                               //!< [m] Inner radius of the cylindrical tank

    FuelTankModelCentrifugalBurn() = default;

    ~FuelTankModelCentrifugalBurn() override = default;

    void computeTankProps(double mFuel) override {
        double rhoFuel = this->propMassInit / (M_PI * this->radiusTankInit * this->radiusTankInit * this->lengthTank);
        this->radiusInner = std::sqrt(std::max(this->radiusTankInit * this->radiusTankInit - mFuel / (M_PI * this->lengthTank * rhoFuel), 0.0));
        this->r_TcT_T = this->r_TcT_TInit;
        this->ITankPntT_T.setZero();
        this->ITankPntT_T(0, 0) = this->ITankPntT_T(1, 1) = mFuel *
                                                ((this->radiusTankInit * this->radiusTankInit +
                                                this->radiusInner * this->radiusInner) / 4.0 +
                                                 this->lengthTank * this->lengthTank / 12.0);
        this->ITankPntT_T(2, 2) = mFuel * (this->radiusTankInit * this->radiusTankInit +
                this->radiusInner * this->radiusInner) / 2;
    }

    void computeTankPropDerivs(double mFuel, double mDotFuel) override {
        this->IPrimeTankPntT_T.setZero();
        this->IPrimeTankPntT_T(0, 0) = this->IPrimeTankPntT_T(1, 1) =
                mDotFuel * (this->radiusInner * this->radiusInner / 2.0 + this->lengthTank * this->lengthTank / 12.0);
        this->IPrimeTankPntT_T(2, 2) = mDotFuel * this->radiusInner * this->radiusInner;
        this->rPrime_TcT_T.setZero();
        this->rPPrime_TcT_T.setZero();
    }
};

/*! Fuel tank effector model class */
class FuelTank :
        public StateEffector, public SysModel {
public:
    std::string nameOfMassState{};                      //!< -- name of mass state
    std::vector<FuelSlosh *> fuelSloshParticles;        //!< -- vector of fuel slosh particles
    std::vector<ThrusterDynamicEffector *> thrDynEffectors;        //!< -- Vector of dynamic effectors for thrusters
    std::vector<ThrusterStateEffector *> thrStateEffectors;        //!< -- Vector of state effectors for thrusters
    Eigen::Matrix3d dcm_TB;                             //!< -- DCM from body frame to tank frame
    Eigen::Vector3d r_TB_B;                             //!< [m] position of tank in B frame
    bool updateOnly = true;                             //!< -- Sets whether to use update only mass depletion
    Message<FuelTankMsgPayload> fuelTankOutMsg{};       //!< -- fuel tank output message name
    FuelTankMsgPayload fuelTankMassPropMsg{};           //!< instance of messaging system message struct

private:
    StateData *omegaState{};                            //!< -- state data for omega_BN of the hub
    StateData *massState{};                             //!< -- state data for mass state
    double fuelConsumption{};                           //!< [kg/s] rate of fuel being consumed
    double tankFuelConsumption{};                       //!< [kg/s] rate of fuel being consumed from tank
    FuelTankModel *fuelTankModel{};                     //!< -- style of tank to simulate
    Eigen::Matrix3d ITankPntT_B;
    Eigen::Vector3d r_TcB_B;
    static uint64_t effectorID;                         //!< [] ID number of this fuel tank effector

public:
    FuelTank();
    ~FuelTank();
    void writeOutputMessages(uint64_t currentClock);
    void UpdateState(uint64_t currentSimNanos) override;
    void setTankModel(FuelTankModel *model);
    void pushFuelSloshParticle(FuelSlosh *particle);            //!< -- Attach fuel slosh particle
    void registerStates(DynParamManager &states) override;      //!< -- Register mass state with state manager
    void linkInStates(DynParamManager &states) override;        //!< -- Give the tank access to other states
    void updateEffectorMassProps(double integTime) override;    //!< -- Add contribution mass props from the tank
    void setNameOfMassState(const std::string nameOfMassState); //!< -- Setter for fuel tank mass state name
    void addThrusterSet(ThrusterDynamicEffector *dynEff);       //!< -- Add DynamicEffector thruster
    void addThrusterSet(ThrusterStateEffector *stateEff);       //!< -- Add StateEffector thruster
    void updateContributions(double integTime,
                             BackSubMatrices &backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;     //!< -- Back-sub contributions
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d &rotAngMomPntCContr_B,
                                      double &rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override;  //!< -- Energy and momentum calculations
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN) override; //!< -- Calculate stateEffector's derivatives
};


#endif /* FUEL_TANK_H */
