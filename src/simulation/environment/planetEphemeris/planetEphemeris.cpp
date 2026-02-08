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
#include "simulation/environment/planetEphemeris/planetEphemeris.h"
#include <iostream>
#include <string.h>
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"


/*! This constructor initializes the variables.
 */
PlanetEphemeris::PlanetEphemeris()
{
    return;
}

/*! Module deconstructor */
PlanetEphemeris::~PlanetEphemeris()
{
    for (long unsigned int c=0; c<this->planetOutMsgs.size(); c++) {
        delete this->planetOutMsgs.at(c);
    }
    return;
}

/*! add list of planet names */
void PlanetEphemeris::setPlanetNames(std::vector<std::string> names)
{
    this->planetNames = names;

    /* create corresponding output messages */
    for (long unsigned int c=0; c<this->planetNames.size(); c++) {
        Message<SpicePlanetStateMsgPayload> *spMsg;
        spMsg = new Message<SpicePlanetStateMsgPayload>;
        this->planetOutMsgs.push_back(spMsg);
    }
}



void PlanetEphemeris::Reset(uint64_t CurrenSimNanos)
{
    this->epochTime = CurrenSimNanos*NANO2SEC;

    /*! - do sanity checks that the vector arrays for planet names and ephemeris have the same length */
    if (this->planetElements.size() != this->planetNames.size()) {
        bskLogger.bskLog(BSK_ERROR, "Only %lu planet element sets provided, but %lu plane names are present.",
                  this->planetElements.size(), this->planetNames.size());
    }

    /*! - See if planet orientation information is set */
    if(this->lst0.size() == 0 && this->rotRate.size() == 0 &&
       this->declination.size() == 0 && this->rightAscension.size() == 0) {
        this->computeAttitudeFlag = 0;
        return;
    } else {
        this->computeAttitudeFlag = 1;
    }

    if (computeAttitudeFlag) {
        /*! - check that the right number of planet local sideral time angles are provided */
        if (this->lst0.size() != this->planetNames.size()) {
            bskLogger.bskLog(BSK_ERROR, "Only %lu planet initial principal rotation angles provided, but %lu planet names are present.",
                      this->lst0.size(), this->planetNames.size());
        }

        /*! - check that the right number of planet polar axis right ascension angles are provided */
        if (this->rightAscension.size() != this->planetNames.size()) {
            bskLogger.bskLog(BSK_ERROR, "Only %lu planet right ascension angles provided, but %lu planet names are present.",
                      this->rightAscension.size(), this->planetNames.size());
        }

        /*! - check that the right number of planet polar axis declination angles are provided */
        if (this->declination.size() != this->planetNames.size()) {
            bskLogger.bskLog(BSK_ERROR, "Only %lu planet declination angles provided, but %lu planet names are present.",
                      this->declination.size(), this->planetNames.size());
        }

        /*! - check that the right number of planet polar rotation rates are provided */
        if (this->rotRate.size() != this->planetNames.size()) {
            bskLogger.bskLog(BSK_ERROR, "Only %lu planet rotation rates provided, but %lu planet names are present.",
                      this->rotRate.size(), this->planetNames.size());
        }
    }

    return;
}


/*! This update routine loops over all the planets and creates their heliocentric position
 and velocity vectors at the current time. If the planet orientation information is provided,
 then this is computed as well.  The default orientation information is a zero orientation.

 @param CurrentSimNanos The current clock time for the simulation
 */
void PlanetEphemeris::UpdateState(uint64_t CurrentSimNanos)
{
    std::vector<std::string>::iterator it;
    double time;                            // [s] time since epoch
    double mu;                              // [m^3/s^2] gravity constant of the sun
    uint64_t c;                               // [] counter
    double f0;                              // [r] true anomaly at epoch
    double e;                               // []  orbit eccentricity
    double M0;                              // [r] mean anomaly at epoch
    double M;                               // [r] current mean anomaly
    double lst;                             // [r] local sidereal time angle
    double omega_NP_P[3];                   // [r/s] angular velocity of inertial frame relative to planet frame in planet frame components
    double tilde[3][3];                     // [] skew-symmetric matrix
    double theta_PN[3];                     // [rad] 3-2-3 planet orientation coordinates



    //! - set time in units of seconds
    time = CurrentSimNanos*NANO2SEC;

    //! - set sun gravity constant
    mu = MU_SUN*pow(1000.,3);

    //! - Loop over the planet names and create new data
    for(it = this->planetNames.begin(), c=0; it != this->planetNames.end(); it++, c++)
    {
        //! - Create new planet output message copy
        SpicePlanetStateMsgPayload newPlanet;
        newPlanet = this->planetOutMsgs.at(c)->zeroMsgPayload;
        //! - specify planet name in output message
        strcpy(newPlanet.PlanetName, it->c_str());

        //! - tag time to message
        newPlanet.J2000Current = time;

        //! - compute current planet sun-centric inertial position and velocity vectors
        f0 = this->planetElements[c].f;
        e = this->planetElements[c].e;
        M0 = E2M(f2E(f0, e), e);
        M = M0 + sqrt(mu/pow(this->planetElements[c].a, 3)) * (time - this->epochTime);
        this->planetElements[c].f = E2f(M2E(M, e),e);
        elem2rv(mu, &(this->planetElements[c]), newPlanet.PositionVector, newPlanet.VelocityVector);

        //! - retain planet epoch information
        this->planetElements[c].f = f0;

        if (this->computeAttitudeFlag == 1) {
            //! - compute current planet principal rotation parameter vector */
            lst = this->lst0[c] + this->rotRate[c]*(time - this->epochTime);

            v3Set(this->rightAscension.at(c), M_PI_2 - this->declination.at(c), lst, theta_PN);
            Euler3232C(theta_PN, newPlanet.J20002Pfix);

            //! - compute the planet DCM rate
            v3Set(0.0, 0.0, -this->rotRate[c], omega_NP_P);
            v3Tilde(omega_NP_P, tilde);
            m33MultM33(tilde, newPlanet.J20002Pfix, newPlanet.J20002Pfix_dot);

            //! - set flag that planet orientation is computed
            newPlanet.computeOrient = 1;
        } else {
            m33SetIdentity(newPlanet.J20002Pfix); // set default zero orientation
        }

        //! - write output message
        this->planetOutMsgs.at(c)->write(&newPlanet, this->moduleID, CurrentSimNanos);

    }
    return;
}
