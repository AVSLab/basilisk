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

#include "eclipse.h"
#include <iostream>
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"


Eclipse::Eclipse()
{
    rEqCustom = -1.0;
    return;
}

Eclipse::~Eclipse()
{
    for (long unsigned int c=0; c<this->eclipseOutMsgs.size(); c++) {
        delete this->eclipseOutMsgs.at(c);
    }
    return;
}



/*! Reset the module to origina configuration values.

 */
void Eclipse::Reset(uint64_t CurrenSimNanos)
{
    if (!this->sunInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Eclipse: sunInMsg must be linked to sun Spice state message.");
    }

    if (this->positionInMsgs.size() == 0) {
        bskLogger.bskLog(BSK_ERROR, "Eclipse: positionInMsgs is empty.  Must use addSpacecraftToModel() to add at least one spacecraft.");
    }

    if (this->planetInMsgs.size() == 0) {
        bskLogger.bskLog(BSK_ERROR, "Eclipse: planetInMsgs is empty.  Must use addPlanetToModel() to add at least one planet.");
    }

}

/*! This method reads the spacecraft state, spice planet states and the sun position from the messaging system.

 */
void Eclipse::readInputMessages()
{
    for (long unsigned int c = 0; c<this->positionInMsgs.size(); c++){
        this->scStateBuffer.at(c) = this->positionInMsgs.at(c)();
    }

    this->sunInMsgState = this->sunInMsg();

    for (long unsigned int c = 0; c<this->planetInMsgs.size(); c++){
        this->planetBuffer[c] = this->planetInMsgs[c]();
    }
}

/*! This method takes the computed shadow factors and outputs them to the
 messaging system.
 @param CurrentClock The current simulation time (used for time stamping)

 */
void Eclipse::writeOutputMessages(uint64_t CurrentClock)
{
    for (long unsigned int c = 0; c < this->eclipseOutMsgs.size(); c++) {
        EclipseMsgPayload tmpEclipseMsg = {};
        tmpEclipseMsg.shadowFactor = this->eclipseShadowFactors.at(c);
        this->eclipseOutMsgs.at(c)->write(&tmpEclipseMsg, this->moduleID, CurrentClock);
    }
}

/*! This method governs the calculation and checking for eclipse
 conditions.
 @param CurrentSimNanos The current clock time for the simulation

 */
void Eclipse::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();

    // A lot of different vectors here. The below letters denote frames
    // P: planet frame
    // B: spacecraft body frame
    // N: inertial frame
    // H: sun (helio) frame
    Eigen::Vector3d r_PN_N(0.0, 0.0, 0.0); // r_planet
    Eigen::Vector3d r_HN_N(this->sunInMsgState.PositionVector); // r_sun
    Eigen::Vector3d r_BN_N(0.0, 0.0, 0.0); // r_sc
    Eigen::Vector3d s_BP_N(0.0, 0.0, 0.0); // s_sc wrt planet
    Eigen::Vector3d s_HP_N(0.0, 0.0, 0.0); // s_sun wrt planet
    Eigen::Vector3d r_HB_N(0.0, 0.0, 0.0); // r_sun wrt sc
    std::vector<SCStatesMsgPayload>::iterator scIt;
    std::vector<SpicePlanetStateMsgPayload>::iterator planetIt;


    // Index to assign the shadowFactor for each body position (S/C)
    // being tracked
    int scIdx = 0;

    for(scIt = this->scStateBuffer.begin(); scIt != this->scStateBuffer.end(); scIt++)
    {
        double tmpShadowFactor = 1.0; // 1.0 means 100% illumination (no eclipse)
        double eclipsePlanetDistance = 0.0;
        int64_t eclipsePlanetKey = -1;
        r_BN_N = cArray2EigenVector3d(scIt->r_BN_N);

        // Find the closest planet if there is one
        int idx = 0;
        for(planetIt = this->planetBuffer.begin(); planetIt != this->planetBuffer.end(); planetIt++)
        {
            r_PN_N = cArray2EigenVector3d(planetIt->PositionVector);
            s_HP_N = r_HN_N - r_PN_N;
            r_HB_N = r_HN_N - r_BN_N;
            s_BP_N = r_BN_N - r_PN_N;

            // If spacecraft is closer to sun than planet
            // then eclipse not possible
            if (r_HB_N.norm() < s_HP_N.norm()) {
                idx++;
                continue;
            }
            else{
                // Find the closest planet and save its distance and vector index slot
                if (s_BP_N.norm() < eclipsePlanetDistance || eclipsePlanetKey < 0) {
                    eclipsePlanetDistance = s_BP_N.norm();
                    eclipsePlanetKey = idx;
                }
                idx++;
            }
        }

        // If planetkey is not -1 then we have a planet for which
        // we compute the eclipse conditions
        if (eclipsePlanetKey >= 0) {
            r_PN_N = cArray2EigenVector3d(this->planetBuffer[eclipsePlanetKey].PositionVector);
            s_BP_N = r_BN_N - r_PN_N;
            r_HB_N = r_HN_N - r_BN_N;
            s_HP_N = r_HN_N - r_PN_N;

            double s = s_BP_N.norm();
            std::string plName(this->planetBuffer[eclipsePlanetKey].PlanetName);
            double planetRadius = this->getPlanetEquatorialRadius(plName);
            double f_1 = safeAsin((REQ_SUN*1000 + planetRadius)/s_HP_N.norm());
            double f_2 = safeAsin((REQ_SUN*1000 - planetRadius)/s_HP_N.norm());
            double s_0 = (-s_BP_N.dot(s_HP_N))/s_HP_N.norm();
            double c_1 = s_0 + planetRadius/sin(f_1);
            double c_2 = s_0 - planetRadius/sin(f_2);
            double l = safeSqrt(s*s - s_0*s_0);
            double l_1 = c_1*tan(f_1);
            double l_2 = c_2*tan(f_2);

            if (fabs(l) < fabs(l_2)) {
                if (c_2 < 0) { // total eclipse
                    tmpShadowFactor = this->computePercentShadow(planetRadius, r_HB_N, s_BP_N);
                } else { //c_2 > 0 // annular
                    tmpShadowFactor = this->computePercentShadow(planetRadius, r_HB_N, s_BP_N);
                }
            } else if (fabs(l) < fabs(l_1)) { // partial
                tmpShadowFactor = this->computePercentShadow(planetRadius, r_HB_N, s_BP_N);
            }
        }
        this->eclipseShadowFactors.at(scIdx) = tmpShadowFactor;
        scIdx++;
    }
    this->writeOutputMessages(CurrentSimNanos);
}

/*! This method computes the fraction of sunlight given an eclipse.
 @param planetRadius
 @param r_HB_N
 @param s_BP_N
 @return double fractionShadow The eclipse shadow fraction.
 */
double Eclipse::computePercentShadow(double planetRadius, Eigen::Vector3d r_HB_N, Eigen::Vector3d s_BP_N)
{
    double area = 0.0;
    double shadowFraction = 1.0; // Initialise to value for no eclipse
    double normR_HB_N = r_HB_N.norm();
    double normS_BP_N = s_BP_N.norm();
    double a = safeAsin(REQ_SUN*1000/normR_HB_N); // apparent radius of sun
    double b = safeAsin(planetRadius/normS_BP_N); // apparent radius of occulting body
    double c = safeAcos((-s_BP_N.dot(r_HB_N))/(normS_BP_N*normR_HB_N));

    // The order of these conditionals is important.
    // In particular (c < a + b) must check last to avoid testing
    // with implausible a, b and c values
    if (c < b - a) { // total eclipse, implying a < b
        shadowFraction = 0.0;
    } else if (c < a - b) { // partial maximum eclipse, implying a > b
        double areaSun = M_PI*a*a;
        double areaBody = M_PI*b*b;
        area = areaSun - areaBody;
        shadowFraction = 1 - area/(M_PI*a*a);
    } else if (c < a + b) { // partial eclipse
        double x = (c*c + a*a - b*b)/(2*c);
        double y = sqrt(a*a - x*x);
        area = a*a*safeAcos(x/a) + b*b*safeAcos((c-x)/b) - c*y;
        shadowFraction = 1 - area/(M_PI*a*a);
    }
    return shadowFraction;
}

/*!
    This method adds spacecraft state data message names to a vector, creates
    a new unique output message for the eclipse data message.
    @param tmpScMsg The state output message for the spacecraft for which to compute the eclipse data.

 */
void Eclipse::addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg)
{
    this->positionInMsgs.push_back(tmpScMsg->addSubscriber());

    /* create output message */
    Message<EclipseMsgPayload> *msg;
    msg = new Message<EclipseMsgPayload>;
    this->eclipseOutMsgs.push_back(msg);

    /* expand the sc state buffer vector */
    SCStatesMsgPayload scMsg;
    this->scStateBuffer.push_back(scMsg);

    // Now that we know the number of output messages we can size and zero
    // the eclipse data vector
    this->eclipseShadowFactors.resize(this->eclipseOutMsgs.size());

}

/*! This method adds planet state data message names to a vector.
 @param tmpSpMsg The planet name

 */
void Eclipse::addPlanetToModel(Message<SpicePlanetStateMsgPayload> *tmpSpMsg)
{
    this->planetInMsgs.push_back(tmpSpMsg->addSubscriber());

    SpicePlanetStateMsgPayload tmpMsg;
    this->planetBuffer.push_back(tmpMsg);
    return;
}

/*! This method return planet radii.
 @param  planetSpiceName The planet name according to the spice NAIF Integer ID codes.
 @return double  The equatorial radius in metres associated with the given planet name.
 */
double Eclipse::getPlanetEquatorialRadius(std::string planetSpiceName)
{
    if (planetSpiceName == "mercury") {
        return REQ_MERCURY*1000.0; // [meters]
    } else if (planetSpiceName == "venus") {
        return REQ_VENUS*1000.0;
    } else if (planetSpiceName == "earth") {
        return REQ_EARTH*1000.0;
    } else if (planetSpiceName == "moon") {
        return REQ_MOON*1000.0;
    } else if (planetSpiceName == "mars barycenter") {
        return REQ_MARS*1000.0;
    } else if (planetSpiceName == "mars") {
        return REQ_MARS*1000.0;
    } else if (planetSpiceName == "jupiter barycenter") {
        return REQ_JUPITER*1000.0;
    } else if (planetSpiceName == "saturn") {
        return REQ_SATURN*1000.0;
    } else if (planetSpiceName == "uranus") {
        return REQ_URANUS*1000.0;
    } else if (planetSpiceName == "neptune") {
        return REQ_NEPTUNE*1000.0;
    } else if (planetSpiceName == "custom") {
        if (rEqCustom <= 0.0) {
            bskLogger.bskLog(BSK_ERROR, "Eclipse: Invalid rEqCustom set.");
            return 1.0;
        } else {
            return rEqCustom;
        }
    } else {
        bskLogger.bskLog(BSK_ERROR, "Eclipse: unrecognized planetSpiceName.");
        return 1.0;
    }
}
