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

#include "magneticField.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "utilities/bsk_Print.h"
#include "utilities/linearAlgebra.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/*! This method initializes some basic parameters for the module.
 @return void
 */
MagneticField::MagneticField()
{
    this->planetPosInMsgName = "";
    this->OutputBufferCount = 2;
    //! - Set the default atmospheric properties to those of Earth
    this->envType = MODEL_CENTERED_DIPOLE;  // - simple dipole magnectic field model center in the planet
    this->dipoleParams.g10 = -30926.00/pow(10,9);       // [T] Earth IGRF 2020 g_1^0 cooefficient
    this->dipoleParams.g11 =  -2318.00/pow(10,9);       // [T] Earth IGRF 2020 g_1^1 cooefficient
    this->dipoleParams.h11 =   5817.00/pow(10,9);       // [T] Earth IGRF 2020 h_1^1 cooefficient
    this->planetRadius = 6371.2*1000; // [m] Earth magnetic spherical reference radius (see p. 404 in doi:10.1007/978-1-4939-0802-8)

    this->relativePos_N.fill(0.0);
    this->scStateInMsgNames.clear();
    this->planetPosInMsgId = -1;

    //! - zero the planet message, and set the DCM to an identity matrix
    memset(&this->planetState, 0x0, sizeof(SpicePlanetStateSimMsg));
    this->planetState.J20002Pfix[0][0] = 1.0;
    this->planetState.J20002Pfix[1][1] = 1.0;
    this->planetState.J20002Pfix[2][2] = 1.0;

    return;
}

/*! Destructor.
 @return void
 */
MagneticField::~MagneticField()
{
    return;
}

/*! Sets the model used to compute magnetic field; must be set before init.
 @return void
 @param inputType The desired model type.
 */
void MagneticField::setEnvType(std::string inputType)
{
    this->envType = inputType;
    return;
}

/*! Sets the epoch date used by some models. This is converted automatically to the desired units.
 @return void
 @param julianDate The specified epoch date in JD2000.
 */
void MagneticField::setEpoch(double julianDate)
{
    this->epochDate = julianDate;
    return;
}

/*! Adds the spacecraft message name to a vector of sc message names and automatically creates an output message name.
    Must be called after ``setEnvType''.
 @return void
 @param tmpScMsgName A spacecraft state message name.
 */
void MagneticField::addSpacecraftToModel(std::string tmpScMsgName){
    std::string tmpEnvMsgName;
    this->scStateInMsgNames.push_back(tmpScMsgName);
    tmpEnvMsgName = this->envType + "_" + std::to_string(this->scStateInMsgNames.size()-1)+"_data";
    this->envOutMsgNames.push_back(tmpEnvMsgName);
    return;
}

/*! SelfInit for this method creates a seperate magnetic field message for each of the spacecraft
that were added using AddSpacecraftToModel. Additional model outputs are also initialized per-spacecraft.
 @return void
*/
void MagneticField::SelfInit()
{
    uint64_t tmpMagFieldMsgId;
    std::vector<std::string>::iterator it;

    //! - create all the environment output messages for each spacecraft
    for (it = this->envOutMsgNames.begin(); it!=this->envOutMsgNames.end(); it++) {
        tmpMagFieldMsgId = SystemMessaging::GetInstance()->CreateNewMessage(*it, sizeof(MagneticFieldSimMsg),
                this->OutputBufferCount, "MagneticFieldSimMsg", moduleID);
        this->envOutMsgIds.push_back(tmpMagFieldMsgId);
    }

    return;
}

/*! This method is used to connect the input position message from the spacecraft. Additonal model-specific cross inits are also conducted.
 @return void
 */
void MagneticField::CrossInit()
{
    //! - if a planet message name is specified, subscribe to this message. If not, then a zero planet position and orientation is assumed
    if (this->planetPosInMsgName.length() > 0) {
        this->planetPosInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->planetPosInMsgName,
                                                                                    sizeof(SpicePlanetStateSimMsg),
                                                                                    moduleID);
    }

    //! - subscribe to the spacecraft messages and create associated output message buffer
    std::vector<std::string>::iterator it;
    this->magFieldOutBuffer.clear();
    MagneticFieldSimMsg tmpMagField;
    memset(&tmpMagField, 0x0, sizeof(MagneticFieldSimMsg));
    for(it = this->scStateInMsgNames.begin(); it!=this->scStateInMsgNames.end(); it++){
        this->scStateInMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(SCPlusStatesSimMsg), moduleID));
        this->magFieldOutBuffer.push_back(tmpMagField);
    }


    return;
}


/*! This method is used to write the output magnetic field messages whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void MagneticField::WriteOutputMessages(uint64_t CurrentClock)
{
    MagneticFieldSimMsg tmpMagFieldOutMsg;
    std::vector<int64_t>::iterator it;
    std::vector<MagneticFieldSimMsg>::iterator magFieldIt;
    magFieldIt = this->magFieldOutBuffer.begin();
    for(it = this->envOutMsgIds.begin(); it!= this->envOutMsgIds.end(); it++, magFieldIt++){
        tmpMagFieldOutMsg = *magFieldIt;
        SystemMessaging::GetInstance()->WriteMessage(*it,
                                                  CurrentClock,
                                                  sizeof(MagneticFieldSimMsg),
                                                  reinterpret_cast<uint8_t*>(&tmpMagFieldOutMsg),
                                                  moduleID);
    }

    return;
}


/*! This method is used to read the incoming command message and set the
 associated spacecraft positions for computing the atmosphere.
 @return void
 */
bool MagneticField::ReadInputs()
{
    SCPlusStatesSimMsg scMsg;
    SingleMessageHeader localHeader;

    this->scStates.clear();

    //! -SC message reads
    bool scRead;
    if(this->scStateInMsgIds.size() > 0)
    {
        scRead = true;
        //! Iterate over spacecraft message ids
        std::vector<int64_t>::iterator it;
            for(it = scStateInMsgIds.begin(); it!= scStateInMsgIds.end(); it++){
                bool tmpScRead;
                memset(&scMsg, 0x0, sizeof(SCPlusStatesSimMsg));
                tmpScRead = SystemMessaging::GetInstance()->ReadMessage(*it, &localHeader,
                                                      sizeof(SCPlusStatesSimMsg),
                                                      reinterpret_cast<uint8_t*>(&scMsg),
                                                      moduleID);
                scRead = scRead && tmpScRead;

                this->scStates.push_back(scMsg);
            }
    } else {
        BSK_PRINT(MSG_ERROR, "Atmosphere model has no spacecraft added to it.\n");
        scRead = false;
    }

    //! - Planet message read
    bool planetRead = true;     // if no planet message is set, then a zero planet position, velocity and orientation is assumed
    if(planetPosInMsgId >= 0)
    {
        planetRead = SystemMessaging::GetInstance()->ReadMessage(this->planetPosInMsgId , &localHeader,
                                              sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&this->planetState), moduleID);
    }

    return(planetRead && scRead);
}

/*! This method is used to update the local magnetic field based on each spacecraft's position.
  @return void
 */
void MagneticField::updateLocalMagField(double currentTime)
{
    double tmpPosMag;           // [m] Magnitude of the spacecraft's current position
    std::vector<SCPlusStatesSimMsg>::iterator it;
    uint64_t atmoInd = 0;


    //! - loop over all the spacecraft
    std::vector<MagneticFieldSimMsg>::iterator magMsgIt;
    magMsgIt = this->magFieldOutBuffer.begin();
    for(it = scStates.begin(); it != scStates.end(); it++, atmoInd++, magMsgIt++){
        //! - Computes planet relative state vector
        this->updateRelativePos(&(this->planetState), &(*it));

        //! - compute spacecraft altitude above planet radius
        tmpPosMag = this->relativePos_N.norm();

        //! - zero the output message for each spacecraft by default
        memset(&(*magMsgIt), 0x0, sizeof(MagneticFieldSimMsg));

        //! - check if radius is in permissible range
        if(tmpPosMag > this->envMinReach &&
           (tmpPosMag < this->envMaxReach || this->envMaxReach < 0)) {
            //! - check for simple dipole model case
            if(this->envType.compare(MODEL_CENTERED_DIPOLE)==0){
                runCenteredDipole(tmpPosMag,  &(*magMsgIt));
            } else {
                BSK_PRINT(MSG_WARNING, "Magnetic field model not set. Skipping computation.\n")
            }
        }
    }

    return;
}

/*! This method is used to determine the spacecraft position vector relative to the planet.
 @param planetState A space planetstate message struct.
 @param scState A spacecraftPlusStates message struct.
 @return void
 */
void MagneticField::updateRelativePos(SpicePlanetStateSimMsg *planetState, SCPlusStatesSimMsg *scState)
{
    v3Subtract(scState->r_BN_N, planetState->PositionVector, this->relativePos_N.data());

    return;
}


/*! Computes the current local magnetic field for each spacecraft and writes their respective messages.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void MagneticField::UpdateState(uint64_t CurrentSimNanos)
{
    //! - clear the output buffer
    std::vector<MagneticFieldSimMsg>::iterator it;
    for(it = this->magFieldOutBuffer.begin(); it!= this->magFieldOutBuffer.end(); it++){
        memset(&(*it), 0x0, sizeof(MagneticFieldSimMsg));
    }

    //! - update local neutral density information
    if(this->ReadInputs())
    {
        updateLocalMagField(CurrentSimNanos*NANO2SEC);
    }

    //! - write out neutral density message
    WriteOutputMessages(CurrentSimNanos);

    return;
}

/*! This method is evaluates the centered dipole magnetic field model.
 @param radius [m] spacecraft orbit radius
 @param msg magnetic field message structure
 @return void
 */void MagneticField::runCenteredDipole(double radius, MagneticFieldSimMsg *msg)
{
    Eigen::Vector3d magField_E;         // [T] magnetic field in Earth fixed frame
    Eigen::Vector3d relativePos_E;      // [m] position vector of spacecraft relative to planet center in Earth fixed frame
    Eigen::Vector3d rHat_E;             // [] normalized position vector in E frame components
    Eigen::Vector3d m_E;                // [Am^2] planet dipole vector
    Eigen::Vector3d dipoleCoefficients; // [] The first 3 IGRF coefficient that define the magnetic dipole

    //! - convert spacecraft position vector in Earth-fixed vector components
    m33MultV3(this->planetState.J20002Pfix, this->relativePos_N.data(), relativePos_E.data());

    //! - compute normalized E-frame position vector
    rHat_E = relativePos_E.normalized();

    //! - compute magnetic field vector in E-frame components (see p. 405 in doi:10.1007/978-1-4939-0802-8)
    dipoleCoefficients << this->dipoleParams.g11, this->dipoleParams.h11, this->dipoleParams.g10;
    magField_E = pow(this->planetRadius/radius,3)*(3*rHat_E*rHat_E.dot(dipoleCoefficients) - dipoleCoefficients);

    //! - convert magnetic field vector in N-frame components and store in output message
    m33tMultV3(this->planetState.J20002Pfix, magField_E.data(), msg->magField_N);

    return;
}
