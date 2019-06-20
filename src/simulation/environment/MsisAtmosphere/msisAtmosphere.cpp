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

#include "msisAtmosphere.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "utilities/bsk_Print.h"
#include "utilities/linearAlgebra.h"
#include "utilities/geodeticConversion.h"
#include "../../dynamics/_GeneralModuleFiles/stateData.h"
#include "../../_GeneralModuleFiles/sys_model.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/*! This method initializes some basic parameters for the module.
 @return void
 */
MsisAtmosphere::MsisAtmosphere()
{
    //! - Set the default atmospheric properties to yield a zero response
    this->baseDensity = 0.0;            // [T]
    this->scaleHeight = 1.0;            // [m]
    this->planetRadius = 0.0;   // [m]
    this->localTemp = 1.0; // [K]

    this->defaultMsisInitialConditions();

    return;
}

/*! Destructor.
 @return void
 */
MsisAtmosphere::~MsisAtmosphere()
{
    return;
}

/*! Sets the model used to compute atmospheric density/temperature; must be set before init.
 @return void
 @param inputType The desired model type.
 */
void MsisAtmosphere::defaultMsisInitialConditions()
{
        this->year = 1984;
        this->doy = 1;
        this->sec = 0.0;
        this->alt = 0.0;
        this->g_lat = 0.0;
        this->g_long = 0.0;
        this->lst = 0.0;
        this-> f107A = 0.0;
        this->f107 = 0.0;
        this->ap = 0.0;

        for(int apInd = 0; apInd < 7; ++apInd) {
            this->aph.a[apInd] = 0.0;
        }
        this->msisInput.year = 1984;
        this->msisInput.doy = 1;
        this->msisInput.sec = 0.0;
        this->msisInput.alt = 0.0;
        this->msisInput.g_lat = 0.0;
        this->msisInput.g_long = 0.0;
        this->msisInput.lst = 0.0;
        this->msisInput.f107A = 0.0;
        this->msisInput.f107 = 0.0;
        this->msisInput.ap = 0.0;
        this->msisInput.ap_a = &this->aph;

        this->updateInputParams();


        this->msisFlags.switches[0] = 0;
        //! Set default settings for NRLMSISE-00; we're using all the settings by default
        for(int switchInd = 1; switchInd < 24; ++switchInd){
            this->msisFlags.switches[switchInd] = 1;
        }

        this->startDoy = 1;
        this->startTime = 0.0;

        // Set other required interface values

        this->swDataInMsgNames.push_back("ap_24_0");
        this->swDataInMsgNames.push_back("ap_3_0");
        this->swDataInMsgNames.push_back("ap_3_-3");
        this->swDataInMsgNames.push_back("ap_3_-6");
        this->swDataInMsgNames.push_back("ap_3_-9");

        this->swDataInMsgNames.push_back("ap_3_-12");
        this->swDataInMsgNames.push_back("ap_3_-15");
        this->swDataInMsgNames.push_back("ap_3_-18");
        this->swDataInMsgNames.push_back("ap_3_-21");
        this->swDataInMsgNames.push_back("ap_3_-24");
        this->swDataInMsgNames.push_back("ap_3_-27");
        this->swDataInMsgNames.push_back("ap_3_-30");
        this->swDataInMsgNames.push_back("ap_3_-33");

        this->swDataInMsgNames.push_back("ap_3_-36");
        this->swDataInMsgNames.push_back("ap_3_-39");
        this->swDataInMsgNames.push_back("ap_3_-42");
        this->swDataInMsgNames.push_back("ap_3_-45");
        this->swDataInMsgNames.push_back("ap_3_-48");
        this->swDataInMsgNames.push_back("ap_3_-51");
        this->swDataInMsgNames.push_back("ap_3_-54");
        this->swDataInMsgNames.push_back("ap_3_-57");

        this->swDataInMsgNames.push_back("f107_1944_0");
        this->swDataInMsgNames.push_back("f107_24_-24");

    return;
}

/*! Sets the epoch date used by some models. This is converted automatically to the desired units.
 @return void
 @param julianDate The specified epoch date in JD2000.
 */
void MsisAtmosphere::setEpoch(double julianDate)
{
    this->epochDate = julianDate;
    return;
}


/*! SelfInit for this method creates a seperate density message for each of the spacecraft
that were added using AddSpacecraftToModel. Additional model outputs are also initialized per-spacecraft.
 @return void
*/
void MsisAtmosphere::customSelfInit()
{
    if(this->envType.compare(MODEL_MSISE)==0) {
        BSK_PRINT(MSG_WARNING, "NRLMSISE-00 specific messages are not implemented..\n")
    }
    return;
}

/*! This method is used to connect the input position message from the spacecraft. Additonal model-specific cross inits are also conducted.
 @return void
 */
void MsisAtmosphere::customCrossInit()
{

        //* [WIP] Also do MSISE messaging setup*//
    for(int ind=0; ind < 23; ind++){
        this->swDataInMsgIds[ind] = SystemMessaging::GetInstance()->subscribeToMessage(this->swDataInMsgNames[ind], sizeof(SwDataSimMsg), moduleID);
    }


    return;
}


/*! This method is used to write the output densities whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void MsisAtmosphere::customWriteMessages(uint64_t CurrentClock)
{
        /* [WIP] - Include additional outputs for other MSISE outputs (species count, etc.)*/
    }
}


/*! This method is used to read the incoming command message and set the
 associated spacecraft positions for computing the atmosphere.
 @return void
 */
bool MsisAtmosphere::customReadMessages(){
    bool swRead = false;
    int failCount = 0;
    SwDataSimMsg tmpSwData;
    /* WIP - Also read in all the MSISE inputs.*/
    //! Iterate over swData message ids
    for(int ind = 0; ind < 23; ind++) {
        if (this->swDataInMsgIds[ind] >= 0) {
            swRead = SystemMessaging::GetInstance()->ReadMessage(this->swDataInMsgIds[ind], &localHeader,
                                                                    sizeof(SwDataSimMsg),
                                                                    reinterpret_cast<uint8_t *>(&tmpSwData),
                                                                    moduleID);
            if (good_read) {
                this->swDataList.push_back(tmpSwData);
            } else {
                failCount = failCount + 1;
            }
        }
    }
    if (failCount > 0) {
        swRead = false;
    }
    return(swRead);
}


void MsisAtmosphere::evaluateAtmosphereModel(AtmoPropsSimMsg *msg)
{
    this->updateSwIndices();
    this->updateInputParams();
    //! Compute the geodetic position using the planet orientation.

    this->currentLLA = PCI2LLA(this->relativePos_N, this->planetState.J20002Pfix, this->planetRadius);
    this->msisInput.g_lat = R2D*this->currentLLA[0];
    this->msisInput.g_long = R2D*this->currentLLA[1];
    this->msisInput.alt = this->currentLLA[2]/1000.0; // NRLMSISE Altitude input must be in kilometers!

    //! Update time.
    this->msisInput.doy = this->startDoy + floor(currentTime * 1E-09 * (1.0/86400));
    this->msisInput.sec = this->startTime + currentTime;

    //WIP - need to actually figure out how to pull in these values.
    this->msisInput.lst = this->msisInput.sec/3600.0 + this->msisInput.g_long/15.0;

    //!  NRLMSISE-00 uses different models depending on the altitude.
    if(this->msisInput.alt < 500.0){
        gtd7(&this->msisInput, \
       &this->msisFlags, \
       &this->msisOutput);
    }

        /* GTD7D */
        /*   This subroutine provides Effective Total Mass Density for output
         *   d[5] which includes contributions from "anomalous oxygen" which can
         *   affect satellite drag above 500 km. See the section "output" for
         *   additional details.
         */
    else if(this->msisInput.alt >= 500.0){
        gtd7d(&this->msisInput, \
       &this->msisFlags, \
       &this->msisOutput);
    }
    msg->neutralDensity = this->msisOutput.d[5];
    msg->localTemp = this->msisOutput.t[1];
    return;
}
