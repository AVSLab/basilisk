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
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/geodeticConversion.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

/*! This method initializes some basic parameters for the module.

 */
MsisAtmosphere::MsisAtmosphere()
{
    //! - Set the default atmospheric properties
    this->planetRadius = REQ_EARTH*1000.;   // must be the radius of Earth for MSIS

    this->epochDoy = -1;                     // negative value means this is not set

    this->f107A = 0.0;
    this->f107 = 0.0;
    this->ap = 0.0;
    for(int apInd = 0; apInd < 7; ++apInd) {
        this->aph.a[apInd] = 0.0;
    }
    this->msisInput.ap_a = &this->aph;
    this->updateInputParams();


    this->msisFlags.switches[0] = 1; //! NRLMSISE-00 should output in kg/m^3 for consistency with other atmospheric modules.
    //! Set default settings for NRLMSISE-00; we're using all the settings by default
    for(int switchInd = 1; switchInd < 24; ++switchInd){
        this->msisFlags.switches[switchInd] = 1;
    }

    // Set other required interface values
    for (int c = 0; c < 23; c++) {
        ReadFunctor<SwDataMsgPayload> *msgIn;
        msgIn = new ReadFunctor<SwDataMsgPayload>;
        this->swDataInMsgs.push_back(*msgIn);
    }
    /* the above message order is
         0 - ap_24_0
         1 - ap_3_0
         2 - ap_3_-3
         3 - ap_3_-6
         4 - ap_3_-9
         5 - ap_3_-12
         6 - ap_3_-15
         7 - ap_3_-18
         8 - ap_3_-21
         9 - ap_3_-24
        10 - ap_3_-27
        11 - ap_3_-30
        12 - ap_3_-33
        13 - ap_3_-36
        14 - ap_3_-39
        15 - ap_3_-42
        16 - ap_3_-45
        17 - ap_3_-48
        18 - ap_3_-51
        19 - ap_3_-54
        20 - ap_3_-57
        21 - f107_1944_0
        22 - f107_24_-24

     */

    return;
}

/*! Destructor.

 */
MsisAtmosphere::~MsisAtmosphere()
{
    return;
}


/*! This method is used to reset the module.

 */
void MsisAtmosphere::customReset(uint64_t CurrentSimNanos)
{
    for(int ind = 0; ind < 23; ind++) {
        if (!this->swDataInMsgs[ind].isLinked()) {
            bskLogger.bskLog(BSK_ERROR, "Required MSIS input messages No. %d are not connected.", ind);
        }
    }
}


/*! Custom customSetEpochFromVariable() method.  This allows specifying epochDoy directly from Python.  If an epoch message is set then this variable is not used.

 */
void MsisAtmosphere::customSetEpochFromVariable()
{
    //! - only convert if the day-in-year variable was set to a non-zero value.  Otherwise use the BSK epoch default setup by the base class.
    if (this->epochDoy > 0.0) {
        /* here the BSK default epoch year is used on Jan 1, mid-night, and the requested days of year are added and converted to a proper date-time structure */
        this->epochDateTime.tm_mday = this->epochDoy;  // assumes 1 is first day of year
        mktime(&this->epochDateTime);
    }

    return;
}


/*! This method is used to write the output densities whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message

 */
void MsisAtmosphere::customWriteMessages(uint64_t CurrentClock)
{
        /* [WIP] - Include additional outputs for other MSISE outputs (species count, etc.)*/

}


/*! This method is used to read the incoming command message and set the
 associated spacecraft positions for computing the atmosphere.

 */
bool MsisAtmosphere::customReadMessages(){
    bool swRead = false;
    int failCount = 0;
    SwDataMsgPayload tmpSwData;
    this->swDataList.clear();

    //! Iterate over swData message ids
    for(int ind = 0; ind < 23; ind++) {
        tmpSwData = this->swDataInMsgs.at(ind)();
        swRead = this->swDataInMsgs.at(ind).isWritten();
        if (swRead) {
            this->swDataList.push_back(tmpSwData);
        } else {
            failCount = failCount + 1;
        }
    }

    if (failCount > 0) {
        swRead = false;
    }
    return(swRead);
}


void MsisAtmosphere::updateInputParams()
{
    this->msisInput.ap = this->ap;
    this->msisInput.ap_a->a[0] = this->aph.a[0];
    this->msisInput.ap_a->a[1] = this->aph.a[1];
    this->msisInput.ap_a->a[2] = this->aph.a[2];
    this->msisInput.ap_a->a[3] = this->aph.a[3];
    this->msisInput.ap_a->a[4] = this->aph.a[4];
    this->msisInput.ap_a->a[5] = this->aph.a[5];
    this->msisInput.ap_a->a[6] = this->aph.a[6];

    this->msisInput.f107A = this->f107A;
    this->msisInput.f107 = this->f107;

}

void MsisAtmosphere::updateSwIndices()
{
    this->ap = this->swDataList[0].dataValue;
    this->aph.a[0] = this->swDataList[0].dataValue;
    this->aph.a[1] = this->swDataList[1].dataValue;
    this->aph.a[2] = this->swDataList[2].dataValue;
    this->aph.a[3] = this->swDataList[3].dataValue;
    this->aph.a[4] = this->swDataList[4].dataValue;

    uint64_t mth;
    double tmp_avg = 0.0;
    for(mth = 5; mth < 13; mth++){
        tmp_avg = tmp_avg + this->swDataList[mth].dataValue;
    }
    this->aph.a[5] = tmp_avg/8.0;

    uint64_t nth;
    tmp_avg = 0.0;
    for(nth = 13; nth < 21; nth++){
        tmp_avg = tmp_avg + this->swDataList[nth].dataValue;
    }
    this->aph.a[6] = tmp_avg/8.0;

    this->f107A = this->swDataList[21].dataValue;
    this->f107 = this->swDataList[22].dataValue;

}

void MsisAtmosphere::evaluateAtmosphereModel(AtmoPropsMsgPayload *msg, double currentTime)
{
    this->updateSwIndices();
    this->updateInputParams();
    //! Compute the geodetic position using the planet orientation.

    this->currentLLA = PCI2LLA(this->r_BP_N, this->planetState.J20002Pfix, this->planetRadius);
    this->msisInput.g_lat = R2D*this->currentLLA[0];
    this->msisInput.g_long = R2D*this->currentLLA[1];
    this->msisInput.alt = this->currentLLA[2]/1000.0; // NRLMSISE Altitude input must be in kilometers!

    //! Update time.
    struct tm localDateTime;                            // []       date/time structure
    localDateTime = this->epochDateTime;
    localDateTime.tm_sec += (int) currentTime;   // sets the current seconds
    mktime(&localDateTime);
    this->msisInput.year = localDateTime.tm_year + 1900;
    this->msisInput.doy = localDateTime.tm_yday + 1;    // Jan 1 is the 1st day of year, not 0th
    double fracSecond = currentTime - (int) currentTime;
    this->msisInput.sec = localDateTime.tm_hour * 3600.0 + localDateTime.tm_min * 60.0 + localDateTime.tm_sec + fracSecond;

    // WIP - need to actually figure out how to pull in these values.
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
