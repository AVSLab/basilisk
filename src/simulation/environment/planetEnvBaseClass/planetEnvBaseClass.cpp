/*
 ISC License

 Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "planetEnvBaseClass.h"

/*! This is the constructor, just setting the variables to zero */
PlanetEnvironmentModel::PlanetEnvironmentModel()
{
    // Initialize the environment

    return;
}

/*! This is the destructor, nothing to report here */
PlanetEnvironmentModel::~PlanetEnvironmentModel(){
    return;
}

/*! This method defines an interface for adding spacecraft to a provided model.
 * The output message name is provided as "[env_type]+[_len(scStateInMsgNames)-1]_data.
 * This method creates a vector of both spacecraft input message names and environment output message names*/
void PlanetEnvironmentModel::addSpacecraftToModel(std::string tmpScMsgName){
    std::string tmpEnvMsgName;
    this->scStateInMsgNames.push_back(tmpScMsgName);
    tmpAtmoMsgName = this->env_type + std::to_string(this->scStateInMsgNames.size()-1)+"_data";
    this->envOutMsgNames.push_back(tmpEnvMsgName);
    return;
}

/*! This method is used to read the incoming command message and set the
 associated spacecraft positions for computing the atmosphere.
 */
/*
bool PlanetEnvironmentModel::ReadScInputs()
{
    SCPlusStatesSimMsg tmpState;
    //! Begin method steps
    SingleMessageHeader localHeader;
    memset(&tmpState, 0x0, sizeof(SCPlusStatesSimMsg));
    this->scStates.clear();
    if(this->scStateInMsgIds[0] >= 0)
    {
        //! Iterate over spacecraft message ids
        std::vector<int64_t>::iterator it;
        for(it = scStateInMsgIds.begin(); it!= scStateInMsgIds.end(); it++){
            SystemMessaging::GetInstance()->ReadMessage(*it, &localHeader,
                                                        sizeof(SCPlusStatesSimMsg),
                                                        reinterpret_cast<uint8_t*>(&tmpState),
                                                        moduleID);
            this->scStates.push_back(tmpState);
        }
    }
}

/*! Optional method to read in the planet SPICE message if it has been set. */
/*
bool PlanetEnvironmentModel::ReadPlanetInputs()
{

    SingleMessageHeader localHeader;
    memset(&this->bodyState, 0x0, sizeof(SpicePlanetStateSimMsg));

    if(planetPosInMsgId >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(this->planetPosInMsgId , &localHeader,
                                                    sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&this->bodyState), moduleID);
    }
    return(true);
}*/