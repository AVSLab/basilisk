//
// Created by andrew on 2/18/19.
//

#include "planetEnvBaseClass.h"

/*! This is the constructor, just setting the variables to zero */
PlanetEnvironmentModel::PlanetEnvironmentModel()
{
    // Initialize the environment

    return;
}

/*! This is the destructor, nothing to report here */
DynamicEffector::~DynamicEffector()
{
    return;
}

/*! This method defines an interface for adding spacecraft to a provided model. */

void ExponentialAtmosphere::addSpacecraftToModel(std::string tmpScMsgName){
    std::string tmpEnvMsgName;
    this->scStateInMsgNames.push_back(tmpScMsgName);
    tmpAtmoMsgName = this->env_type + std::to_string(this->scStateInMsgNames.size()-1)+"_data";
    this->envOutMsgNames.push_back(tmpEnvMsgName);
    return;
}