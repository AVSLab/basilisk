/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef PLANET_ENV_H
#define PLANET_ENV_H

#include <Eigen/Dense>
#include <string>

/*! @brief Abstract class that is used to implement planet-relative "environment" models (for example: atmospheric density, magnetic fields, etc.) */
class PlanetEnvironmentModel {
public:
    PlanetEnvironmentModel();                      //!< -- Constructor
    virtual ~PlanetEnvironmentModel();             //!< -- Destructor
    virtual void setEnvType(std::string inputType)=0; //!< [string]
    virtual void setEpoch(double julianDate) = 0; //!< [JulianDate] Method to set epoch time for the module
    void addSpacecraftToModel(std::string tmpScMsgName);
    void ReadScInputs(); //! -- Iterate over scStateInMsgNames and get their information
    void ReadPlanetInput(); //! -- Read the current SPICE planet state

public:
    std::vector<std::string> scStateInMsgNames;	//!< Vector of the spacecraft position/velocity message names
    std::vector<std::string> envOutMsgNames; //!< Vector of message names to be written out by the environment
    std::string planetPosInMsgName;			//!< Message name for the planet's SPICE position message
    double envMinReach = 0; //!< [m] Minimum planet-relative position needed for the environment to work
    double envMaxReach = -1; //!< [m] Maximum distance at which the environment will be calculated
};

#endif /* SPACE_ENV_H */
