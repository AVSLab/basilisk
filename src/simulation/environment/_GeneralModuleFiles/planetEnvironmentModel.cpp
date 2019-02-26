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

#include "planetEnvironmentModel.h"
#include <string>

/*! This is the constructor, just setting the variables to zero */
PlanetEnvironmentModel::PlanetEnvironmentModel()
{
    // Initialize the environment
    this->planetPosInMsgName = "";
    this->envType = "None";
    return;
}

///*! This is the destructor, nothing to report here */
PlanetEnvironmentModel::~PlanetEnvironmentModel()
{
    return;
}

/*! This method defines an interface for adding spacecraft to a provided model.
 * The output message name is provided as "[env_type]+[_len(scStateInMsgNames)-1]_data.
 * This method appends a vector of both spacecraft input message names and environment output message names*/

