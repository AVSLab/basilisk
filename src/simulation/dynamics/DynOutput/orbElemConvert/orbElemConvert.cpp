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
#include "simulation/dynamics/DynOutput/orbElemConvert/orbElemConvert.h"
#include <iostream>
#include "architecture/utilities/linearAlgebra.h"

//! The constructor.  Note that you may want to overwrite the message names.
OrbElemConvert::OrbElemConvert()
{
    return;
}

//! The destructor.  So tired of typing this.
OrbElemConvert::~OrbElemConvert()
{
    return;
}


/*! This method is used to reset the module.

 */
void OrbElemConvert::Reset(uint64_t CurrentSimNanos)
{
    int numInputs = 0;      //!< number of input messages connected
    int numOutputs = 0;     //!< number of output messages connected

    numInputs += this->scStateInMsg.isLinked();
    numInputs += this->spiceStateInMsg.isLinked();
    numInputs += this->elemInMsg.isLinked();

    numOutputs += this->scStateOutMsg.isLinked();
    numOutputs += this->spiceStateOutMsg.isLinked();
    numOutputs += this->elemOutMsg.isLinked();

    if (numInputs == 0) {
        bskLogger.bskLog(BSK_ERROR, "No input message was connected.");
    }
    if (numOutputs == 0) {
        bskLogger.bskLog(BSK_ERROR, "No output message was connected.");
    }
    if (numInputs > 1) {
        bskLogger.bskLog(BSK_ERROR, "Found %d input messages.  There can be only one.", numInputs);
    }
}

/*! This method writes the output data out into the messaging system.  It does
 switch depending on whether it is outputting cartesian position/velocity or
 orbital elements.

 @param CurrentClock The current time in the system for output stamping
 */
void OrbElemConvert::WriteOutputMessages(uint64_t CurrentClock)
{
    if (this->elemOutMsg.isLinked() && this->inputsGood) {
        auto payload = ClassicElementsMsgPayload();
        payload.a = this->CurrentElem.a;
        payload.e = this->CurrentElem.e;
        payload.i = this->CurrentElem.i;
        payload.Omega = this->CurrentElem.Omega;
        payload.omega = this->CurrentElem.omega;
        payload.f = this->CurrentElem.f;
        payload.rmag = this->CurrentElem.rmag;
        payload.alpha = this->CurrentElem.alpha;
        payload.rPeriap = this->CurrentElem.rPeriap;
        payload.rApoap = this->CurrentElem.rApoap;
        this->elemOutMsg.write(&payload, this->moduleID, CurrentClock);
    }
    if (this->scStateOutMsg.isLinked() && this->inputsGood) {
        SCStatesMsgPayload scMsg;
        scMsg = this->scStateOutMsg.zeroMsgPayload;
        v3Copy(this->r_N, scMsg.r_BN_N);
        v3Copy(this->v_N, scMsg.v_BN_N);
        this->scStateOutMsg.write(&scMsg, this->moduleID, CurrentClock);
    }
    if (this->spiceStateOutMsg.isLinked() && this->inputsGood) {
        SpicePlanetStateMsgPayload spiceMsg;
        spiceMsg = this->spiceStateOutMsg.zeroMsgPayload;
        v3Copy(this->r_N, spiceMsg.PositionVector);
        v3Copy(this->v_N, spiceMsg.VelocityVector);
        this->spiceStateOutMsg.write(&spiceMsg, this->moduleID, CurrentClock);
    }
}

/*! The name kind of says it all right?  Converts CurrentElem to pos/vel.

 */
void OrbElemConvert::Elements2Cartesian()
{
    elem2rv(mu, &CurrentElem, r_N, v_N);
}

/*! The name kind of says it all right?  Converts pos/vel to CurrentElem.

 */
void OrbElemConvert::Cartesian2Elements()
{
    rv2elem(mu, r_N, v_N, &CurrentElem);
}

/*! This method reads the input message in from the system and sets the
 appropriate parameters based on which direction the module is running

 */
void OrbElemConvert::ReadInputs()
{
    this->inputsGood = false;
    if (this->elemInMsg.isLinked()) {
        auto elements = ClassicElements();
        auto inputElement = this->elemInMsg();
        elements.a = inputElement.a;
        elements.e = inputElement.e;
        elements.i = inputElement.i;
        elements.Omega = inputElement.Omega;
        elements.omega = inputElement.omega;
        elements.f = inputElement.f;
        elements.rmag = inputElement.rmag;
        elements.alpha = inputElement.alpha;
        elements.rPeriap = inputElement.rPeriap;
        elements.rApoap = inputElement.rApoap;
        this->CurrentElem = elements;
        this->inputsGood = this->elemInMsg.isWritten();
    }

    if (this->scStateInMsg.isLinked()) {
        this->statesIn = this->scStateInMsg();
        this->inputsGood = this->scStateInMsg.isWritten();
        v3Copy(this->statesIn.r_BN_N, this->r_N);
        v3Copy(this->statesIn.v_BN_N, this->v_N);
    }

    if (this->spiceStateInMsg.isLinked()) {
        this->planetIn = this->spiceStateInMsg();
        this->inputsGood = this->spiceStateInMsg.isWritten();
        v3Copy(this->planetIn.PositionVector, this->r_N);
        v3Copy(this->planetIn.VelocityVector, this->v_N);
    }
}

/*! This method is the main carrier for the conversion routine.  If it detects
 that it needs to re-init (direction change maybe) it will re-init itself.
 The it either converts elements to cartesian or cartesian to elements.

 @param CurrentSimNanos The current simulation time for system
 */
void OrbElemConvert::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read the input message and convert it over appropriately depending on switch
    ReadInputs();
    if(this->elemInMsg.isLinked() && inputsGood)
    {
        Elements2Cartesian();
    }
    else if(inputsGood)
    {
        Cartesian2Elements();
    }

    //! Write out the current output for current time
    WriteOutputMessages(CurrentSimNanos);
}
