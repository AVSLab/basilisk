/*
 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "cielimInterface.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

#include <fstream>
#include <google/protobuf/io/coded_stream.h>
#include <iostream>

void message_buffer_deallocate(void *data, void *hint);

/*! CielimInterface Constructor */
CielimInterface::CielimInterface() = default;

/*! CielimInterface Destructor */
CielimInterface::~CielimInterface() {
    for (const auto & opnavImageOutMsg : this->opnavImageOutMsgs) {
        delete opnavImageOutMsg;
    }
}

/*! A Reset method to put the module back into a clean state
 @param CurrentSimNanos The current sim time in nanoseconds
*/
void CielimInterface::Reset(uint64_t CurrentSimNanos) {
    if (this->opNavMode != ClosedLoopMode::OPEN_LOOP || this->liveStream) {
        for (size_t camCounter = 0; camCounter < this->cameraConfInMsgs.size(); camCounter++) {
            this->bskImagePtrs[camCounter] = nullptr;
        }
        if (!this->connector.isConnected()) this->connector.connect();
        bskLogger.bskLog(BSK_INFORMATION, "Cielim-Vizard connection made");
    }

    for (auto & scIt : this->scData) {
        /* Check spacecraft input message */
        if (scIt.scStateInMsg.isLinked()) {
            scIt.scStateInMsgStatus.dataFresh = false;
            scIt.scStateInMsgStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        } else {
            bskLogger.bskLog(BSK_ERROR, "vizInterface: spacecraft msg not linked.");
        }

        /* Check CSS data input messages */
        {
            MsgCurrStatus cssStatus;
            cssStatus.dataFresh = false;
            cssStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
            scIt.cssConfLogInMsgStatus.clear();
            scIt.cssInMessage.clear();
            for (size_t idx = 0; idx < scIt.cssInMsgs.size(); idx++) {
                if (!scIt.cssInMsgs.at(idx).isLinked()) {
                    bskLogger.bskLog(BSK_ERROR, "vizInterface: CSS(%zu) msg not linked.", idx);
                }
                scIt.cssConfLogInMsgStatus.push_back(cssStatus);
                CSSConfigLogMsgPayload logMsg = {};
                scIt.cssInMessage.push_back(logMsg);
            }
        }

        /* Check RW input message */
        {
            MsgCurrStatus rwStatus;
            rwStatus.dataFresh = false;
            rwStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
            scIt.rwInMsgStatus.clear();
            scIt.rwInMessage.clear();
            for (size_t idx = 0; idx < scIt.rwInMsgs.size(); idx++) {
                if (!scIt.rwInMsgs.at(idx).isLinked()) {
                    bskLogger.bskLog(BSK_ERROR, "vizInterface: RW(%zu) msg not linked.", idx);
                }
                scIt.rwInMsgStatus.push_back(rwStatus);
                RWConfigLogMsgPayload logMsg = {};
                scIt.rwInMessage.push_back(logMsg);
            }
        }

        /* Check Thr input message */
        {
            MsgCurrStatus thrStatus;
            thrStatus.dataFresh = false;
            thrStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;

            for (size_t thrCounter = 0; thrCounter < scIt.thrInMsgs.size(); thrCounter++) {
                if (scIt.thrInMsgs.at(thrCounter).isLinked()) {
                    scIt.thrMsgStatus.push_back(thrStatus);
                    THROutputMsgPayload logMsg;
                    scIt.thrOutputMessage.push_back(logMsg);
                } else {
                    bskLogger.bskLog(BSK_ERROR, "vizInterface: TH(%d) msg requested but not found.", (int)thrCounter);
                }
            }
            if (scIt.thrInfo.size() != scIt.thrInMsgs.size()) {
                bskLogger.bskLog(BSK_ERROR,
                                 "vizInterface: thrInfo vector (%d) must be the same size as thrInMsgs (%d)",
                                 (int)scIt.thrInfo.size(),
                                 (int)scIt.thrInMsgs.size());
            }
        }
    }

    /* Check Camera input messages */
    for (size_t camCounter = 0; camCounter < this->cameraConfInMsgs.size(); camCounter++) {
        if (this->cameraConfInMsgs[camCounter].isLinked()) {
            this->cameraConfMsgStatus[camCounter].dataFresh = false;
            this->cameraConfMsgStatus[camCounter].lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        }
    }

    /* Check Spice input message */
    {
        MsgCurrStatus spiceStatus;
        spiceStatus.dataFresh = true;  // this ensures that default planet states are also used
        spiceStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        this->spiceInMsgStatus.clear();
        this->spiceMessage.clear();
        for (long unsigned int c = 0; c < this->gravBodyInformation.size(); c++) {
            /* set default zero translation and rotation states */
            SpicePlanetStateMsgPayload logMsg = {};
            m33SetIdentity(logMsg.J20002Pfix);
            strcpy(logMsg.PlanetName, this->gravBodyInformation.at(c).bodyName.c_str());

            this->spiceInMsgStatus.push_back(spiceStatus);
            this->spiceMessage.push_back(logMsg);
        }
    }

    this->frameNumber = -1;
    if (this->saveFile) {
        this->outputStream = new std::ofstream(this->protoFilename, std::ios::out | std::ios::binary);
    }

    this->settings.dataFresh = true;        // reset flag to transmit Vizard settings

    this->epochMsgBuffer.year = EPOCH_YEAR;
    this->epochMsgBuffer.month = EPOCH_MONTH;
    this->epochMsgBuffer.day = EPOCH_DAY;
    this->epochMsgBuffer.hours = EPOCH_HOUR;
    this->epochMsgBuffer.minutes = EPOCH_MIN;
    this->epochMsgBuffer.seconds = EPOCH_SEC;
    this->epochMsgStatus.dataFresh = true;
}

/*! A method in which the module reads the content of all available bsk messages
 */
void CielimInterface::readBskMessages() {
    for (auto scIt = this->scData.begin(); scIt != this->scData.end(); scIt++) {
        /* Read BSK spacecraft state msg */
        if (scIt->scStateInMsg.isLinked()) {
            SCStatesMsgPayload localSCStateArray;
            localSCStateArray = scIt->scStateInMsg();
            if (scIt->scStateInMsg.isWritten() &&
                scIt->scStateInMsg.timeWritten() != scIt->scStateInMsgStatus.lastTimeTag) {
                scIt->scStateInMsgStatus.lastTimeTag = scIt->scStateInMsg.timeWritten();
                scIt->scStateInMsgStatus.dataFresh = true;
            }
            scIt->scStateMsgBuffer = localSCStateArray;
        }

        /* Read BSK RW constellation msg */
        this->readRWConstellationMessages(*scIt);

        /* Read incoming Thruster constellation msg */
        this->readThrusterMessages(*scIt);

        /* Read CSS constellation log msg */
        this->readCSSMessages(*scIt);

        /* read in true trajectory line color if connected */
        if (scIt->trueTrajectoryLineColorInMsg.isLinked()) {
            if (scIt->trueTrajectoryLineColorInMsg.isWritten()) {
                ColorMsgPayload colorMsg;
                colorMsg = scIt->trueTrajectoryLineColorInMsg();
                scIt->trueTrajectoryLineColor.clear();
                for (int i : colorMsg.colorRGBA) {
                    scIt->trueTrajectoryLineColor.push_back(i);
                }
            }
        }

        /* read in generic sensor cmd value */
        {
            for (size_t idx = 0; idx < scIt->genericSensorList.size(); idx++) {
                if (scIt->genericSensorList[idx]->genericSensorCmdInMsg.isLinked()) {
                    DeviceCmdMsgPayload deviceCmdMsgBuffer;
                    deviceCmdMsgBuffer = scIt->genericSensorList[idx]->genericSensorCmdInMsg();
                    if (scIt->genericSensorList[idx]->genericSensorCmdInMsg.isWritten()) {
                        scIt->genericSensorList[idx]->genericSensorCmd = deviceCmdMsgBuffer.deviceCmd;
                    }
                }
            }
        }

        /* read in light on/off cmd value */
        {
            for (size_t idx = 0; idx < scIt->lightList.size(); idx++) {
                if (scIt->lightList[idx]->onOffCmdInMsg.isLinked()) {
                    DeviceCmdMsgPayload onOffCmdMsgBuffer;
                    onOffCmdMsgBuffer = scIt->lightList[idx]->onOffCmdInMsg();
                    if (scIt->lightList[idx]->onOffCmdInMsg.isWritten()) {
                        scIt->lightList[idx]->lightOn = (int) onOffCmdMsgBuffer.deviceCmd;
                    }
                }
            }
        }

        /* read in transceiver state values */
        {
            for (auto transceiver : scIt->transceiverList) {
                if (transceiver->transceiverStateInMsgs.size() > 0) {
                    transceiver->transceiverState = 0;
                    for (size_t idxTr = 0; idxTr < transceiver->transceiverStateInMsgs.size(); idxTr++) {
                        if (transceiver->transceiverStateInMsgs[idxTr].isLinked()) {
                            DataNodeUsageMsgPayload stateMsgBuffer;
                            stateMsgBuffer = transceiver->transceiverStateInMsgs[idxTr]();
                            if (transceiver->transceiverStateInMsgs[idxTr].isWritten()) {
                                /* state 0->off, 1->sending, 2->receiving, 3->sending and receiving */
                                if (stateMsgBuffer.baudRate < 0.0) {
                                    /* sending data */
                                    transceiver->transceiverState = transceiver->transceiverState | 1;
                                } else if (stateMsgBuffer.baudRate > 0.0) {
                                    /* receiving data */
                                    transceiver->transceiverState = transceiver->transceiverState | 2;
                                }
                            }
                        }
                    }
                }
            }
        }

        /* read in generic storage state values */
        {
            for (auto genericStorageItem : scIt->genericStorageList) {
                /* read in battery device state */
                if (genericStorageItem->batteryStateInMsg.isLinked()) {
                    PowerStorageStatusMsgPayload deviceStateMsgBuffer;
                    deviceStateMsgBuffer = genericStorageItem->batteryStateInMsg();
                    if (genericStorageItem->batteryStateInMsg.isWritten()) {
                        genericStorageItem->currentValue = deviceStateMsgBuffer.storageLevel;
                        genericStorageItem->maxValue = deviceStateMsgBuffer.storageCapacity;
                    }
                }
                /* read in data storage device state */
                if (genericStorageItem->dataStorageStateInMsg.isLinked()) {
                    DataStorageStatusMsgPayload deviceStateMsgBuffer;
                    deviceStateMsgBuffer = genericStorageItem->dataStorageStateInMsg();
                    if (genericStorageItem->dataStorageStateInMsg.isWritten()) {
                        genericStorageItem->currentValue = deviceStateMsgBuffer.storageLevel;
                        genericStorageItem->maxValue = deviceStateMsgBuffer.storageCapacity;
                    }
                }
                /* read in fuel tank device state */
                if (genericStorageItem->fuelTankStateInMsg.isLinked()) {
                    FuelTankMsgPayload deviceStateMsgBuffer;
                    deviceStateMsgBuffer = genericStorageItem->fuelTankStateInMsg();
                    if (genericStorageItem->fuelTankStateInMsg.isWritten()) {
                        genericStorageItem->currentValue = deviceStateMsgBuffer.fuelMass;
                        genericStorageItem->maxValue = deviceStateMsgBuffer.maxFuelMass;
                    }
                }
            }
        }

        /* read in MSM charge values */
        {
            /* read in MSM charge states */
            if (scIt->msmInfo.msmChargeInMsg.isLinked()) {
                if (scIt->msmInfo.msmChargeInMsg.isWritten()) {
                    ChargeMsmMsgPayload msmChargeMsgBuffer;
                    msmChargeMsgBuffer = scIt->msmInfo.msmChargeInMsg();
                    if (msmChargeMsgBuffer.q.size() == scIt->msmInfo.msmList.size()) {
                        for (size_t idx = 0; idx < scIt->msmInfo.msmList.size(); idx++) {
                            scIt->msmInfo.msmList[idx]->currentValue = msmChargeMsgBuffer.q[idx];
                        }
                    } else {
                        bskLogger.bskLog(BSK_ERROR,
                                         "vizInterface: the number of charged in MSM message and the number of"
                                         " msm vizInterface spheres must be the same.");
                    }
                }
            }
        }
    } /* end of scIt loop */

    /*! Read incoming camera config msg */
    for (size_t camCounter = 0; camCounter < this->cameraConfInMsgs.size(); camCounter++) {
        if (this->cameraConfInMsgs[camCounter].isLinked()) {
            CameraConfigMsgPayload localCameraConfigArray;
            localCameraConfigArray = this->cameraConfInMsgs[camCounter]();
            if (this->cameraConfInMsgs[camCounter].isWritten() &&
                this->cameraConfInMsgs[camCounter].timeWritten() != this->cameraConfMsgStatus[camCounter].lastTimeTag) {
                this->cameraConfMsgStatus[camCounter].lastTimeTag = this->cameraConfInMsgs[camCounter].timeWritten();
                this->cameraConfMsgStatus[camCounter].dataFresh = true;
            }
            this->cameraConfigBuffers[camCounter] = localCameraConfigArray;
        }
    }

    /*! Read incoming epoch msg */
    if (this->epochInMsg.isLinked()) {
        EpochMsgPayload epochMsg_Buffer;
        epochMsg_Buffer = this->epochInMsg();
        if (this->epochInMsg.isWritten() &&
            this->epochInMsg.timeWritten() != this->epochMsgStatus.lastTimeTag) {
            this->epochMsgStatus.lastTimeTag = this->epochInMsg.timeWritten();
            this->epochMsgStatus.dataFresh = true;
            this->epochMsgBuffer = epochMsg_Buffer;
        }
    }

    /*! Read BSK Spice constellation msg */
    {
        for (size_t i = 0; i < this->spiceInMsgs.size(); i++) {
            if (this->spiceInMsgs.at(i).isLinked()) {
                // If the spice msg is not linked then the default zero planet ephemeris is used
                SpicePlanetStateMsgPayload localSpiceArray;
                localSpiceArray = this->spiceInMsgs.at(i)();
                if (this->spiceInMsgs.at(i).isWritten() &&
                    this->spiceInMsgs.at(i).timeWritten() != this->spiceInMsgStatus[i].lastTimeTag) {
                    this->spiceInMsgStatus[i].lastTimeTag = this->spiceInMsgs.at(i).timeWritten();
                    this->spiceInMsgStatus[i].dataFresh = true;
                    this->spiceMessage[i] = localSpiceArray;
                }
            }
        }
    }
}

/*! The method in which the vizInterface writes a protobuffer with the information from the simulation.
 @param CurrentSimNanos The current sim time in nanoseconds
 */
void CielimInterface::writeProtobuffer(uint64_t CurrentSimNanos) {
    auto *visPayload = new vizProtobufferMessage::VizMessage;

    /*! Send the Vizard settings once */
    if (this->settings.dataFresh) {
        auto vizSettings = this->collectVizSettings();
        visPayload->set_allocated_settings(vizSettings);
        this->settings.dataFresh = false;
    }

    /*! Send the Vizard live settings */
    auto liveVizSettings = this->collectVizLiveSettings();
    visPayload->set_allocated_livesettings(liveVizSettings);

    /*! Write timestamp output msg */
    auto *time = new vizProtobufferMessage::VizMessage::TimeStamp;
    time->set_framenumber(this->frameNumber);
    time->set_simtimeelapsed((double) CurrentSimNanos);
    visPayload->set_allocated_currenttime(time);

    /*! write epoch msg */
    if (this->epochMsgStatus.dataFresh) {
        auto *epoch = new vizProtobufferMessage::VizMessage::EpochDateTime;
        epoch->set_year(this->epochMsgBuffer.year);
        epoch->set_month(this->epochMsgBuffer.month);
        epoch->set_day(this->epochMsgBuffer.day);
        epoch->set_hours(this->epochMsgBuffer.hours);
        epoch->set_minutes(this->epochMsgBuffer.minutes);
        epoch->set_seconds(this->epochMsgBuffer.seconds);
        visPayload->set_allocated_epoch(epoch);
        this->epochMsgStatus.dataFresh = false;
    }

    /*! write the Locations protobuf messages */
    for (auto & location : this->locations) {
        vizProtobufferMessage::VizMessage::Location *glp = visPayload->add_locations();
        glp->set_stationname(location->stationName);
        glp->set_parentbodyname(location->parentBodyName);
        glp->set_fieldofview(location->fieldOfView * R2D);
        glp->set_range(location->range);
        for (int i = 0; i < 3; i++) {
            glp->add_r_gp_p(location->r_GP_P[i]);
            glp->add_ghat_p(location->gHat_P[i]);
        }
        for (int i : location->color) {
            glp->add_color(i);
        }
    }

    for (auto & scIt : scData) {
        /*! Write spacecraft state output msg */
        if (scIt.scStateInMsg.isLinked() && scIt.scStateInMsgStatus.dataFresh) {
            vizProtobufferMessage::VizMessage::Spacecraft *scp = visPayload->add_spacecraft();
            scp->set_spacecraftname(scIt.spacecraftName);
            scp->set_parentspacecraftname(scIt.parentSpacecraftName);
            for (int i = 0; i < 3; i++) {
                scp->add_position(scIt.scStateMsgBuffer.r_BN_N[i]);
                scp->add_velocity(scIt.scStateMsgBuffer.v_BN_N[i]);
                scp->add_rotation(scIt.scStateMsgBuffer.sigma_BN[i]);
            }

            /* Write the SC sprite string */
            scp->set_spacecraftsprite(scIt.spacecraftSprite);

            /*! Write RW output msg */
            for (size_t idx = 0; idx < scIt.rwInMsgs.size(); idx++) {
                if (scIt.rwInMsgs[idx].isLinked() && scIt.rwInMsgStatus[idx].dataFresh) {
                    vizProtobufferMessage::VizMessage::ReactionWheel *rwheel = scp->add_reactionwheels();
                    rwheel->set_wheelspeed(scIt.rwInMessage[idx].Omega);
                    rwheel->set_maxspeed(scIt.rwInMessage[idx].Omega_max);
                    rwheel->set_wheeltorque(scIt.rwInMessage[idx].u_current);
                    rwheel->set_maxtorque(scIt.rwInMessage[idx].u_max);
                    for (int i = 0; i < 3; i++) {
                        rwheel->add_position(scIt.rwInMessage[idx].rWB_B[i]);
                        rwheel->add_spinaxisvector(scIt.rwInMessage[idx].gsHat_B[i]);
                    }
                }
            }

            /*! Write Thr output msg */
            for (size_t idx = 0; idx < scIt.thrInMsgs.size(); idx++) {
                if (scIt.thrInMsgs[idx].isLinked() && scIt.thrMsgStatus[idx].dataFresh) {
                    vizProtobufferMessage::VizMessage::Thruster *thr = scp->add_thrusters();
                    thr->set_maxthrust(scIt.thrOutputMessage[idx].maxThrust);
                    thr->set_currentthrust(scIt.thrOutputMessage[idx].thrustForce);
                    thr->set_thrustertag(scIt.thrInfo[idx].thrTag);
                    if (scIt.thrInfo[idx].color[0] >= 0) {
                        for (int i : scIt.thrInfo[idx].color) {
                            thr->add_color(i);
                        }
                    }
                    for (int i = 0; i < 3; i++) {
                        thr->add_position(scIt.thrOutputMessage[idx].thrusterLocation[i]);
                        thr->add_thrustvector(scIt.thrOutputMessage[idx].thrusterDirection[i]);
                    }
                }
            }

            // Write CSS output msg
            for (size_t idx = 0; idx < scIt.cssInMsgs.size(); idx++) {
                if (scIt.cssInMsgs[idx].isLinked() && scIt.cssConfLogInMsgStatus[idx].dataFresh) {
                    vizProtobufferMessage::VizMessage::CoarseSunSensor *css = scp->add_css();
                    for (int j = 0; j < 3; j++) {
                        css->add_normalvector(scIt.cssInMessage[idx].nHat_B[j]);
                        css->add_position(scIt.cssInMessage[idx].r_B[j]);
                    }
                    css->set_currentmsmt(scIt.cssInMessage[idx].signal);
                    css->set_maxmsmt(scIt.cssInMessage[idx].maxSignal);
                    css->set_minmsmt(scIt.cssInMessage[idx].minSignal);
                    css->set_cssgroupid(scIt.cssInMessage[idx].CSSGroupID);
                    css->set_fieldofview(scIt.cssInMessage[idx].fov * 2 * R2D); // must be edge to edge fov in degrees
                }
            }

            // Write generic sensor messages
            for (auto & idx : scIt.genericSensorList) {
                vizProtobufferMessage::VizMessage::GenericSensor *gs = scp->add_genericsensors();

                for (int j = 0; j < 3; j++) {
                    gs->add_position(idx->r_SB_B[j]);
                    gs->add_normalvector(idx->normalVector[j]);
                }
                for (double j : idx->fieldOfView) {
                    gs->add_fieldofview(j * R2D);
                }
                gs->set_ishidden(idx->isHidden);
                gs->set_size(idx->size);
                gs->set_label(idx->label);
                for (int j : idx->color) {
                    gs->add_color(j);
                }
                gs->set_activitystatus((int) idx->genericSensorCmd);
            }

            // Write Ellipsoid messages
            for (auto & idx : scIt.ellipsoidList) {
                vizProtobufferMessage::VizMessage::Ellipsoid *el = scp->add_ellipsoids();
                el->set_ison(idx->isOn);
                el->set_usebodyframe(idx->useBodyFrame);
                for (int j = 0; j < 3; j++) {
                    el->add_position(idx->position[j]);
                    el->add_semimajoraxes(idx->semiMajorAxes[j]);
                }
                for (int j : idx->color) {
                    el->add_color(j);
                }
                el->set_showgridlines(idx->showGridLines);
            }


            // Write transceiver messages
            for (auto & idx : scIt.transceiverList) {
                vizProtobufferMessage::VizMessage::Transceiver *tr = scp->add_transceivers();

                for (int j = 0; j < 3; j++) {
                    tr->add_position(idx->r_SB_B[j]);
                    tr->add_normalvector(idx->normalVector[j]);
                }
                tr->set_fieldofview(idx->fieldOfView * R2D);
                tr->set_ishidden(idx->isHidden);
                tr->set_label(idx->label);
                for (int j : idx->color) {
                    tr->add_color(j);
                }
                tr->set_transmitstatus(idx->transceiverState);
                tr->set_animationspeed(idx->animationSpeed);
            }

            // Write generic storage device messages
            for (auto & idx : scIt.genericStorageList) {
                vizProtobufferMessage::VizMessage::GenericStorage *gsd = scp->add_storagedevices();

                gsd->set_label(idx->label);
                gsd->set_currentvalue(idx->currentValue);
                gsd->set_maxvalue(idx->maxValue);
                gsd->set_units(idx->units);
                for (int j : idx->color) {
                    gsd->add_color(j);
                }
                for (int threshold : idx->thresholds) {
                    gsd->add_thresholds(threshold);
                }
            }

            // Write light device messages
            for (auto & idx : scIt.lightList) {
                vizProtobufferMessage::VizMessage::Light *ld = scp->add_lights();

                ld->set_label(idx->label);
                for (uint64_t j = 0; j < 3; j++) {
                    ld->add_position(idx->position[j]);
                    ld->add_normalvector(idx->normalVector[j]);
                }
                /* light on integer can only be 1, 0 or -1*/
                if (idx->lightOn > 1) {
                    idx->lightOn = 1;
                } else if (idx->lightOn < 0) {
                    idx->lightOn = -1;
                }
                ld->set_lighton(idx->lightOn);
                ld->set_fieldofview(idx->fieldOfView * R2D);
                ld->set_range(idx->range);
                ld->set_intensity(idx->intensity);
                ld->set_showlightmarker(idx->showLightMarker);
                ld->set_markerdiameter(idx->markerDiameter);
                for (int j : idx->color) {
                    ld->add_color(j);
                }
                ld->set_gammasaturation(idx->gammaSaturation);
                ld->set_showlensflare(idx->showLensFlare);
                ld->set_lensflarebrightness(idx->lensFlareBrightness);
                ld->set_lensflarefadespeed(idx->lensFlareFadeSpeed);
            }

            /* Write the SC sprite string */
            scp->set_modeldictionarykey(scIt.modelDictionaryKey);

            /* Write the sc logoTexture string */
            scp->set_logotexture(scIt.logoTexture);

            /* set spacecraft osculating orbit line color */
            for (int i : scIt.oscOrbitLineColor) {
                scp->add_oscorbitlinecolor(i);
            }

            /* set spacecraft true orbit line color */
            for (int i : scIt.trueTrajectoryLineColor) {
                scp->add_truetrajectorylinecolor(i);
            }

            // Write Multi-Sphere-Model messages
            for (auto & idx : scIt.msmInfo.msmList) {
                vizProtobufferMessage::VizMessage::MultiSphere *msmp = scp->add_multispheres();

                msmp->set_ison(idx->isOn);
                for (double j : idx->position) {
                    msmp->add_position(j);
                }
                msmp->set_radius(idx->radius);
                msmp->set_currentvalue(idx->currentValue);
                msmp->set_maxvalue(idx->maxValue);
                for (int j : idx->positiveColor) {
                    msmp->add_positivecolor(j);
                }
                for (int j : idx->negativeColor) {
                    msmp->add_negativecolor(j);
                }
                msmp->set_neutralopacity(idx->neutralOpacity);
            }
        }
    }

    /*! Write camera output msg */
    for (size_t camCounter = 0; camCounter < this->cameraConfInMsgs.size(); camCounter++) {
        if ((this->cameraConfInMsgs[camCounter].isLinked() && this->cameraConfMsgStatus[camCounter].dataFresh)
            || this->cameraConfigBuffers[camCounter].cameraID >= 0) {
            /*! This corrective rotation allows unity to place the camera as is expected by the python setting. Unity has a -x pointing camera, with z vertical on the sensor, and y horizontal which is not the OpNav frame: z point, x horizontal, y vertical (down) */
            double sigma_CuC[3];
            double unityCameraMRP[3]; /*! Cu is the unity Camera frame */
            v3Set(1. / 3, 1. / 3, -1. / 3, sigma_CuC);
            addMRP(this->cameraConfigBuffers[camCounter].sigma_CB, sigma_CuC, unityCameraMRP);
            vizProtobufferMessage::VizMessage::CameraConfig *camera = visPayload->add_cameras();
            for (int j = 0; j < 3; j++) {
                if (j < 2) {
                    camera->add_resolution(this->cameraConfigBuffers[camCounter].resolution[j]);
                }
                camera->add_cameradir_b(unityCameraMRP[j]);
                camera->add_camerapos_b(this->cameraConfigBuffers[camCounter].cameraPos_B[j]);
            }
            camera->set_renderrate(
                    this->cameraConfigBuffers[camCounter].renderRate); // Unity expects nanoseconds between images
            camera->set_cameraid(this->cameraConfigBuffers[camCounter].cameraID);
            camera->set_fieldofview(this->cameraConfigBuffers[camCounter].fieldOfView * R2D); // Unity expects degrees
            camera->set_skybox(this->cameraConfigBuffers[camCounter].skyBox);
            camera->set_parentname(this->cameraConfigBuffers[camCounter].parentName);
            camera->set_postprocessingon(this->cameraConfigBuffers[camCounter].postProcessingOn);
            camera->set_ppfocusdistance(this->cameraConfigBuffers[camCounter].ppFocusDistance);
            camera->set_ppaperture(this->cameraConfigBuffers[camCounter].ppAperture);
            camera->set_ppfocallength(this->cameraConfigBuffers[camCounter].ppFocalLength * 1000.0); // Unity expects mm
            camera->set_ppmaxblursize(this->cameraConfigBuffers[camCounter].ppMaxBlurSize);
            camera->set_updatecameraparameters(this->cameraConfigBuffers[camCounter].updateCameraParameters);
        }
    }

    /*! Write spice output msgs */
    for (size_t k = 0; k < this->gravBodyInformation.size(); k++) {
        if (this->spiceInMsgStatus[k].dataFresh) {
            vizProtobufferMessage::VizMessage::CelestialBody *spice = visPayload->add_celestialbodies();
            spice->set_bodyname(this->gravBodyInformation.at(k).bodyName);
            spice->set_mu(this->gravBodyInformation.at(k).mu / 1e9);  /* must be in km^3/s^2 */
            spice->set_radiuseq(this->gravBodyInformation.at(k).radEquator / 1000.);  /* must be in km */
            spice->set_radiusratio(this->gravBodyInformation.at(k).radiusRatio);
            spice->set_modeldictionarykey(this->gravBodyInformation.at(k).modelDictionaryKey);
            for (int i = 0; i < 3; i++) {
                spice->add_position(this->spiceMessage[k].PositionVector[i]);
                spice->add_velocity(this->spiceMessage[k].VelocityVector[i]);
                for (int j = 0; j < 3; j++) {
                    spice->add_rotation(this->spiceMessage[k].J20002Pfix[i][j]);
                }
            }
        }
    }

    google::protobuf::uint8 varIntBuffer[4];
    auto byteCount = (uint32_t) visPayload->ByteSizeLong();
    google::protobuf::uint8 const *end = google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(byteCount,
                                                                                                 varIntBuffer);
    auto varIntBytes = (unsigned long) (end - varIntBuffer);
    if (this->saveFile) {
        this->outputStream->write(reinterpret_cast<char * > (varIntBuffer), (int) varIntBytes);
    }

    /*! Enter in lock-step with the vizard to simulate a camera */
    /*!--OpNavMode set to ALL_FRAMES is to stay in lock-step with the viz at all time steps. It is a slower run,
     * but provides visual capabilities during OpNav */
    /*!--OpNavMode set to REQUESTED_FRAMES is a faster mode in which the viz only steps forward to the BSK time step
     * if an image is requested. This is a faster run but nothing can be visualized post-run */
    if (this->opNavMode == ClosedLoopMode::ALL_FRAMES ||
        (this->opNavMode == ClosedLoopMode::REQUESTED_FRAMES && this->shouldRequestACameraImage(CurrentSimNanos)) ||
        this->liveStream) {
        this->connector.send(*visPayload);

        for (size_t camCounter = 0; camCounter < this->cameraConfInMsgs.size(); camCounter++) {
            /*! - If the camera is requesting periodic images, request them */
            if (this->opNavMode != ClosedLoopMode::OPEN_LOOP &&
                CurrentSimNanos % this->cameraConfigBuffers[camCounter].renderRate == 0 &&
                this->cameraConfigBuffers[camCounter].isOn == 1) {
                this->requestImage(camCounter, CurrentSimNanos);
            }
        }
        if (this->shouldRequestACameraImage(CurrentSimNanos)) {
            this->connector.ping();
        }
    }
    /*!  Write protobuf to file */
    if (!this->saveFile || !visPayload->SerializeToOstream(this->outputStream)) {
        return;
    }

    delete visPayload;
    google::protobuf::ShutdownProtobufLibrary();
}

bool CielimInterface::shouldRequestACameraImage(uint64_t CurrentSimNanos) const{
    for (size_t camCounter = 0; camCounter < this->cameraConfInMsgs.size(); camCounter++) {
        if (CurrentSimNanos % this->cameraConfigBuffers[camCounter].renderRate == 0 &&
            this->cameraConfigBuffers[camCounter].isOn == 1 /*|| this->firstPass < 11*/) {
            return true;
        }
    }
    return false;
}

vizProtobufferMessage::VizMessage::LiveVizSettingsPb* CielimInterface::collectVizLiveSettings() {
    auto liveVizSettings = new vizProtobufferMessage::VizMessage::LiveVizSettingsPb;

    // define any pointing lines for Vizard
    for (auto & targetLine : this->liveSettings.targetLineList) {
        vizProtobufferMessage::VizMessage::PointLine *pl = liveVizSettings->add_targetlines();
        pl->set_tobodyname(targetLine.toBodyName);
        pl->set_frombodyname(targetLine.fromBodyName);

        for (int i : targetLine.lineColor) {
            pl->add_linecolor(i);
        }
    }

    liveVizSettings->set_relativeorbitchief(this->liveSettings.relativeOrbitChief);
    return liveVizSettings;
}

vizProtobufferMessage::VizMessage::VizSettingsPb*  CielimInterface::collectVizSettings() {
    vizProtobufferMessage::VizMessage::VizSettingsPb *vizSettings;
    vizSettings = new vizProtobufferMessage::VizMessage::VizSettingsPb;

    // define the viz ambient light setting
    vizSettings->set_ambient(this->settings.ambient);
    if (this->settings.ambient > 8.0 ||
        (this->settings.ambient < 0.0 && this->settings.ambient != -1.0)) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard ambient light value must be within [0,8]. "
                         "A value of %f was received.",
                         this->settings.ambient);
    }

    // define if osculating orbit lines should be shown
    vizSettings->set_orbitlineson(this->settings.orbitLinesOn);
    if (abs(this->settings.orbitLinesOn) > 2) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard orbitLinesOn flag must be either -1, 0, 1 or 2. "
                         "A value of %d was received.",
                         this->settings.orbitLinesOn);
    }

    // define if true orbit lines should be shown
    vizSettings->set_truetrajectorylineson(this->settings.trueTrajectoryLinesOn);
    if (abs(this->settings.trueTrajectoryLinesOn) > 2) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard trueTrajectoryLinesOn flag must be either -1, 0, 1 or 2. "
                         "A value of %d was received.",
                         this->settings.trueTrajectoryLinesOn);
    }

    // define if spacecraft axes should be shown
    vizSettings->set_spacecraftcson(this->settings.spacecraftCSon);
    if (abs(this->settings.spacecraftCSon) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard spacecraftCSon flag must be either -1, 0 or 1.  "
                         "A value of %d was received.",
                         this->settings.spacecraftCSon);
    }

    // define if planet axes should be shown
    vizSettings->set_planetcson(this->settings.planetCSon);
    if (abs(this->settings.planetCSon) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard planetCSon flag must be either -1, 0 or 1.  "
                         "A value of %d was received.",
                         this->settings.planetCSon);
    }

    // define the skyBox variable
    if (!this->settings.skyBox.empty()) {
        vizSettings->set_skybox(this->settings.skyBox);
    }

    // define any pointing lines for Vizard
    for (auto & idx : this->settings.pointLineList) {
        vizProtobufferMessage::VizMessage::PointLine *pl = vizSettings->add_pointlines();
        pl->set_tobodyname(idx.toBodyName);
        pl->set_frombodyname(idx.fromBodyName);
        for (int i : idx.lineColor) {
            pl->add_linecolor(i);
        }
    }

    // define any keep in/out cones for Vizard
    for (auto & idx : this->settings.coneList) {
        vizProtobufferMessage::VizMessage::KeepOutInCone *cone = vizSettings->add_keepoutincones();
        cone->set_iskeepin(idx.isKeepIn);
        for (int i = 0; i < 3; i++) {
            cone->add_position(idx.position_B[i]);
            cone->add_normalvector(idx.normalVector_B[i]);
        }
        cone->set_incidenceangle(idx.incidenceAngle * R2D);  // Unity expects degrees
        cone->set_coneheight(idx.coneHeight);
        cone->set_tobodyname(idx.toBodyName);
        cone->set_frombodyname(idx.fromBodyName);
        for (int i : idx.coneColor) {
            cone->add_conecolor(i);
        }
        cone->set_conename(idx.coneName);
    }

    // define if camera boresight line should be shown
    vizSettings->set_viewcameraboresighthud(this->settings.viewCameraBoresightHUD);
    if (abs(this->settings.viewCameraBoresightHUD) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard viewCameraBoresightHUD flag must be either -1, 0 or 1.  "
                         "A value of %d was received.",
                         this->settings.viewCameraBoresightHUD);
    }

    // define if camera cone should be shown
    vizSettings->set_viewcameraconehud(this->settings.viewCameraConeHUD);
    if (abs(this->settings.viewCameraConeHUD) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard viewCameraConeHUD flag must be either -1, 0 or 1.  "
                         "A value of %d was received.",
                         this->settings.viewCameraConeHUD);
    }

    // define if coordinate system labels should be shown
    vizSettings->set_showcslabels(this->settings.showCSLabels);
    if (abs(this->settings.showCSLabels) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard showCSLabels flag must be either -1, 0 or 1.  "
                         "A value of %d was received.",
                         this->settings.showCSLabels);
    }

    // define if celestial body labels should be shown
    vizSettings->set_showcelestialbodylabels(this->settings.showCelestialBodyLabels);
    if (abs(this->settings.showCelestialBodyLabels) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard showCelestialBodyLabels flag must be either -1, 0 or 1.  "
                         "A value of %d was received.",
                         this->settings.showCelestialBodyLabels);
    }

    // define if spacecraft labels should be shown
    vizSettings->set_showspacecraftlabels(this->settings.showSpacecraftLabels);
    if (abs(this->settings.showSpacecraftLabels) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard showSpacecraftLabels flag must be either -1, 0 or 1.  "
                         "A value of %d was received.",
                         this->settings.showSpacecraftLabels);
    }

    // define if camera labels should be shown
    vizSettings->set_showcameralabels(this->settings.showCameraLabels);
    if (abs(this->settings.showCameraLabels) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard showCameraLabels flag must be either -1, 0 or 1.  "
                         "A value of %d was received.",
                         this->settings.showCameraLabels);
    }

    // define the GUI scaling factor
    vizSettings->set_customguiscale(this->settings.customGUIScale);
    if (abs(this->settings.customGUIScale) > 3.0) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard customGUIScale flag must be either -1 or [0.5, 3]  "
                         "A value of %d was received.",
                         this->settings.customGUIScale);
    }

    // define default spacecraft sprite behavior
    vizSettings->set_defaultspacecraftsprite(this->settings.defaultSpacecraftSprite);

    // define if spacecraft should be shown as sprites
    vizSettings->set_showspacecraftassprites(this->settings.showSpacecraftAsSprites);
    if (abs(this->settings.showSpacecraftAsSprites) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard showSpacecraftAsSprites flag must be either -1, 0 or 1. "
                         "A value of %d was received.",
                         this->settings.showSpacecraftAsSprites);
    }

    // define if celestial objects should be shown as sprites
    vizSettings->set_showcelestialbodiesassprites(this->settings.showCelestialBodiesAsSprites);
    if (abs(this->settings.showCelestialBodiesAsSprites) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard showCelestialBodiesAsSprites flag must be either -1, 0 or 1. "
                         "A value of %d was received.",
                         this->settings.showCelestialBodiesAsSprites);
    }

    // define if the time should be shown using a 24h clock
    vizSettings->set_show24hrclock(this->settings.show24hrClock);
    if (abs(this->settings.show24hrClock) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard show24hrClock flag must be either -1, 0 or 1. "
                         "A value of %d was received.",
                         this->settings.show24hrClock);
    }

    // define if the data frame rate should be shown
    vizSettings->set_showdataratedisplay(this->settings.showDataRateDisplay);
    if (abs(this->settings.showDataRateDisplay) > 1) {
        bskLogger.bskLog(BSK_WARNING,
                         "vizInterface: The Vizard showDataRateDisplay flag must be either -1, 0 or 1. "
                         "A value of %d was received.",
                         this->settings.showDataRateDisplay);
    }

    // define the keyboard driven camera rates
    vizSettings->set_keyboardangularrate(this->settings.keyboardAngularRate * R2D);
    vizSettings->set_keyboardzoomrate(this->settings.keyboardZoomRate);

    // add default thrust plume color
    if (this->settings.defaultThrusterColor[0] >= 0) {
        for (int i : this->settings.defaultThrusterColor) {
            vizSettings->add_defaultthrustercolor(i);
        }
    }

    vizSettings->set_defaultthrusterplumelifescalar(this->settings.defaultThrusterPlumeLifeScalar);
    vizSettings->set_orbitlinesegments(this->settings.orbitLineSegments);
    vizSettings->set_relativeorbitrange(this->settings.relativeOrbitRange);
    vizSettings->set_showhillframe(this->settings.showHillFrame);
    vizSettings->set_showvelocityframe(this->settings.showVelocityFrame);
    vizSettings->set_relativeorbitframe(this->settings.relativeOrbitFrame);
    vizSettings->set_maincameratarget(this->settings.mainCameraTarget);
    vizSettings->set_spacecraftshadowbrightness(this->settings.spacecraftShadowBrightness);
    vizSettings->set_spacecraftsizemultiplier(this->settings.spacecraftSizeMultiplier);
    vizSettings->set_spacecrafthelioviewsizemultiplier(this->settings.spacecraftHelioViewSizeMultiplier);
    vizSettings->set_forcestartatspacecraftlocalview(this->settings.forceStartAtSpacecraftLocalView);
    vizSettings->set_showlocationcommlines(this->settings.showLocationCommLines);
    vizSettings->set_showlocationcones(this->settings.showLocationCones);
    vizSettings->set_showlocationlabels(this->settings.showLocationLabels);
    vizSettings->set_atmospheresoff(this->settings.atmospheresOff);
    vizSettings->set_scviewtoplanetviewboundarymultiplier(this->settings.scViewToPlanetViewBoundaryMultiplier);
    vizSettings->set_planetviewtohelioviewboundarymultiplier(
            this->settings.planetViewToHelioViewBoundaryMultiplier);
    vizSettings->set_sunintensity(this->settings.sunIntensity);
    vizSettings->set_attenuatesunlightwithdistance(this->settings.attenuateSunLightWithDistance);
    vizSettings->set_showlightlabels(this->settings.showLightLabels);
    vizSettings->set_celestialbodyhelioviewsizemultiplier(this->settings.celestialBodyHelioViewSizeMultiplier);
    vizSettings->set_showmissiontime(this->settings.showMissionTime);

    // define actuator GUI settings
    for (auto & idx : this->settings.actuatorGuiSettingsList) {
        vizProtobufferMessage::VizMessage::ActuatorSettings *al = vizSettings->add_actuatorsettings();
        al->set_spacecraftname(idx.spacecraftName);
        al->set_viewthrusterpanel(idx.viewThrusterPanel);
        al->set_viewthrusterhud(idx.viewThrusterHUD);
        al->set_viewrwpanel(idx.viewRWPanel);
        al->set_viewrwhud(idx.viewRWHUD);
        al->set_showthrusterlabels(idx.showThrusterLabels);
        al->set_showrwlabels(idx.showRWLabels);
    }

    // define instrument GUI settings
    for (auto & idx : this->settings.instrumentGuiSettingsList) {
        vizProtobufferMessage::VizMessage::InstrumentSettings *il = vizSettings->add_instrumentsettings();
        il->set_spacecraftname(idx.spacecraftName);
        il->set_viewcsspanel(idx.viewCSSPanel);
        il->set_viewcsscoverage(idx.viewCSSCoverage);
        il->set_viewcssboresight(idx.viewCSSBoresight);
        il->set_showcsslabels(idx.showCSSLabels);
        il->set_showgenericsensorlabels(idx.showGenericSensorLabels);
        il->set_showtransceiverlabels(idx.showTransceiverLabels);
        il->set_showtransceiverfrustrum(idx.showTransceiverFrustrum);
        il->set_showgenericstoragepanel(idx.showGenericStoragePanel);
        il->set_showmultispherelabels(idx.showMultiSphereLabels);
    }


    // define scene object custom object shapes
    for (auto & idx : this->settings.customModelList) {
        vizProtobufferMessage::VizMessage::CustomModel *cm = vizSettings->add_custommodels();
        CustomModel *cmp = &idx;
        cm->set_modelpath(cmp->modelPath);
        for (const auto & i : cmp->simBodiesToModify) {
            cm->add_simbodiestomodify(i);
        }
        for (size_t i = 0; i < 3; i++) {
            cm->add_offset(cmp->offset[i]);
            cm->add_rotation(cmp->rotation[i] * R2D);  // Unity expects degrees
            cm->add_scale(cmp->scale[i]);
        }
        cm->set_customtexturepath(cmp->customTexturePath);
        cm->set_normalmappath(cmp->normalMapPath);
        cm->set_shader(cmp->shader);
        for (int i : cmp->color) {
            cm->add_color(i);
        }
    }

    // define camera settings
    for (auto & idx : this->settings.stdCameraList) {
        vizProtobufferMessage::VizMessage::StandardCameraSettings *sc = vizSettings->add_standardcamerasettings();
        StdCameraSettings *scp = &idx;
        sc->set_spacecraftname(scp->spacecraftName);
        sc->set_setmode(scp->setMode);
        if (scp->fieldOfView < 0)
            sc->set_fieldofview(-1.0);
        else {
            sc->set_fieldofview(scp->fieldOfView * R2D); // Unity expects degrees
        }
        sc->set_bodytarget(scp->bodyTarget);
        sc->set_setview(scp->setView);
        for (double i : scp->pointingVector_B) {
            sc->add_pointingvector(i);
        }
        if (v3Norm(scp->position_B) > 0.00001) {
            for (double i : scp->position_B) {
                sc->add_position(i);
            }
        }
        sc->set_displayname(scp->displayName);
    }

    return vizSettings;
}

/*! Update this module at the task rate
 @param currentSimNanos The current sim time
 */
void CielimInterface::UpdateState(uint64_t currentSimNanos) {
    this->frameNumber += 1;
    this->readBskMessages();
    if (currentSimNanos > 0) {
        this->writeProtobuffer(currentSimNanos);
    }
}

/*! Method to add a Vizard instrument camera module to vizInterface
 @param tmpMsg Camera configuration msg
 */
void CielimInterface::addCamMsgToModule(Message<CameraConfigMsgPayload> *tmpMsg) {
    /* add the message reader to the vector of input messages */
    this->cameraConfInMsgs.push_back(tmpMsg->addSubscriber());

    /* expand vector of camera status and data buffer information */
    MsgCurrStatus tmpStatusMsg = {};
    this->cameraConfMsgStatus.push_back(tmpStatusMsg);

    CameraConfigMsgPayload tmpCamConfigMsg = {};
    tmpCamConfigMsg.cameraID = -1;
    strcpy(tmpCamConfigMsg.skyBox, "");
    tmpCamConfigMsg.renderRate = 0;
    this->cameraConfigBuffers.push_back(tmpCamConfigMsg);

    /* create output message */
    Message<CameraImageMsgPayload> *msg;
    msg = new Message<CameraImageMsgPayload>;
    this->opnavImageOutMsgs.push_back(msg);

    /* create image pointer */
    void *imgPtr = nullptr;
    this->bskImagePtrs.push_back(imgPtr);
}

void CielimInterface::requestImage(size_t camCounter, uint64_t currentSimNanos) {
    auto imageData = this->connector.requestImage(this->cameraConfigBuffers[camCounter].cameraID);
    this->bskImagePtrs[camCounter] = &imageData.imageBuffer;

    CameraImageMsgPayload imagePayload = {};
    imagePayload.timeTag = currentSimNanos;
    imagePayload.valid = 0;
    imagePayload.imagePointer = imageData.imageBuffer;
    imagePayload.imageBufferLength = imageData.imageBufferLength;
    imagePayload.cameraID = this->cameraConfigBuffers[camCounter].cameraID;
    imagePayload.imageType = 3;
    if (imageData.imageBufferLength > 0) { imagePayload.valid = 1; }
    this->opnavImageOutMsgs[camCounter]->write(&imagePayload, this->moduleID, currentSimNanos);
}

ClosedLoopMode CielimInterface::getOpNavMode() const {
    return this->opNavMode;
}

void CielimInterface::setOpNavMode(ClosedLoopMode mode) {
    this->opNavMode = mode;
}

int64_t CielimInterface::getFrameNumber() const {
    return this->frameNumber;
}

/*! A cleaning method to ensure the message buffers are wiped clean.
 @param data The current sim time in nanoseconds
 @param hint
 */
void message_buffer_deallocate(void *data, void *hint) {
    free(data);
}

void CielimInterface::readThrusterMessages(VizSpacecraftData &scIt) const {
    for (size_t idx = 0; idx < scIt.thrInMsgs.size(); idx++) {
        if (scIt.thrInMsgs[idx].isLinked()) {
            THROutputMsgPayload localThrusterArray = scIt.thrInMsgs.at(idx)();
            if (scIt.thrInMsgs.at(idx).isWritten() &&
                scIt.thrInMsgs.at(idx).timeWritten() != scIt.thrMsgStatus[idx].lastTimeTag) {
                scIt.thrMsgStatus[idx].lastTimeTag = scIt.thrInMsgs.at(idx).timeWritten();
                scIt.thrMsgStatus[idx].dataFresh = true;
                scIt.thrOutputMessage[idx] = localThrusterArray;
            }
        }
    }
}

void CielimInterface::readCSSMessages(VizSpacecraftData &scIt) const {
    for (size_t idx = 0; idx < scIt.cssInMsgs.size(); idx++) {
        if (scIt.cssInMsgs[idx].isLinked()) {
            CSSConfigLogMsgPayload localCSSMsg = scIt.cssInMsgs.at(idx)();
            if (scIt.cssInMsgs.at(idx).isWritten() &&
                scIt.cssInMsgs.at(idx).timeWritten() != scIt.cssConfLogInMsgStatus[idx].lastTimeTag) {
                scIt.cssConfLogInMsgStatus[idx].lastTimeTag = scIt.cssInMsgs.at(idx).timeWritten();
                scIt.cssConfLogInMsgStatus[idx].dataFresh = true;
                scIt.cssInMessage[idx] = localCSSMsg;
            }
        }
    }
}

void CielimInterface::readRWConstellationMessages(VizSpacecraftData &scIt) const{
    for (size_t idx = 0; idx < scIt.rwInMsgs.size(); idx++) {
        if (scIt.rwInMsgs[idx].isLinked()) {
            RWConfigLogMsgPayload localRWArray = scIt.rwInMsgs.at(idx)();
            if (scIt.rwInMsgs.at(idx).isWritten() &&
                scIt.rwInMsgs.at(idx).timeWritten() != scIt.rwInMsgStatus[idx].lastTimeTag) {
                scIt.rwInMsgStatus[idx].lastTimeTag = scIt.rwInMsgs.at(idx).timeWritten();
                scIt.rwInMsgStatus[idx].dataFresh = true;
                scIt.rwInMessage[idx] = localRWArray;
            }
        }
    }
}
