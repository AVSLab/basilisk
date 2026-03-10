/*
 ISC License

Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

%module downlinkHandling

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "downlinkHandling.h"
%}

%include "swig_common_model.i"
%include "carrays.i"
%include "sys_model.i"

%include "simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h"
%include "downlinkHandling.h"

%include "architecture/msgPayloadDefC/DataNodeUsageMsgPayload.h"
struct DataNodeUsageMsg_C;
%include "architecture/msgPayloadDefC/DeviceCmdMsgPayload.h"
struct DeviceCmdMsg_C;
%include "architecture/msgPayloadDefC/LinkBudgetMsgPayload.h"
struct LinkBudgetMsg_C;
%include "architecture/msgPayloadDefC/DownlinkHandlingMsgPayload.h"
struct DownlinkHandlingMsg_C;
%include "architecture/msgPayloadDefCpp/DataStorageStatusMsgPayload.h"

%pythoncode %{
def _set_bit_rate_request(self, value):
    if not self.setBitRateRequest(value):
        raise ValueError("bitRateRequest must be finite and >= 0 [bit/s].")


def _set_packet_size_bits(self, value):
    if not self.setPacketSizeBits(value):
        raise ValueError("packetSizeBits must be finite and > 0 [bit].")


def _set_max_retransmissions(self, value):
    if not self.setMaxRetransmissions(value):
        raise ValueError("maxRetransmissions must be >= 1.")


def _set_receiver_antenna(self, value):
    if not self.setReceiverAntenna(value):
        raise ValueError("receiverAntenna must be 0 (auto), 1, or 2.")


def _set_removal_policy(self, value):
    if not self.setRemovalPolicy(value):
        raise ValueError("removalPolicy must be 0 (REMOVE_ATTEMPTED) or 1 (REMOVE_DELIVERED_ONLY).")


def _set_require_full_packet(self, value):
    self.setRequireFullPacket(bool(value))


DownlinkHandling.bitRateRequest = property(DownlinkHandling.getBitRateRequest, _set_bit_rate_request)
DownlinkHandling.packetSizeBits = property(DownlinkHandling.getPacketSizeBits, _set_packet_size_bits)
DownlinkHandling.maxRetransmissions = property(DownlinkHandling.getMaxRetransmissions, _set_max_retransmissions)
DownlinkHandling.receiverAntenna = property(DownlinkHandling.getReceiverAntenna, _set_receiver_antenna)
DownlinkHandling.removalPolicy = property(DownlinkHandling.getRemovalPolicy, _set_removal_policy)
DownlinkHandling.requireFullPacket = property(DownlinkHandling.getRequireFullPacket, _set_require_full_packet)
%}

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
