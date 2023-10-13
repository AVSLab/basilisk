/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

%module dataFileToViz
%{
   #include "dataFileToViz.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

%include "swig_conly_data.i"
%include "std_string.i"
%include "sys_model.i"
%include "std_vector.i"


%include "dataFileToViz.h"
%include "simulation/vizard/_GeneralModuleFiles/vizStructures.h"

%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;
%include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
struct RWConfigLogMsg_C;
%include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"


// Instantiate templates used by example
namespace std {
    %template(VizThrConfig) vector<ThrClusterMap>;
    %template(ThrClusterMapVectorVector) std::vector <std::vector <ThrClusterMap>>;
    %template(THROutputMsgOutMsgsVector) std::vector<Message<THROutputMsgPayload>>;
    %template(THROutputMsgOutMsgsPtrVector) std::vector<Message<THROutputMsgPayload>*>;
    %template(THROutputMsgInMsgsVector) std::vector<ReadFunctor<THROutputMsgPayload>>;
    %template(THROutputOutMsgsVectorVector) std::vector <std::vector <Message<THROutputMsgPayload>*>>;
    %template(RWConfigLogMsgOutMsgsVector) std::vector<Message<RWConfigLogMsgPayload>>;
    %template(RWConfigLogMsgOutMsgsPtrVector) std::vector<Message<RWConfigLogMsgPayload>*>;
    %template(RWConfigLogMsgInMsgsVector) std::vector<ReadFunctor<RWConfigLogMsgPayload>>;
    %template(RWConfigLogMsgInMsgsVectorVector) std::vector <std::vector <Message<RWConfigLogMsgPayload>*>>;

}

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
