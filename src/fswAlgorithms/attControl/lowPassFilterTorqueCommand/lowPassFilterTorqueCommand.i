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
%module lowPassFilterTorqueCommand
%{
   #include "lowPassFilterTorqueCommand.h"
%}

%include "swig_conly_data.i"
%constant void Update_lowPassFilterTorqueCommand(void*, uint64_t, uint64_t);
%ignore Update_lowPassFilterTorqueCommand;
%constant void SelfInit_lowPassFilterTorqueCommand(void*, uint64_t);
%ignore SelfInit_lowPassFilterTorqueCommand;
%constant void CrossInit_lowPassFilterTorqueCommand(void*, uint64_t);
%ignore CrossInit_lowPassFilterTorqueCommand;
%constant void Reset_lowPassFilterTorqueCommand(void*, uint64_t, uint64_t);
%ignore Reset_lowPassFilterTorqueCommand;
GEN_SIZEOF(lowPassFilterTorqueCommandConfig);
%include "lowPassFilterTorqueCommand.h"

// sample Module supportfile to be included in this sub-module
%include "simFswInterfaceMessages/cmdTorqueBodyIntMsg.h"
GEN_SIZEOF(CmdTorqueBodyIntMsg);

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
