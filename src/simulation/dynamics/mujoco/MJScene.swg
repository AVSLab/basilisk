/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

%{
   #include "MJScene.h"
%}

%pythonbegin %{
from typing import List
%}

%include "std_vector.i"
%include "std_string.i"
%template() std::vector<std::string>;

%include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.i"
%include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"

%ignore MJScene::fromFile;

%ignore MJScene::SelfInit;
%ignore MJScene::Reset;
%ignore MJScene::initializeDynamics;
%ignore MJScene::UpdateState;
%ignore MJScene::equationsOfMotion;
%ignore MJScene::preIntegration;
%ignore MJScene::postIntegration;
%ignore MJScene::markAsNeedingToRecompileModel;
%ignore MJScene::areKinematicsStale;
%ignore MJScene::markKinematicsAsStale;
%ignore MJScene::writeFwdKinematicsMessages;
%ignore MJScene::getMujocoModel;
%ignore MJScene::getMujocoData;
%ignore MJScene::areMujocoModelConstStale;
%ignore MJScene::markMujocoModelConstAsStale;

%extend MJScene
{
%pythoncode %{
    @classmethod
    def fromFile(cls, filePath: str, files: List[str] = []):
        with open(filePath, "r") as f:
            return cls(f.read(), files)
%}
}

%include "MJScene.h"

%include "architecture/msgPayloadDefCpp/MJSceneStateMsgPayload.h"
