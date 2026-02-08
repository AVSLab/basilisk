/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "simulation/mujocoDynamics/MJSystemCoM/MJSystemCoM.h"
#include <iostream>
#include <cstring>
#include <mujoco/mujoco.h>

/*! Initialize C-wrapped output messages */
void
MJSystemCoM::SelfInit(){
    SCStatesMsg_C_init(&this->comStatesOutMsgC);
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
MJSystemCoM::MJSystemCoM()
{
}

/*! This method is used to reset the module and checks that required input messages are connect.
*/
void MJSystemCoM::Reset(uint64_t CurrentSimNanos)
{
    if (!scene) {
        bskLogger.bskLog(BSK_ERROR, "MJSystemCoM: scene pointer not set!");
    }
}


/*! This extracts the total spacecraft CoM position and velocity
*/
void MJSystemCoM::UpdateState(uint64_t CurrentSimNanos)
{
    const auto model = scene->getMujocoModel();
    const auto data = scene->getMujocoData();
    double massSC = 0.0;
    Eigen::Vector3d r_CN_N;
    Eigen::Vector3d v_CN_N;
    SCStatesMsgPayload payload;  //!< local copy of message buffer

    // always zero the output message buffers before assigning values
    payload = this->comStatesOutMsg.zeroMsgPayload;
    r_CN_N.setZero();
    v_CN_N.setZero();

    // calculate CoM position
    for (int i = 1; i < model->nbody; i++) {
        const double mi = model->body_mass[i];
        if (mi <= 0.0) continue;

        // Body COM position in world: xipos[3*i : 3*i+3]
        const Eigen::Vector3d ri = Eigen::Map<const Eigen::Vector3d>(data->xipos + 3*i);

        massSC += mi;
        r_CN_N += mi * ri;
    }

    // calculate CoM velocity
    if (model->nv > 0 && massSC > 0.0) {
        // jacp = linear 3×nv, jacr = angular 3×nv
        std::vector<double> jacp(3*model->nv, 0.0);
        std::vector<double> jacr(3*model->nv, 0.0);

        Eigen::Map<const Eigen::VectorXd> qv(data->qvel, model->nv);

        for (int i = 1; i < model->nbody; ++i) {
            const double mi = model->body_mass[i];
            if (mi <= 0.0) continue;

            // Fill jacobians at the BODY COM
            // (MuJoCo API: mj_jacBodyCom(m, d, jacp, jacr, body_id))
            std::fill(jacp.begin(), jacp.end(), 0.0);
            std::fill(jacr.begin(), jacr.end(), 0.0);
            mj_jacBodyCom(model, data, jacp.data(), jacr.data(), i);

            // vi = jacp * qvel   (jacp is row-major 3×nv)
            Eigen::Map<const Eigen::Matrix<double,3,Eigen::Dynamic,Eigen::RowMajor>>
                Jp(jacp.data(), 3, model->nv);

            const Eigen::Vector3d vi = Jp * qv;   // linear COM velocity of body i

            v_CN_N += mi * vi;
        }
    }

    if (massSC > 0) {
        r_CN_N /= massSC;
        v_CN_N /= massSC;
    }

    // write to the output messages
    std::copy_n(r_CN_N.data(), 3, payload.r_CN_N);
    std::copy_n(v_CN_N.data(), 3, payload.v_CN_N);
    this->comStatesOutMsg.write(&payload, this->moduleID, CurrentSimNanos);
    // Write the C-wrapped output message
    SCStatesMsg_C_write(&payload, &this->comStatesOutMsgC, this->moduleID, CurrentSimNanos);
}
