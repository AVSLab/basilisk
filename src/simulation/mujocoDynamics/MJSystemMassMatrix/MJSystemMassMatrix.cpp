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


#include "simulation/mujocoDynamics/MJSystemMassMatrix/MJSystemMassMatrix.h"
#include "architecture/utilities/macroDefinitions.h"
#include <iostream>
#include <cstring>
#include <mujoco/mujoco.h>
#include <iomanip>

/*! Initialize C-wrapped output messages */
void
MJSystemMassMatrix::SelfInit(){
    MJSysMassMatrixMsg_C_init(&this->massMatrixOutMsgC);
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
MJSystemMassMatrix::MJSystemMassMatrix()
{
}

/*! This method is used to reset the module and checks that required input messages are connect.
*/
void MJSystemMassMatrix::Reset(uint64_t CurrentSimNanos)
{
    if (!scene) {
        bskLogger.bskLog(BSK_ERROR, "MJSystemMassMatrix: scene pointer not set!");
    }

    const auto model = scene->getMujocoModel();
    // extract the DOF dimensions
    this->nDOF = model -> nv;
    this->nbase = (model->nq >= 7 ? 6 : 0);
    this->nj = std::max(0, this->nDOF - this->nbase);

    if (this->nbase + this->nj > 6 + MAX_EFF_CNT) {
        bskLogger.bskLog(BSK_ERROR, "MJSystemMassMatrix: number of DOF (%d) exceeds message capacity (%d)!", this->nbase+this->nj, 6+MAX_EFF_CNT);
    }
}


/*! This extracts the total spacecraft mass matrix from the MuJoCo scene.
*/
void MJSystemMassMatrix::UpdateState(uint64_t CurrentSimNanos)
{
    const auto model = scene->getMujocoModel();
    const auto data = scene->getMujocoData();
    MJSysMassMatrixMsgPayload payload;

    // always zero the output message buffers before assigning values
    payload = this->massMatrixOutMsg.zeroMsgPayload;

    // Build dense M in mjtNum
    std::vector<mjtNum> Mdense(this->nDOF * this->nDOF, mjtNum(0));
    mj_fullM(model, Mdense.data(), data->qM);

    for (int i = 0; i < this->nbase + this->nj; ++i) {
        const mjtNum* src_row = Mdense.data() + i*this->nDOF;  // stride by nDOF
        double*       dst_row = &payload.MassMatrix[i][0];
        std::copy_n(src_row, this->nDOF, dst_row);
    }

    // write to the output messages
    payload.nbase = this->nbase;
    payload.nj = this->nj;
    this->massMatrixOutMsg.write(&payload, this->moduleID, CurrentSimNanos);
    // Write the C-wrapped output message
    MJSysMassMatrixMsg_C_write(&payload, &this->massMatrixOutMsgC, this->moduleID, CurrentSimNanos);

}
