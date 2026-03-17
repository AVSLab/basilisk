/*
 ISC License

 Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder

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

#include "zeroWindModel.h"

ZeroWindModel::ZeroWindModel() = default;

void ZeroWindModel::evaluateWindModel(WindMsgPayload *msg, const Eigen::Vector3d& r_BP_N,
                                     const Eigen::Vector3d& v_corotatingAir_N, double /*currentTime*/)
{
    // For ZeroWindModel, v_air_N is just the co-rotating atmosphere velocity
    Eigen::Map<Eigen::Vector3d>(msg->v_air_N) = v_corotatingAir_N;
    // Zero out the wind perturbation since this model represents no additional wind
    Eigen::Map<Eigen::Vector3d>(msg->v_wind_N).setZero();
}
