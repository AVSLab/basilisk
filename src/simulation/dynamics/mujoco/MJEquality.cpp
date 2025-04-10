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

#include "MJEquality.h"
#include "MJSpec.h"

#include <algorithm>

void MJEquality::setActive(bool active)
{
    auto data = this->spec.getMujocoData();
    data->eq_active[this->getId()] = active;
}

void MJSingleJointEquality::setJointOffsetConstraint(double val)
{
    auto model = this->spec.getMujocoModel();
    for (size_t i = 0; i < mjNEQDATA; i++) {
        model->eq_data[this->getId() * mjNEQDATA + i] = i == 0 ? val : 0;
    }
}
