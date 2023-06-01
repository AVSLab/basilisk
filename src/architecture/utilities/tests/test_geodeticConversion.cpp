/*
 ISC License

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

#include "architecture/utilities/geodeticConversion.h"
#include "architecture/utilities/linearAlgebra.h"
#include <gtest/gtest.h>


TEST(GeodeticConversion, testPCI2PCPF) {
    Eigen::Vector3d pciPosition;
    pciPosition << 1., 2., 3.;

    double J20002Pfix[3][3];
    m33Set( 1, 0, 0,
            0, 0, 1,
            0,-1, 0,
           J20002Pfix);
    Eigen::Vector3d ans = PCI2PCPF(pciPosition, J20002Pfix);

    Eigen::Vector3d expected;
    expected << 1, 3, -2;
    EXPECT_TRUE(ans == expected);
}

TEST(GeodeticConversion, testPCPF2PCI) {
    Eigen::Vector3d pcpfPosition;
    pcpfPosition << 1., 2., 3.;

    double J20002Pfix[3][3];
    m33Set( 1, 0, 0,
            0, 0, 1,
            0,-1, 0,
           J20002Pfix);
    Eigen::Vector3d ans = PCPF2PCI(pcpfPosition, J20002Pfix);

    Eigen::Vector3d expected;
    expected << 1, -3, 2;
    EXPECT_TRUE(ans == expected);
}
