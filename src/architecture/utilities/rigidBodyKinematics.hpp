/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include <Eigen/Dense>

Eigen::Vector4d addEp(const Eigen::Vector4d& ep1, const Eigen::Vector4d& ep2);
Eigen::Vector3d addMrp(const Eigen::Vector3d& mrp1, const Eigen::Vector3d& mrp2);
Eigen::Vector3d addPrv(const Eigen::Vector3d& prv1, const Eigen::Vector3d& prv2);
Eigen::Vector3d addEulerAngles321(const Eigen::Vector3d& euler3211, const Eigen::Vector3d& euler3212);

Eigen::Matrix<double, 3, 4> binvEp(const Eigen::Vector4d& ep);
Eigen::Matrix3d binvMrp(const Eigen::Vector3d& mrp);
Eigen::Matrix3d binvPrv(const Eigen::Vector3d& prv);
Eigen::Matrix3d binvEulerAngles321(const Eigen::Vector3d& euler321);

Eigen::Matrix<double, 4, 3> bmatEp(const Eigen::Vector4d& ep);
Eigen::Matrix3d bmatMrp(const Eigen::Vector3d& mrp);
Eigen::Matrix3d bmatDotMrp(const Eigen::Vector3d& mrp, const Eigen::Vector3d& dmrp);
Eigen::Matrix3d bmatPrv(const Eigen::Vector3d& prv);
Eigen::Matrix3d bmatEulerAngles321(const Eigen::Vector3d& euler321);

Eigen::Vector4d dcmToEp(const Eigen::Matrix3d& dcm);
Eigen::Vector3d dcmToMrp(const Eigen::Matrix3d& dcm);
Eigen::Vector3d dcmToPrv(const Eigen::Matrix3d& dcm);
Eigen::Vector3d dcmToEulerAngles321(const Eigen::Matrix3d& dcm);

Eigen::Vector4d dep(const Eigen::Vector4d& ep, const Eigen::Vector3d& omega);
Eigen::Vector3d dmrp(const Eigen::Vector3d& mrp, const Eigen::Vector3d& omega);
Eigen::Vector3d dmrpToOmega(const Eigen::Vector3d& mrp, const Eigen::Vector3d& dmrp);
Eigen::Vector3d ddmrp(const Eigen::Vector3d& mrp,
                      const Eigen::Vector3d& dmrp,
                      const Eigen::Vector3d& omega,
                      const Eigen::Vector3d& domega);
Eigen::Vector3d ddmrpTodOmega(const Eigen::Vector3d& mrp, const Eigen::Vector3d& dmrp, const Eigen::Vector3d& ddmrp);
Eigen::Vector3d dprv(const Eigen::Vector3d& prv, const Eigen::Vector3d& omega);
Eigen::Vector3d deuler321(const Eigen::Vector3d& euler321, const Eigen::Vector3d& omega);

Eigen::Matrix3d epToDcm(const Eigen::Vector4d& ep);
Eigen::Vector3d epToMrp(const Eigen::Vector4d& ep);
Eigen::Vector3d epToPrv(const Eigen::Vector4d& ep);
Eigen::Vector3d epToEulerAngles321(const Eigen::Vector4d& ep);

Eigen::Matrix3d eulerAngles321ToDcm(const Eigen::Vector3d& euler321);
Eigen::Vector4d eulerAngles321ToEp(const Eigen::Vector3d& euler321);
Eigen::Vector3d eulerAngles321ToMrp(const Eigen::Vector3d& euler321);
Eigen::Vector3d eulerAngles321ToPrv(const Eigen::Vector3d& euler321);

Eigen::Matrix3d mrpToDcm(const Eigen::Vector3d& mrp);
Eigen::Vector4d mrpToEp(const Eigen::Vector3d& mrp);
Eigen::Vector3d mrpToPrv(const Eigen::Vector3d& mrp);
Eigen::Vector3d mrpToEulerAngles321(const Eigen::Vector3d& mrp);
Eigen::Vector3d mrpSwitch(const Eigen::Vector3d& mrp, const double s);
Eigen::Vector3d mrpShadow(const Eigen::Vector3d& mrp);

Eigen::Matrix3d prvToDcm(const Eigen::Vector3d& prv);
Eigen::Vector4d prvToEp(const Eigen::Vector3d& prv);
Eigen::Vector3d prvToMrp(const Eigen::Vector3d& prv);
Eigen::Vector3d prvToEulerAngles321(const Eigen::Vector3d& prv);

Eigen::Vector4d subEp(const Eigen::Vector4d& ep1, const Eigen::Vector4d& ep2);
Eigen::Vector3d subMrp(const Eigen::Vector3d& mrp1, const Eigen::Vector3d& mrp2);
Eigen::Vector3d subPrv(const Eigen::Vector3d& prv1, const Eigen::Vector3d& prv2);
Eigen::Vector3d subEulerAngles321(const Eigen::Vector3d& euler3211, const Eigen::Vector3d& euler3212);

Eigen::Matrix3d rotationMatrix(const double angle, const int axis_number);
Eigen::Matrix3d tildeMatrix(const Eigen::Vector3d& vector);
