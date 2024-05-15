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

Eigen::Vector4d addQuaternion(const Eigen::Vector4d& ep1, const Eigen::Vector4d& ep2);
Eigen::Vector3d addMrp(const Eigen::Vector3d& mrp1, const Eigen::Vector3d& mrp2);
Eigen::Vector3d addPrv(const Eigen::Vector3d& prv1, const Eigen::Vector3d& prv2);
Eigen::Vector3d addEulerAngles321(const Eigen::Vector3d& euler3211, const Eigen::Vector3d& euler3212);

Eigen::Matrix<double, 3, 4> bInvQuaternion(const Eigen::Vector4d& ep);
Eigen::Matrix3d bInvMrp(const Eigen::Vector3d& mrp);
Eigen::Matrix3d bInvPrv(const Eigen::Vector3d& prv);
Eigen::Matrix3d bInvEulerAngles321(const Eigen::Vector3d& euler321);

Eigen::Matrix<double, 4, 3> bQuaternion(const Eigen::Vector4d& ep);
Eigen::Matrix3d bMrp(const Eigen::Vector3d& mrp);
Eigen::Matrix3d bDotMrp(const Eigen::Vector3d& mrp, const Eigen::Vector3d& dmrp);
Eigen::Matrix3d bPrv(const Eigen::Vector3d& prv);
Eigen::Matrix3d bEulerAngles321(const Eigen::Vector3d& euler321);

Eigen::Vector4d dcmToQuaternion(const Eigen::Matrix3d& dcm);
Eigen::Vector3d dcmToMrp(const Eigen::Matrix3d& dcm);
Eigen::Vector3d dcmToPrv(const Eigen::Matrix3d& dcm);
Eigen::Vector3d dcmToEulerAngles321(const Eigen::Matrix3d& dcm);

Eigen::Vector4d quaternionDot(const Eigen::Vector4d& ep, const Eigen::Vector3d& omega);
Eigen::Vector3d mrpDot(const Eigen::Vector3d& mrp, const Eigen::Vector3d& omega);
Eigen::Vector3d mrpDotToOmega(const Eigen::Vector3d& mrp, const Eigen::Vector3d& dmrp);
Eigen::Vector3d mrpDotDot(const Eigen::Vector3d& mrp,
                      const Eigen::Vector3d& dmrp,
                      const Eigen::Vector3d& omega,
                      const Eigen::Vector3d& domega);
Eigen::Vector3d mrpDotDotToOmegaDot(const Eigen::Vector3d& mrp, const Eigen::Vector3d& dmrp, const Eigen::Vector3d& ddmrp);
Eigen::Vector3d prvDot(const Eigen::Vector3d& prv, const Eigen::Vector3d& omega);
Eigen::Vector3d euler321Dot(const Eigen::Vector3d& euler321, const Eigen::Vector3d& omega);

Eigen::Matrix3d quaternionToDcm(const Eigen::Vector4d& ep);
Eigen::Vector3d quaternionToMrp(const Eigen::Vector4d& ep);
Eigen::Vector3d quaternionToPrv(const Eigen::Vector4d& ep);
Eigen::Vector3d quaternionToEulerAngles321(const Eigen::Vector4d& ep);

Eigen::Matrix3d eulerAngles321ToDcm(const Eigen::Vector3d& euler321);
Eigen::Vector4d eulerAngles321ToQuaternion(const Eigen::Vector3d& euler321);
Eigen::Vector3d eulerAngles321ToMrp(const Eigen::Vector3d& euler321);
Eigen::Vector3d eulerAngles321ToPrv(const Eigen::Vector3d& euler321);

Eigen::Matrix3d mrpToDcm(const Eigen::Vector3d& mrp);
Eigen::Vector4d mrpToQuaternion(const Eigen::Vector3d& mrp);
Eigen::Vector3d mrpToPrv(const Eigen::Vector3d& mrp);
Eigen::Vector3d mrpToEulerAngles321(const Eigen::Vector3d& mrp);
Eigen::Vector3d mrpInnerSet(const Eigen::Vector3d& mrp, const double s);
Eigen::Vector3d mrpShadowSet(const Eigen::Vector3d& mrp);

Eigen::Matrix3d prvToDcm(const Eigen::Vector3d& prv);
Eigen::Vector4d prvToQuaternion(const Eigen::Vector3d& prv);
Eigen::Vector3d prvToMrp(const Eigen::Vector3d& prv);
Eigen::Vector3d prvToEulerAngles321(const Eigen::Vector3d& prv);

Eigen::Vector4d subQuaternion(const Eigen::Vector4d& ep1, const Eigen::Vector4d& ep2);
Eigen::Vector3d subMrp(const Eigen::Vector3d& mrp1, const Eigen::Vector3d& mrp2);
Eigen::Vector3d subPrv(const Eigen::Vector3d& prv1, const Eigen::Vector3d& prv2);
Eigen::Vector3d subEulerAngles321(const Eigen::Vector3d& euler3211, const Eigen::Vector3d& euler3212);

Eigen::Matrix3d rotationMatrix(const double angle, const int axis_number);
Eigen::Matrix3d tildeMatrix(const Eigen::Vector3d& vector);
