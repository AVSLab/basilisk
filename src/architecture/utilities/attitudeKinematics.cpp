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

#include "attitudeKinematics.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

constexpr double eps = std::numeric_limits<double>::epsilon();

/**
 * Provide the Euler parameter vector which corresponds to performing to successive rotations B1 and B2
 * @param ep1 Eigen::Vector4d
 * @param ep2 Eigen::Vector4d
 * @return Eigen::Vector4d
 */
Eigen::Vector4d addEp(const Eigen::Vector4d& ep1, const Eigen::Vector4d& ep2){
    Eigen::Vector4d ep;

    ep(0) = ep2(0) * ep1(0) - ep2(1) * ep1(1) - ep2(2) * ep1(2) - ep2(3) * ep1(3);
    ep(1) = ep2(1) * ep1(0) + ep2(0) * ep1(1) + ep2(3) * ep1(2) - ep2(2) * ep1(3);
    ep(2) = ep2(2) * ep1(0) - ep2(3) * ep1(1) + ep2(0) * ep1(2) + ep2(1) * ep1(3);
    ep(3) = ep2(3) * ep1(0) + ep2(2) * ep1(1) - ep2(1) * ep1(2) + ep2(0) * ep1(3);

    return ep;
}

/**
 * Provide the mrp parameter vector which corresponds to performing to successive rotations q1 and q2
 * @param mrp1 Eigen::Vector3d
 * @param mrp2 Eigen::Vector3d
 * @return Eigen::Vector3d
 */
Eigen::Vector3d addMrp(const Eigen::Vector3d& mrp1, const Eigen::Vector3d& mrp2){
    Eigen::Vector3d sigma1(mrp1);
    double denominator;
    denominator = 1 + mrp1.dot(mrp1)*mrp2.dot(mrp2) - 2 * mrp1.dot(mrp2);
    if (std::abs(denominator) < 0.1) {
        sigma1 = mrpShadow(mrp1); // Shadow set
        denominator = (1 + sigma1.dot(sigma1) * mrp2.dot(mrp2) - 2 * sigma1.dot(mrp2));
    }

    assert(std::abs(denominator) > eps);
    Eigen::Vector3d numerator;
    numerator << (1 - sigma1.dot(sigma1))*mrp2 + (1 - mrp2.dot(mrp2))*sigma1 - 2*mrp2.cross(sigma1);

    Eigen::Vector3d mrp(numerator/denominator);
    /* map mrp to inner set */
    mrp = mrpSwitch(mrp, 1);

    return mrp;
}

/**
 * Provide the principal rotation vector which corresponds to performing successive principal rotations Q1 and Q2.
 * @param prv1 Eigen::Vector3d
 * @param prv2 Eigen::Vector3d
 * @return Eigen::Vector3d
 */
Eigen::Vector3d addPrv(const Eigen::Vector3d& prv1, const Eigen::Vector3d& prv2){

    if(prv1.norm() < 1.0E-7 || prv2.norm() < 1.0E-7){return prv1 + prv2;}

    double cosPhi1 = std::cos(prv1.norm() / 2.);
    double cosPhi2 = std::cos(prv2.norm() / 2.);
    double sinPhi1 = std::sin(prv1.norm() / 2.);
    double sinPhi2 = std::sin(prv2.norm() / 2.);

    Eigen::Vector3d unitVector1(prv1/prv1.norm());
    Eigen::Vector3d unitVector2(prv2/prv2.norm());
    assert(std::abs(cosPhi1 * cosPhi2 - sinPhi1 * sinPhi2 * unitVector1.dot(unitVector2)) < 1);

    double angle;
    angle = 2 * acos(cosPhi1 * cosPhi2 - sinPhi1 * sinPhi2 * unitVector1.dot(unitVector2));

    Eigen::Vector3d prv;
    if(std::abs(angle) < 1.0E-13){return prv.setZero();}
    prv << cosPhi1*sinPhi2*unitVector2 + cosPhi2*sinPhi1*unitVector1 + sinPhi1*sinPhi2*unitVector1.cross(unitVector2);
    return prv*angle/std::sin(angle/2);
}

/**
 * Compute the overall 321 Euler angle vector corresponding to two successive 321) rotations.
 * @param euler3211 Eigen::Vector3d
 * @param euler3212 Eigen::Vector3d
 * @return Eigen::Vector3d
 */
Eigen::Vector3d addEulerAngles321(const Eigen::Vector3d& euler3211, const Eigen::Vector3d& euler3212){
    return dcmToEulerAngles321(eulerAngles321ToDcm(euler3212)*eulerAngles321ToDcm(euler3211));
}

/**
 * Return the matrix which relates the derivative of Euler parameter vector to the body angular
 * velocity vector omega = 2 [B(Q)]^(-1) dQ/dt
 * @param ep const Eigen::Vector4
 * @return Eigen::Matrix<double, 3, 4>
 */
Eigen::Matrix<double, 3, 4> binvEp(const Eigen::Vector4d& ep){
    Eigen::Matrix<double, 3, 4> Binv;

    Binv << -ep(1), ep(0), ep(3), -ep(2),
    -ep(2), -ep(3), ep(0), ep(1),
    -ep(3), ep(2), -ep(1), ep(0);

    return Binv;
}

/**
 * Return the matrix which relates the derivative of mrp vector Q to the body angular velocity vector
 * omega = 4 [B(Q)]^(-1) dQ/dt
 * @param mrp Eigen::Vector3d
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d binvMrp(const Eigen::Vector3d& mrp){

    Eigen::Matrix3d Binv;

    Binv(0, 0) = 1 - mrp.dot(mrp) + 2 * mrp(0) * mrp(0);
    Binv(0, 1) = 2 * (mrp(0) * mrp(1) + mrp(2));
    Binv(0, 2) = 2 * (mrp(0) * mrp(2) - mrp(1));
    Binv(1, 0) = 2 * (mrp(1) * mrp(0) - mrp(2));
    Binv(1, 1) = 1 - mrp.dot(mrp) + 2 * mrp(1) * mrp(1);
    Binv(1, 2) = 2 * (mrp(1) * mrp(2) + mrp(0));
    Binv(2, 0) = 2 * (mrp(2) * mrp(0) + mrp(1));
    Binv(2, 1) = 2 * (mrp(2) * mrp(1) - mrp(0));
    Binv(2, 2) = 1 - mrp.dot(mrp) + 2 * mrp(2) * mrp(2);

    return Binv/ (1 + mrp.dot(mrp)) / (1 + mrp.dot(mrp));
}

/**
 * Return the matrix which relates the derivative of principal rotation vector to the body angular velocity vector
 * omega = [B(Q)]^(-1) dQ/dt
 * @param prv Eigen::Vector3
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d binvPrv(const Eigen::Vector3d& prv){
    double c1 = (1 - std::cos(prv.norm())) / std::pow(prv.norm(), 2);
    double c2 = (prv.norm() - std::sin(prv.norm())) / std::pow(prv.norm(), 3);

    Eigen::Matrix3d Binv;
    Binv(0, 0) = 1 - c2 * (prv(1) * prv(1) + prv(2) * prv(2));
    Binv(0, 1) = c1 * prv(2) + c2 * prv(0) * prv(1);
    Binv(0, 2) = -c1 * prv(1) + c2 * prv(0) * prv(2);
    Binv(1, 0) = -c1 * prv(2) + c2 * prv(0) * prv(1);
    Binv(1, 1) = 1 - c2 * (prv(0) * prv(0) + prv(2) * prv(2));
    Binv(1, 2) = c1 * prv(0) + c2 * prv(1) * prv(2);
    Binv(2, 0) = c1 * prv(1) + c2 * prv(2) * prv(0);
    Binv(2, 1) = -c1 * prv(0) + c2 * prv(2) * prv(1);
    Binv(2, 2) = 1 - c2 * (prv(0) * prv(0) + prv(1) * prv(1));

    return Binv;
}

/**
 * Return the matrix which relates the derivative of a 321 Euler angle vector to the body angular velocity vector.
 * omega = [B(euler321)]^(-1) dQ/dt
 * @param euler321 Eigen::Vector3d
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d binvEulerAngles321(const Eigen::Vector3d& euler321){
    double sin2 = std::sin(euler321(1));
    double cos2 = std::cos(euler321(1));
    double sin3 = std::sin(euler321(2));
    double cos3 = std::cos(euler321(2));

    Eigen::Matrix3d Binv;
    Binv << -sin2, 0, 1,
    cos2*sin3, cos3, 0,
    cos2*cos3, - sin3, 0;

    return Binv;
}

/**
 * Return the matrix which relates the body angular velocity vector to the derivative of Euler parameter vector
 * dQ/dt = 1/2 [B(Q)] omega
 * @param ep Eigen::Vector4d
 * @return Eigen::Matrix<double, 4, 3>
 */
Eigen::Matrix<double, 4, 3> bmatEp(const Eigen::Vector4d& ep){
    Eigen::Matrix<double, 4, 3> B;

    B(0, 0) = -ep(1);
    B(0, 1) = -ep(2);
    B(0, 2) = -ep(3);
    B(1, 0) = ep(0);
    B(1, 1) = -ep(3);
    B(1, 2) = ep(2);
    B(2, 0) = ep(3);
    B(2, 1) = ep(0);
    B(2, 2) = -ep(1);
    B(3, 0) = -ep(2);
    B(3, 1) = ep(1);
    B(3, 2) = ep(0);

    return B;
}

/**
 * Return the matrix which relates the body angular velocity vector w to the derivative of mrp vector
 * dQ/dt = 1/4 [B(Q)] omega
 * @param mrp
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d bmatMrp(const Eigen::Vector3d& mrp){
    Eigen::Matrix3d B;

    B(0, 0) = 1 - mrp.dot(mrp) + 2 * mrp(0) * mrp(0);
    B(0, 1) = 2 * (mrp(0) * mrp(1) - mrp(2));
    B(0, 2) = 2 * (mrp(0) * mrp(2) + mrp(1));
    B(1, 0) = 2 * (mrp(1) * mrp(0) + mrp(2));
    B(1, 1) = 1 - mrp.dot(mrp) + 2 * mrp(1) * mrp(1);
    B(1, 2) = 2 * (mrp(1) * mrp(2) - mrp(0));
    B(2, 0) = 2 * (mrp(2) * mrp(0) - mrp(1));
    B(2, 1) = 2 * (mrp(2) * mrp(1) + mrp(0));
    B(2, 2) = 1 - mrp.dot(mrp) + 2 * mrp(2) * mrp(2);

    return B;
}

/**
 * Return the matrix derivative of the bmatMrp matrix, and it is used to relate the  body angular acceleration
 * vector dw to the second order derivative of the mrp vector
 * (d^2Q)/(dt^2) = 1/4 ( [B(Q)] dw + [Bdot(Q,dQ)] w )
 * @param mrp
 * @param dmrp
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d bmatDotMrp(const Eigen::Vector3d& mrp, const Eigen::Vector3d& dmrp){
    Eigen::Matrix3d B;

    B(0, 0) = -2*mrp.dot(dmrp) + 4 * ( mrp(0) * dmrp(0));
    B(0, 1) = 2 * (-dmrp(2) + mrp(0) * dmrp(1) + dmrp(0) * mrp(1));
    B(0, 2) = 2 * ( dmrp(1) + mrp(0) * dmrp(2) + dmrp(0) * mrp(2));
    B(1, 0) = 2 * ( dmrp(2) + mrp(0) * dmrp(1) + dmrp(0) * mrp(1));
    B(1, 1) = -2*mrp.dot(dmrp) + 4 * ( mrp(1) * dmrp(1));
    B(1, 2) = 2 * (-dmrp(0) + mrp(1) * dmrp(2) + dmrp(1) * mrp(2));
    B(2, 0) = 2 * (-dmrp(1) + mrp(0) * dmrp(2) + dmrp(0) * mrp(2));
    B(2, 1) = 2 * ( dmrp(0) + mrp(1) * dmrp(2) + dmrp(1) * mrp(2));
    B(2, 2) = -2*mrp.dot(dmrp) + 4 * ( mrp(2) * dmrp(2));

    return B;
}

/**
 * Return the matrix which relates the  body angular velocity vector to the derivative of principal
 * rotation vector dQ/dt = [B(Q)] omega
 * @param prv
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d bmatPrv(const Eigen::Vector3d& prv){
    double c = 1. / prv.norm() / prv.norm() * (1. - prv.norm() / 2. / std::tan(prv.norm() / 2.));

    Eigen::Matrix3d B;
    B(0, 0) = 1 - c * (prv(1) * prv(1) + prv(2) * prv(2));
    B(0, 1) = -prv(2) / 2 + c * (prv(0) * prv(1));
    B(0, 2) = prv(1) / 2 + c * (prv(0) * prv(2));
    B(1, 0) = prv(2) / 2 + c * (prv(0) * prv(1));
    B(1, 1) = 1 - c * (prv(0) * prv(0) + prv(2) * prv(2));
    B(1, 2) = -prv(0) / 2 + c * (prv(1) * prv(2));
    B(2, 0) = -prv(1) / 2 + c * (prv(0) * prv(2));
    B(2, 1) = prv(0) / 2 + c * (prv(1) * prv(2));
    B(2, 2) = 1 - c * (prv(0) * prv(0) + prv(1) * prv(1));

    return B;
}

/**
 * Return the matrix which relates the body angular velocity vector to the derivative of 321 Euler angle vector.
 * dQ/dt = [B(Q)] omega
 * @param euler321
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d bmatEulerAngles321(const Eigen::Vector3d& euler321){
    double sin2 = sin(euler321(1));
    double cos2 = cos(euler321(1));
    double sin3 = sin(euler321(2));
    double cos3 = cos(euler321(2));

    Eigen::Matrix3d B;
    B << 0, sin3, cos3,
    0, cos2*cos3, -cos2*sin3,
    cos2, sin2*sin3, sin2*cos3;

    return B/cos2;
}

/**
 * Translate the direction cosine matrix into the corresponding Euler parameter vector,
 * where the first component of is the non-dimensional Euler parameter >= 0. Transformation is done
 * using the Stanley method.
 * @param dcm
 * @return Eigen::Vector4d
 */
Eigen::Vector4d dcmToEp(const Eigen::Matrix3d& dcm){
    Eigen::Vector4d ep;

    ep(0) = (1 + dcm.trace()) / 4.;
    ep(1) = (1 + 2 * dcm(0, 0) - dcm.trace()) / 4.;
    ep(2) = (1 + 2 * dcm(1, 1) - dcm.trace()) / 4.;
    ep(3) = (1 + 2 * dcm(2, 2) - dcm.trace()) / 4.;

    std::vector<double> epVector(ep.data(), ep.data() + ep.size());
    double maxVal = *std::max_element(epVector.begin(), epVector.end());

    if (maxVal == ep(0)){
        ep(0) = sqrtSafe(ep(0));
        ep(1) = (dcm(1, 2) - dcm(2, 1)) / 4 / ep(0);
        ep(2) = (dcm(2, 0) - dcm(0, 2)) / 4 / ep(0);
        ep(3) = (dcm(0, 1) - dcm(1, 0)) / 4 / ep(0);
        }
    else if (maxVal == ep(1)){
        ep(1) = sqrtSafe(ep(1));
        ep(0) = (dcm(1, 2) - dcm(2, 1)) / 4 / ep(1);
        if(ep(0) < 0) {
            ep(1) = -ep(1);
            ep(0) = -ep(0);
        }
        ep(2) = (dcm(0, 1) + dcm(1, 0)) / 4 / ep(1);
        ep(3) = (dcm(2, 0) + dcm(0, 2)) / 4 / ep(1);
        }
    else if (maxVal == ep(2)){
        ep(2) = sqrtSafe(ep(2));
        ep(0) = (dcm(2, 0) - dcm(0, 2)) / 4 / ep(2);
        if(ep(0) < 0) {
            ep(2) = -ep(2);
            ep(0) = -ep(0);
        }
        ep(1) = (dcm(0, 1) + dcm(1, 0)) / 4 / ep(2);
        ep(3) = (dcm(1, 2) + dcm(2, 1)) / 4 / ep(2);
        }
    else if (maxVal == ep(3)){
        ep(3) = sqrtSafe(ep(3));
        ep(0) = (dcm(0, 1) - dcm(1, 0)) / 4 / ep(3);
        if(ep(0) < 0) {
            ep(3) = -ep(3);
            ep(0) = -ep(0);
        }
        ep(1) = (dcm(2, 0) + dcm(0, 2)) / 4 / ep(3);
        ep(2) = (dcm(1, 2) + dcm(2, 1)) / 4 / ep(3);
        }

    return ep;
}

/**
 * Translate a direction cosine matrix into the corresponding mrp vector where the mrp vector is chosen such
 * that its norm is less than 1
 * @param dcm
 * @return Eigen::Vector3d
 */
Eigen::Vector3d dcmToMrp(const Eigen::Matrix3d& dcm){
    Eigen::Vector4d ep(dcmToEp(dcm));
    return ep.tail(3)/(1 + ep(0));
}

/**
 * Translate a direction cosine matrix into the corresponding principal rotation vector,
 * where the first component is the principal rotation angle 0<= phi <= Pi
 * @param dcm
 * @return Eigen::Vector3d
 */
Eigen::Vector3d dcmToPrv(const Eigen::Matrix3d& dcm){
    return epToPrv(dcmToEp(dcm));
}

/**
 * Translate a direction cosine matrix into the corresponding 321 Euler angle set.
 * @param dcm
 * @return Eigen::Vector3d
 */
Eigen::Vector3d dcmToEulerAngles321(const Eigen::Matrix3d& dcm){
    Eigen::Vector3d euler321;

    euler321[0] = std::atan2(dcm(0,1), dcm(0,0));
    euler321[1] = aSinSafe(-dcm(0,2));
    euler321[2] = std::atan2(dcm(1,2), dcm(2,2));

    return euler321;
}

/**
 * Return the Euler parameter derivative for a given Euler parameter vector and body
 * angular velocity vector omega, dQ/dt = 1/2 [B(Q)] omega
 * @param ep
 * @param omega
 * @return Eigen::Vector4d
 */
Eigen::Vector4d dep(const Eigen::Vector4d& ep, const Eigen::Vector3d& omega){
    Eigen::Matrix<double, 4, 3> B(bmatEp(ep));
    Eigen::Vector4d dep(B*omega);
    for(int i = 0; i < 4; i++) {
        dep(i) = 0.;
        for(int j = 0; j < 3; j++) {
            dep(i) += B(i, j) * omega(j);
        }
    }
    return dep/2;
}

/**
 * Return the mrp derivative for a given mrp vector and body angular velocity vector.
 * dQ/dt = 1/4 [B(Q)] omega
 * @param mrp
 * @param omega
 * @return Eigen::Vector3d
 */
Eigen::Vector3d dmrp(const Eigen::Vector3d& mrp, const Eigen::Vector3d& omega){
    Eigen::Matrix3d B(bmatMrp(mrp));
    return B*omega/4;
}

/**
 * Return the angular rate for a given mrp vector and mrp derivative.
 * omega = 4 [B(Q)]^(-1) dQ/dt
 * @param mrp
 * @param dmrp
 * @return Eigen::Vector3d
 */
Eigen::Vector3d dmrpToOmega(const Eigen::Vector3d& mrp, const Eigen::Vector3d& dmrp){
    Eigen::Matrix3d Binv(binvMrp(mrp));
    return 4*Binv*dmrp;
}

/**
 * Return the second order mrp derivative for a given mrp vector, first mrp derivative, body angular
 * velocity vector and body angular acceleration vector.
 * (d^2Q)/(dt^2) = 1/4 ( [B(Q)] dw + [Bdot(Q,dQ)] w )
 * @param mrp
 * @param dmrp
 * @param omega
 * @param domega
 * @return Eigen::Vector3d
 */
Eigen::Vector3d ddmrp(const Eigen::Vector3d& mrp,
                      const Eigen::Vector3d& dmrp,
                      const Eigen::Vector3d& omega,
                      const Eigen::Vector3d& domega){
    Eigen::Matrix3d B(bmatMrp(mrp));
    Eigen::Matrix3d Bdot(bmatDotMrp(mrp, dmrp));

    return (B*domega + Bdot*omega)/4;
}

/**
 * Return the angular rate for a given mrp vector and mrp derivative.
 * domega/dt = 4 [B(Q)]^(-1) ( ddQ - [Bdot(Q,dQ)] [B(Q)]^(-1) dQ )
 * @param mrp
 * @param dmrp
 * @param ddmrp
 * @return Eigen::Vector3d
 */
Eigen::Vector3d ddmrpTodOmega(const Eigen::Vector3d& mrp, const Eigen::Vector3d& dmrp, const Eigen::Vector3d& ddmrp){
    Eigen::Matrix3d Binv(binvMrp(mrp));
    Eigen::Matrix3d Bdot(bmatDotMrp(mrp, dmrp));
    Eigen::Vector3d diff(ddmrp - Bdot*Binv*dmrp);
    return 4*Binv*diff;
}

/**Return the PRV derivative for a given PRV vector and body angular velocity vector.
 * dQ/dt =  [B(Q)] omega
 * @param prv
 * @param omega
 * @return Eigen::Vector3d
 */
Eigen::Vector3d dprv(const Eigen::Vector3d& prv, const Eigen::Vector3d& omega){
    return bmatPrv(prv)*omega;
}

/**
 * Return the 321 Euler angle derivative vector for a given 321 Euler angle vector and body
 * angular velocity vector.
 * dQ/dt =  [B(Q)] omega
 * @param euler321
 * @param omega
 * @return Eigen::Vector3d
 */
Eigen::Vector3d deuler321(const Eigen::Vector3d& euler321, const Eigen::Vector3d& omega){
    return bmatEulerAngles321(euler321)*omega;
}

/**
 * Return the direction cosine matrix in terms of the Euler parameter vector. The first element is the
 * non-dimensional Euler parameter, while the remain three elements form the Euler parameter vector.
 * @param ep
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d epToDcm(const Eigen::Vector4d& ep){
    Eigen::Matrix3d dcm;

    dcm(0, 0) = ep(0) * ep(0) + ep(1) * ep(1) - ep(2) * ep(2) - ep(3) * ep(3);
    dcm(0, 1) = 2 * (ep(1) * ep(2) + ep(0) * ep(3));
    dcm(0, 2) = 2 * (ep(1) * ep(3) - ep(0) * ep(2));
    dcm(1, 0) = 2 * (ep(1) * ep(2) - ep(0) * ep(3));
    dcm(1, 1) = ep(0) * ep(0) - ep(1) * ep(1) + ep(2) * ep(2) - ep(3) * ep(3);
    dcm(1, 2) = 2 * (ep(2) * ep(3) + ep(0) * ep(1));
    dcm(2, 0) = 2 * (ep(1) * ep(3) + ep(0) * ep(2));
    dcm(2, 1) = 2 * (ep(2) * ep(3) - ep(0) * ep(1));
    dcm(2, 2) = ep(0) * ep(0) - ep(1) * ep(1) - ep(2) * ep(2) + ep(3) * ep(3);

    return dcm;
}

/**
 * Translate a Euler parameter vector into an mrp vector.
 * @param ep
 * @return Eigen::Vector3d
 */
Eigen::Vector3d epToMrp(const Eigen::Vector4d& ep){
    if (ep(0) >= 0){
        return ep.tail(3)/(1 + ep(0));
    } else {
        return -ep.tail(3)/(1 - ep(0));
    }
}

/**
 * Translates a Euler parameter vector into a principal rotation vector.
 * @param ep
 * @return Eigen::Vector3d
 */
Eigen::Vector3d epToPrv(const Eigen::Vector4d& ep){
    Eigen::Vector3d prv;
    if (std::abs(std::sin(aCosSafe(ep(0)))) < eps){return prv.setZero();}
    prv = ep.tail(3)/ std::sin(aCosSafe(ep(0))) * 2 * aCosSafe(ep(0));
    return prv;
}

/**
 * Translate a Euler parameter vector into the corresponding 321 Euler angle set.
 * @param ep
 * @return Eigen::Vector3d
 */
Eigen::Vector3d epToEulerAngles321(const Eigen::Vector4d& ep){
    Eigen::Vector3d euler321;

    euler321(0) = std::atan2(2 * (ep(1) * ep(2) + ep(0) * ep(3)),
                             ep(0) * ep(0) + ep(1) * ep(1) - ep(2) * ep(2) - ep(3) * ep(3));
    euler321(1) = aSinSafe(-2 * (ep(1) * ep(3) - ep(0) * ep(2)));
    euler321(2) = std::atan2(2 * (ep(2) * ep(3) + ep(0) * ep(1)),
                             ep(0) * ep(0) - ep(1) * ep(1) - ep(2) * ep(2) + ep(3) * ep(3));

    return euler321;
}

/**
 * Return the direction cosine matrix in terms of an mrp vector.
 * @param mrp
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d mrpToDcm(const Eigen::Vector3d& mrp){
    Eigen::Matrix3d dcm;
    dcm << 8 * tildeMatrix(mrp) * tildeMatrix(mrp) - 4*(1 - mrp.dot(mrp))* tildeMatrix(mrp);
    dcm << dcm/(1 + mrp.dot(mrp))/(1 + mrp.dot(mrp));
    return dcm + Eigen::Matrix3d::Identity();
}

/**
 * Translate the mrp vector into the Euler parameter vector.
 * @param mrp
 * @return Eigen::Vector4d
 */
Eigen::Vector4d mrpToEp(const Eigen::Vector3d& mrp){
    Eigen::Vector4d ep;
    ep(0) = 1 - mrp.dot(mrp);
    ep.tail(3) << 2 * mrp;
    return ep/ (1 + mrp.dot(mrp));
}

/**
 * Translate a mrp vector into a principal rotation vector
 * @param mrp
 * @return Eigen::Vector3d
 */
Eigen::Vector3d mrpToPrv(const Eigen::Vector3d& mrp){
    Eigen::Vector3d prv;
    if(mrp.norm() < eps){return prv.setZero();}
    return mrp/mrp.norm() * 4 * std::atan(mrp.norm());
}

/**
 * Translate a MRP vector into a 321 Euler angle vector.
 * @param mrp
 * @return Eigen::Vector3d
 */
Eigen::Vector3d mrpToEulerAngles321(const Eigen::Vector3d& mrp){
    return epToEulerAngles321(mrpToEp(mrp));
}

/**
 * Check if mrp norm is larger than s. If so, map mrp to its shadow set.
 * @param mrp
 * @param s
 * @return Eigen::Vector3d
 */
Eigen::Vector3d mrpSwitch(const Eigen::Vector3d& mrp, const double s){
    if(mrp.dot(mrp) > s*s) {
        return mrpShadow(mrp);
    } else {
        return mrp;
    }
}

/**
 *  Switch from the current mrp to its shadow set
 * @param mrp
 * @return Eigen::Vector3d
 */
Eigen::Vector3d mrpShadow(const Eigen::Vector3d& mrp){
    return - mrp/mrp.dot(mrp);
}

/**
 * Return the direction cosine matrix corresponding to a principal rotation vector
 * @param prv
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d prvToDcm(const Eigen::Vector3d& prv){
    Eigen::Vector3d unitVector(prv/prv.norm());
    if(prv.norm() < eps){return  Eigen::Matrix3d::Identity();}

    Eigen::Matrix3d dcm;
    dcm(0, 0) = unitVector(0) * unitVector(0) * (1 - cos(prv.norm())) + cos(prv.norm());
    dcm(0, 1) = unitVector(0) * unitVector(1) * (1 - cos(prv.norm())) + unitVector(2) * sin(prv.norm());
    dcm(0, 2) = unitVector(0) * unitVector(2) * (1 - cos(prv.norm())) - unitVector(1) * sin(prv.norm());
    dcm(1, 0) = unitVector(1) * unitVector(0) * (1 - cos(prv.norm())) - unitVector(2) * sin(prv.norm());
    dcm(1, 1) = unitVector(1) * unitVector(1) * (1 - cos(prv.norm())) + cos(prv.norm());
    dcm(1, 2) = unitVector(1) * unitVector(2) * (1 - cos(prv.norm())) + unitVector(0) * sin(prv.norm());
    dcm(2, 0) = unitVector(2) * unitVector(0) * (1 - cos(prv.norm())) + unitVector(1) * sin(prv.norm());
    dcm(2, 1) = unitVector(2) * unitVector(1) * (1 - cos(prv.norm())) - unitVector(0) * sin(prv.norm());
    dcm(2, 2) = unitVector(2) * unitVector(2) * (1 - cos(prv.norm())) + cos(prv.norm());

    return dcm;
}

/**
 * Translate a principal rotation vector into an Euler parameter vector.
 * @param prv
 * @return Eigen::Vector4d
 */
Eigen::Vector4d prvToEp(const Eigen::Vector3d& prv){
    Eigen::Vector3d unitVector(prv/prv.norm());

    Eigen::Vector4d ep;
    ep(0) = cos(prv.norm() / 2);
    ep.tail(3) = unitVector *  sin(prv.norm() / 2);

    return ep;
}

/**
 * Translate the principal rotation vector into the mrp vector.
 * @param prv
 * @return Eigen::Vector3d
 */
Eigen::Vector3d prvToMrp(const Eigen::Vector3d& prv){
    return prv/prv.norm() * tan(prv.norm() / 4.);
}

/**
 * Translate a principal rotation vector into a 321 Euler angle vector.
 * @param prv
 * @return Eigen::Vector3d
 */
Eigen::Vector3d prvToEulerAngles321(const Eigen::Vector3d& prv){
    return epToEulerAngles321(prvToEp(prv));
}

/**
 * Return the direction cosine matrix corresponding to a 321 Euler angle rotation.
 * @param euler321
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d eulerAngles321ToDcm(const Eigen::Vector3d& euler321){
    double sin1 = std::sin(euler321(0));
    double sin2 = std::sin(euler321(1));
    double sin3 = std::sin(euler321(2));
    double cos1 = std::cos(euler321(0));
    double cos2 = std::cos(euler321(1));
    double cos3  = std::cos(euler321(2));

    Eigen::Matrix3d dcm;
    dcm << cos2 * cos1, cos2 * sin1, - sin2,
    sin3 * sin2 * cos1 - cos3 * sin1, sin3 * sin2 * sin1 + cos3 * cos1, sin3 * cos2,
    cos3 * sin2 * cos1 + sin3 * sin1, cos3 * sin2 * sin1 - sin3 * cos1, cos3 * cos2;

    return dcm;
}

/**
 * Translate a 321 Euler angle vector into the Euler parameter vector.
 * @param euler321
 * @return Eigen::Vector4d
 */
Eigen::Vector4d eulerAngles321ToEp(const Eigen::Vector3d& euler321){
    double cos1 = std::cos(euler321(0) / 2);
    double cos2 = std::cos(euler321(1) / 2);
    double cos3 = std::cos(euler321(2) / 2);
    double sin1 = std::sin(euler321(0) / 2);
    double sin2 = std::sin(euler321(1) / 2);
    double sin3 = std::sin(euler321(2) / 2);

    Eigen::Vector4d ep;
    ep(0) = cos1 * cos2 * cos3 + sin1 * sin2 * sin3;
    ep(1) = cos1 * cos2 * sin3 - sin1 * sin2 * cos3;
    ep(2) = cos1 * sin2 * cos3 + sin1 * cos2 * sin3;
    ep(3) = sin1 * cos2 * cos3 - cos1 * sin2 * sin3;

    return ep;
}

/**
 * Translate a 321 Euler angle vector into the MRP vector.
 * @param euler321
 * @return Eigen::Vector3d
 */
Eigen::Vector3d eulerAngles321ToMrp(const Eigen::Vector3d& euler321){
    return epToMrp(eulerAngles321ToEp(euler321));
}

/**
 * Translate a 321 Euler angle vector into a principal rotation vector.
 * @param euler321
 * @return Eigen::Vector3d
 */
Eigen::Vector3d eulerAngles321ToPrv(const Eigen::Vector3d& euler321){
    return epToPrv(eulerAngles321ToEp(euler321));
}

/**
 * Provide the Euler parameter vector which corresponds to relative rotation from B2 to B1.
 * @param ep1
 * @param ep2
 * @return Eigen::Vector4d
 */
Eigen::Vector4d subEp(const Eigen::Vector4d& ep1, const Eigen::Vector4d& ep2){
    Eigen::Vector4d ep;

    ep(0) = ep2(0) * ep1(0) + ep2(1) * ep1(1) + ep2(2) * ep1(2) + ep2(3) * ep1(3);
    ep(1) = -ep2(1) * ep1(0) + ep2(0) * ep1(1) + ep2(3) * ep1(2) - ep2(2) * ep1(3);
    ep(2) = -ep2(2) * ep1(0) - ep2(3) * ep1(1) + ep2(0) * ep1(2) + ep2(1) * ep1(3);
    ep(3) = -ep2(3) * ep1(0) + ep2(2) * ep1(1) - ep2(1) * ep1(2) + ep2(0) * ep1(3);

    return ep;
}

/**
 * Provide the MRP vector which corresponds to relative rotation from Q2 to Q1.
 * @param mrp1
 * @param mrp2
 * @return Eigen::Vector3d
 */
Eigen::Vector3d subMrp(const Eigen::Vector3d& mrp1, const Eigen::Vector3d& mrp2){
    Eigen::Vector3d mrp1Shadow(mrp1);
    double denominator = 1 + mrp2.dot(mrp2)*mrp1.dot(mrp1) + 2 * mrp2.dot(mrp1);
    if (std::abs(denominator) < 0.1) {
        mrp1Shadow = mrpShadow(mrp1); // Shadow set
        denominator = (1 + mrp2.dot(mrp2) * mrp1Shadow.dot(mrp1Shadow) + 2 * mrp2.dot(mrp1Shadow));
    }
    assert(std::abs(denominator) > eps);
    Eigen::Vector3d numerator;
    numerator << (1. - mrp2.dot(mrp2))*mrp1Shadow - (1. - mrp1Shadow.dot(mrp1Shadow))*mrp2 + 2*mrp1Shadow.cross(mrp2);
    Eigen::Vector3d mrp(numerator/denominator);
    /* map mrp to inner set */
    mrp = mrpSwitch(mrp, 1);

    return mrp;
}

/**
 * Provide the principal rotation vector which corresponds to relative principal rotation from Q2
 * to Q1.
 * @param prv1
 * @param prv2
 * @return Eigen::Vector3d
 */
Eigen::Vector3d subPrv(const Eigen::Vector3d& prv1, const Eigen::Vector3d& prv2){
    if(prv1.norm() < 1.0E-7 || prv2.norm() < 1.0E-7){return prv1 - prv2;}

    double cosPhi1 = std::cos(prv1.norm() / 2.);
    double cosPhi2 = std::cos(prv2.norm() / 2.);
    double sinPhi1 = std::sin(prv1.norm() / 2.);
    double sinPhi2 = std::sin(prv2.norm() / 2.);

    Eigen::Vector3d unitVector1(prv1/prv1.norm());
    Eigen::Vector3d unitVector2(prv2/prv2.norm());

    assert(std::abs(cosPhi1 * cosPhi2 + sinPhi1 * sinPhi2 * unitVector1.dot(unitVector2)) < 1);
    double angle = 2 * aCosSafe(cosPhi1 * cosPhi2 + sinPhi1 * sinPhi2 * unitVector1.dot(unitVector2));

    Eigen::Vector3d prv;
    if(std::abs(angle) < 1.0E-13){return prv.setZero();}
    prv << cosPhi2 * sinPhi1 * unitVector1 - cosPhi1 * sinPhi2 * unitVector2 +
            sinPhi1 * sinPhi2 * unitVector1.cross(unitVector2);
    return prv*angle/std::sin(angle/2);
}

/**
 * Compute the relative 321 Euler angle vector from E1 to E.
 * @param euler3211
 * @param euler3212
 * @return Eigen::Vector3d
 */
Eigen::Vector3d subEulerAngles321(const Eigen::Vector3d& euler3211, const Eigen::Vector3d& euler3212){
    return dcmToEulerAngles321(eulerAngles321ToDcm(euler3211)*eulerAngles321ToDcm(euler3212).transpose());
}

/**
 * Return the rotation matrix corresponding to a single axis rotation about axis a by the angle theta
 * @param angle
 * @param axis_number
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d rotationMatrix(const double angle, const int axis_number){
    assert(axis_number > 0 && axis_number < 4);
    Eigen::Matrix3d dcm;
    if (axis_number == 1){
        dcm << 1, 0, 0,
            0, std::cos(angle), std::sin(angle),
            0, -std::sin(angle), std::cos(angle);
    }
    else if (axis_number == 2){
        dcm << std::cos(angle), 0, -std::sin(angle),
            0, 1, 0,
            std::sin(angle), 0, std::cos(angle);
    }
    else if (axis_number == 3){
        dcm << std::cos(angle), std::sin(angle), 0,
            -std::sin(angle), std::cos(angle), 0,
            0, 0, 1;
    }
    return dcm;
}

/**
 * Return the the cross product matrix
 * @param vector
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d tildeMatrix(const Eigen::Vector3d& vector){
    Eigen::Matrix3d tilde;
    tilde << 0, -vector(2), vector(1),
            vector(2), 0, -vector(0),
            -vector(1), vector(0), 0;
    return tilde;
}

/**
 * Return the arc-cos of x where x is limited to |x| is less or equal to 1
 * @param x double value
 * @return double
 */
double aCosSafe (double x) {
    if (x < -1.0)
        return acos(-1);
    else if (x > 1.0)
        return acos(1) ;
    return acos (x) ;
}

/**
 * Return the arc-sin of x where x is limited to |x|<=1
 * @param x double value
 * @return double
 */
double aSinSafe (double x) {
    if (x < -1.0)
        return asin(-1);
    else if (x > 1.0)
        return asin(1) ;
    return asin (x) ;
}

/**
 * Return the square root of x where x is limited to x >= 0
 * @param x double value
 * @return double
 */
double sqrtSafe(double x) {
    if (x < 0.0)
        return 0.0;
    return sqrt(x);
}
