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

#include "architecture/utilities/attitudeKinematics.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <random>

std::random_device rd;
std::default_random_engine generator(rd());
std::uniform_real_distribution<double> angleDistribution(-3.14, 3.14);
const double kinematicsAccuracy = 1e-8;

// Test suite for additive properties of kinematic representations
class AdditiveTestSuite: public testing::TestWithParam<
                            std::tuple<std::string,
                            void (*)(double*, double*, double*),
                            std::function<Eigen::Vector3d(Eigen::Vector3d, Eigen::Vector3d)>,
                            std::function<Eigen::Matrix3d(Eigen::Vector3d)>>> {
    public:
        struct PrintToStringParamName {
            template<class ParamType>
            std::string operator()(const testing::TestParamInfo<ParamType> &info) const {
                return std::get<0>(info.param);
            }
        };
};

TEST_P(AdditiveTestSuite, additiveProperties)
{
    Eigen::Vector3d representation1;
    Eigen::Vector3d representation2;
    Eigen::Vector3d expected;
    double expectedArray[3];

    auto [testName, cFunction, cppFunction, dcmFunction] = GetParam();

    representation1 << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    representation2 << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    cFunction(representation1.data(), representation2.data(), expectedArray);
    expected = cArray2EigenVector3d(expectedArray);

    Eigen::Matrix3d dcm;
    dcm = dcmFunction(cppFunction(representation1, representation2));

    EXPECT_TRUE((dcm*dcm.transpose() - Eigen::Matrix3d::Identity()).norm() < kinematicsAccuracy);
    EXPECT_TRUE((cppFunction(representation1, representation2) - expected).norm() < kinematicsAccuracy);
}

INSTANTIATE_TEST_SUITE_P(
  RigidBodyKinematics,
  AdditiveTestSuite,
  ::testing::Values(
        std::make_tuple("addMrps", addMRP, addMrp, mrpToDcm),
        std::make_tuple("addPrvs", addPRV, addPrv, prvToDcm),
        std::make_tuple("addEulerAngles", addEuler321, addEulerAngles321, eulerAngles321ToDcm),
        std::make_tuple("subMrps", subMRP, subMrp, mrpToDcm),
        std::make_tuple("subPrvs", subPRV, subPrv, prvToDcm),
        std::make_tuple("subEulerAngles", subEuler321, subEulerAngles321, eulerAngles321ToDcm)),
    AdditiveTestSuite::PrintToStringParamName()
);

// Extra tests are done on Euler Parameters and their different size makes them harder to test
TEST(RigidBodyKinematics, addEulerParameters){
    Eigen::Vector4d ep1;
    Eigen::Vector4d ep2;
    Eigen::Vector4d expected;
    double expectedArray[4];

    // Generate a random euler parameter satisfying the holonomic constraint
    double phi1 = angleDistribution(generator);
    double phi2 = angleDistribution(generator);
    Eigen::Vector3d unitVector1;
    Eigen::Vector3d unitVector2;
    unitVector1 << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    unitVector1 << unitVector1.normalized();
    unitVector2 << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    unitVector2 << unitVector2.normalized();

    ep1 << std::cos(phi1/2), std::sin(phi1/2)*unitVector1(0), std::sin(phi1/2)*unitVector1(1), std::sin(phi1/2)*unitVector1(2);
    ep2 << std::cos(phi2/2), std::sin(phi2/2)*unitVector2(0), std::sin(phi2/2)*unitVector2(1), std::sin(phi2/2)*unitVector2(2);

    EXPECT_TRUE(std::abs(ep1(0)*ep1(0) + ep1(1)*ep1(1) + ep1(2)*ep1(2) + ep1(3)*ep1(3) - 1) < kinematicsAccuracy);
    EXPECT_TRUE(std::abs(ep2(0)*ep2(0) + ep2(1)*ep2(1) + ep2(2)*ep2(2) + ep2(3)*ep2(3) - 1) < kinematicsAccuracy);

    addEP(ep1.data(), ep2.data(), expectedArray);
    expected = Eigen::Map<Eigen::Vector4d>(expectedArray);

    Eigen::Vector4d ep;
    ep = addEp(ep1, ep2);

    EXPECT_TRUE(std::abs(ep(0)*ep(0) + ep(1)*ep(1) + ep(2)*ep(2) + ep(3)*ep(3) - 1) < kinematicsAccuracy);
    EXPECT_TRUE((ep - expected).norm() < kinematicsAccuracy);
}

// Extra tests are done on Euler Parameters and their different size makes them harder to test
TEST(RigidBodyKinematics, subEulerParameters){
    Eigen::Vector4d ep1;
    Eigen::Vector4d ep2;
    Eigen::Vector4d expected;
    double expectedArray[4];

    // Generate a random euler parameter satisfying the holonomic constraint
    double phi1 = angleDistribution(generator);
    double phi2 = angleDistribution(generator);
    Eigen::Vector3d unitVector1;
    Eigen::Vector3d unitVector2;
    unitVector1 << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    unitVector1 << unitVector1.normalized();
    unitVector2 << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    unitVector2 << unitVector2.normalized();

    ep1 << std::cos(phi1/2), std::sin(phi1/2)*unitVector1(0), std::sin(phi1/2)*unitVector1(1), std::sin(phi1/2)*unitVector1(2);
    ep2 << std::cos(phi2/2), std::sin(phi2/2)*unitVector2(0), std::sin(phi2/2)*unitVector2(1), std::sin(phi2/2)*unitVector2(2);

    EXPECT_TRUE(std::abs(ep1(0)*ep1(0) + ep1(1)*ep1(1) + ep1(2)*ep1(2) + ep1(3)*ep1(3) - 1) < kinematicsAccuracy);
    EXPECT_TRUE(std::abs(ep2(0)*ep2(0) + ep2(1)*ep2(1) + ep2(2)*ep2(2) + ep2(3)*ep2(3) - 1) < kinematicsAccuracy);

    subEP(ep1.data(), ep2.data(), expectedArray);
    expected = Eigen::Map<Eigen::Vector4d>(expectedArray);

    Eigen::Vector4d ep;
    ep = subEp(ep1, ep2);

    EXPECT_TRUE(std::abs(ep(0)*ep(0) + ep(1)*ep(1) + ep(2)*ep(2) + ep(3)*ep(3) - 1) < kinematicsAccuracy);
    EXPECT_TRUE((ep - expected).norm() < kinematicsAccuracy);
}


// Bmatrix Tests are hard to gather under a single fixture (invariants differ and sizes differ
TEST(RigidBodyKinematics, bmatrixInverseEulerParameters){
    Eigen::Vector4d ep;
    Eigen::Matrix<double, 3, 4> expected;
    double expectedArray[3][4];

    // Generate a random euler parameter satisfying the holonomic constraint
    double phi = angleDistribution(generator);
    Eigen::Vector3d unitVector;
    unitVector << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    unitVector << unitVector.normalized();

    ep << std::cos(phi/2), std::sin(phi/2)*unitVector(0), std::sin(phi/2)*unitVector(1), std::sin(phi/2)*unitVector(2);

    BinvEP(ep.data(), expectedArray);
    expected = cArray2EigenMatrixXd(reinterpret_cast<double *>(&expectedArray), 4, 3).transpose();

    EXPECT_TRUE((binvEp(ep) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, bmatrixInverseMrp){
    Eigen::Vector3d mrp;
    Eigen::Matrix3d expected;
    double expectedArray[3][3];

    mrp << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    BinvMRP(mrp.data(), expectedArray);
    expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(&expectedArray));

    EXPECT_TRUE(((1 + mrp.dot(mrp))*(1 + mrp.dot(mrp))*binvMrp(mrp) - bmatMrp(mrp).transpose()).norm() < kinematicsAccuracy);
    EXPECT_TRUE((binvMrp(mrp) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, bmatrixInversePrv){
    Eigen::Vector3d prv;
    Eigen::Matrix3d expected;
    double expectedArray[3][3];

    prv << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    BinvPRV(prv.data(), expectedArray);
    expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(&expectedArray));

    EXPECT_TRUE((binvPrv(prv)*bmatPrv(prv) - Eigen::Matrix3d::Identity()).norm() < kinematicsAccuracy);
    EXPECT_TRUE((binvPrv(prv) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, bmatrixInverseEulerAngles321){
    Eigen::Vector3d euler321;
    Eigen::Matrix3d expected;
    double expectedArray[3][3];

    euler321 << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    BinvEuler321(euler321.data(), expectedArray);
    expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(&expectedArray));

    EXPECT_TRUE((binvEulerAngles321(euler321) - expected).norm() < kinematicsAccuracy);
}


TEST(RigidBodyKinematics, bmatrixEulerParameter){
    Eigen::Vector4d ep;
    Eigen::Matrix<double, 4, 3> expected;
    double expectedArray[4][3];

    // Generate a random euler parameter satisfying the holonomic constraint
    double phi = angleDistribution(generator);
    Eigen::Vector3d unitVector;
    unitVector << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    unitVector << unitVector.normalized();

    ep << std::cos(phi/2), std::sin(phi/2)*unitVector(0), std::sin(phi/2)*unitVector(1), std::sin(phi/2)*unitVector(2);

    BmatEP(ep.data(), expectedArray);
    expected = cArray2EigenMatrixXd(reinterpret_cast<double *>(&expectedArray), 3, 4).transpose();

    EXPECT_TRUE((bmatEp(ep).transpose()*ep).norm() < kinematicsAccuracy);
    EXPECT_TRUE((bmatEp(ep) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, bMatrixMrp){
    Eigen::Vector3d mrp;
    Eigen::Matrix3d expected;
    double expectedArray[3][3];

    mrp << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    BmatMRP(mrp.data(), expectedArray);
    expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(&expectedArray));


    EXPECT_TRUE((bmatMrp(mrp).transpose()*bmatMrp(mrp) - (1 + mrp.dot(mrp))*(1 + mrp.dot(mrp))*
                        Eigen::Matrix3d::Identity()).norm()< kinematicsAccuracy);
    EXPECT_TRUE((bmatMrp(mrp) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, bMatrixDotMrp){
    Eigen::Vector3d mrp;
    Eigen::Vector3d dmrp;
    Eigen::Matrix3d expected;
    double expectedArray[3][3];

    mrp << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    dmrp << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    BdotmatMRP(mrp.data(), dmrp.data(), expectedArray);
    expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(&expectedArray));

    EXPECT_TRUE((bmatDotMrp(mrp, dmrp) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, bMatrixPrv){
    Eigen::Vector3d prv;
    Eigen::Matrix3d expected;
    double expectedArray[3][3];

    prv << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    BmatPRV(prv.data(), expectedArray);
    expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(&expectedArray));

    EXPECT_TRUE((bmatPrv(prv) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, bmatrixEulerAngles321){
    Eigen::Vector3d euler321;
    Eigen::Matrix3d expected;
    double expectedArray[3][3];

    euler321 << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    BmatEuler321(euler321.data(), expectedArray);
    expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(&expectedArray));

    EXPECT_TRUE((bmatEulerAngles321(euler321) - expected).norm() < kinematicsAccuracy);
}

// Test suite for dcm conversions of kinematic representations
class DcmToRepresentationTestSuite:
public testing::TestWithParam<std::tuple<std::string,
                            void (*)(double[3][3], double*),
                            std::function<Eigen::Vector3d(Eigen::Matrix3d)>>>{
    public:
        struct PrintToStringParamName {
            template<class ParamType>
            std::string operator()(const testing::TestParamInfo<ParamType> &info) const {
                return std::get<0>(info.param);
            }
        };
};


TEST_P(DcmToRepresentationTestSuite, dcmToRepresentationTransformations)
{
    Eigen::Matrix3d dcm1;
    Eigen::Matrix3d dcm2;
    Eigen::Matrix3d dcm3;
    Eigen::Matrix3d dcm;
    Eigen::Vector3d expected;
    double expectedArray[3];
    double dcmArray[3][3];

    auto [testName, cFunction, cppFunction] = GetParam();

    dcm1 = rotationMatrix(angleDistribution(generator), 1);
    dcm2 = rotationMatrix(angleDistribution(generator), 2);
    dcm3 = rotationMatrix(angleDistribution(generator), 3);

    dcm = dcm3*dcm2*dcm1;

    eigenMatrix3d2CArray(dcm, reinterpret_cast<double *>(dcmArray));
    cFunction(dcmArray, expectedArray);
    expected = Eigen::Map<Eigen::Vector3d>(expectedArray);

    EXPECT_TRUE((dcm*dcm.transpose() - Eigen::Matrix3d::Identity()).norm() < kinematicsAccuracy);
    EXPECT_TRUE((cppFunction(dcm) - expected).norm() < kinematicsAccuracy);
}

INSTANTIATE_TEST_SUITE_P(
    RigidBodyKinematics,
    DcmToRepresentationTestSuite,
    ::testing::Values(
        std::make_tuple("dcmToMrp", C2MRP, dcmToMrp),
        std::make_tuple("dcmToPrv", C2PRV, dcmToPrv),
        std::make_tuple("dcmToEulerAngles321", C2Euler321, dcmToEulerAngles321)),
    DcmToRepresentationTestSuite::PrintToStringParamName()
);

// Euler Parameters are of a different size and the function I/Os need to be the same for TestWithParam
TEST(RigidBodyKinematics, dcmToEulerParameter){
    Eigen::Matrix3d dcm1;
    Eigen::Matrix3d dcm2;
    Eigen::Matrix3d dcm3;
    Eigen::Matrix3d dcm;
    Eigen::Vector4d expected;
    double expectedArray[4];
    double dcmArray[3][3];

    dcm1 = rotationMatrix(angleDistribution(generator), 1);
    dcm2 = rotationMatrix(angleDistribution(generator), 2);
    dcm3 = rotationMatrix(angleDistribution(generator), 3);

    dcm = dcm3*dcm2*dcm1;

    eigenMatrix3d2CArray(dcm, reinterpret_cast<double *>(dcmArray));
    C2EP(dcmArray, expectedArray);
    expected = Eigen::Map<Eigen::Vector4d>(expectedArray);

    EXPECT_TRUE((dcm*dcm.transpose() - Eigen::Matrix3d::Identity()).norm() < kinematicsAccuracy);
    EXPECT_TRUE((dcmToEp(dcm) - expected).norm() < kinematicsAccuracy);
}

// Test suite for kinematic representations converted to dcms
class RepresentationToDcmTestSuite:
public testing::TestWithParam<std::tuple<std::string,
                            void (*)(double*, double[3][3]),
                            std::function<Eigen::Matrix3d(Eigen::Vector3d)>>>{
    public:
        struct PrintToStringParamName {
            template<class ParamType>
            std::string operator()(const testing::TestParamInfo<ParamType> &info) const {
                return std::get<0>(info.param);
            }
        };
};

TEST_P(RepresentationToDcmTestSuite, representationToDcmTransformations)
{
    Eigen::Vector3d representation;
    Eigen::Matrix3d expected;
    double expectedArray[3][3];

    auto [testName, cFunction, cppFunction] = GetParam();

    representation << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    cFunction(representation.data(), expectedArray);
    expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(expectedArray));

    EXPECT_TRUE((cppFunction(representation)*cppFunction(representation).transpose() - Eigen::Matrix3d::Identity()).norm() < kinematicsAccuracy);
    EXPECT_TRUE((cppFunction(representation) - expected).norm() < kinematicsAccuracy);
}

INSTANTIATE_TEST_SUITE_P(
    RigidBodyKinematics,
    RepresentationToDcmTestSuite,
    ::testing::Values(
        std::make_tuple("mrpToDcm", MRP2C, mrpToDcm),
        std::make_tuple("prvToDcm", PRV2C, prvToDcm),
        std::make_tuple("eulerAngles321ToDcm", Euler3212C, eulerAngles321ToDcm)),
    RepresentationToDcmTestSuite::PrintToStringParamName()
);

// Euler Parameters are of a different size and the function I/Os need to be the same for TestWithParam
TEST(RigidBodyKinematics, eulerToDcmParameter){
    Eigen::Vector4d ep;
    Eigen::Matrix3d expected;
    double expectedArray[3][3];

    double phi = angleDistribution(generator);
    Eigen::Vector3d unitVector;
    unitVector << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    unitVector << unitVector.normalized();
    ep << std::cos(phi/2), std::sin(phi/2)*unitVector(0), std::sin(phi/2)*unitVector(1), std::sin(phi/2)*unitVector(2);

    EP2C(ep.data(), expectedArray);
    expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(expectedArray));

    EXPECT_TRUE((epToDcm(ep)*epToDcm(ep).transpose() - Eigen::Matrix3d::Identity()).norm() < kinematicsAccuracy);
    EXPECT_TRUE((epToDcm(ep) - expected).norm() < kinematicsAccuracy);
}

// Test suite for kinematic representations converted to dcms
class EpToRepresentationTestSuite:
public testing::TestWithParam<std::tuple<std::string,
                            void (*)(double*, double*),
                            std::function<Eigen::Vector3d(Eigen::Vector4d)>>>{
    public:
        struct PrintToStringParamName {
            template<class ParamType>
            std::string operator()(const testing::TestParamInfo<ParamType> &info) const {
                return std::get<0>(info.param);
            }
        };
};

TEST_P(EpToRepresentationTestSuite, epToRepresentationTransformations)
{
    Eigen::Vector4d ep;
    Eigen::Vector3d expected;
    double expectedArray[3];

    auto [testName, cFunction, cppFunction] = GetParam();

    // Generate a random euler parameter satisfying the holonomic constraint
    double phi = angleDistribution(generator);
    Eigen::Vector3d unitVector;
    unitVector << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    unitVector << unitVector.normalized();
    ep << std::cos(phi/2), std::sin(phi/2)*unitVector(0), std::sin(phi/2)*unitVector(1), std::sin(phi/2)*unitVector(2);

    cFunction(ep.data(), expectedArray);
    expected = Eigen::Map<Eigen::Vector3d>(expectedArray);

    EXPECT_TRUE((cppFunction(ep) - expected).norm() < kinematicsAccuracy);
}

INSTANTIATE_TEST_SUITE_P(
    RigidBodyKinematics,
    EpToRepresentationTestSuite,
    ::testing::Values(
        std::make_tuple("epToMrp", EP2MRP, epToMrp),
        std::make_tuple("epToPrv", EP2PRV, epToPrv),
        std::make_tuple("epToEulerAngles321", EP2Euler321, epToEulerAngles321)),
    EpToRepresentationTestSuite::PrintToStringParamName()
);

// Test suite for kinematic representations converted to dcms
class RepresentationToEpTestSuite:
public testing::TestWithParam<std::tuple<std::string,
                            void (*)(double*, double*),
                            std::function<Eigen::Vector4d(Eigen::Vector3d)>>>{
    public:
        struct PrintToStringParamName {
            template<class ParamType>
            std::string operator()(const testing::TestParamInfo<ParamType> &info) const {
                return std::get<0>(info.param);
            }
        };
};

TEST_P(RepresentationToEpTestSuite, representationToEpTransformations)
{
    Eigen::Vector3d representation;
    Eigen::Vector4d ep;
    Eigen::Vector4d expected;
    double expectedArray[4];

    auto [testName, cFunction, cppFunction] = GetParam();

    representation << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    cFunction(representation.data(), expectedArray);
    expected = Eigen::Map<Eigen::Vector4d>(expectedArray);
    ep = cppFunction(representation);

    EXPECT_TRUE(std::abs(ep(0)*ep(0) + ep(1)*ep(1) + ep(2)*ep(2) + ep(3)*ep(3) - 1) < kinematicsAccuracy);
    EXPECT_TRUE((ep - expected).norm() < kinematicsAccuracy);
}

INSTANTIATE_TEST_SUITE_P(
    RigidBodyKinematics,
    RepresentationToEpTestSuite,
    ::testing::Values(
        std::make_tuple("mrpToEp", MRP2EP, mrpToEp),
        std::make_tuple("prvToEp", PRV2EP, prvToEp),
        std::make_tuple("eulerAnglesToEp", Euler3212EP, eulerAngles321ToEp)),
    RepresentationToEpTestSuite::PrintToStringParamName()
);

// Test suite for kinematic representations converted to dcms
class RepresentationTransformationsTestSuite:
public testing::TestWithParam<std::tuple<std::string,
                            void (*)(double*, double*),
                            std::function<Eigen::Vector3d(Eigen::Vector3d)>>>{
    public:
        struct PrintToStringParamName {
            template<class ParamType>
            std::string operator()(const testing::TestParamInfo<ParamType> &info) const {
                return std::get<0>(info.param);
            }
        };
};

TEST_P(RepresentationTransformationsTestSuite, representationTransform3Vectors)
{
    Eigen::Vector3d representation;
    Eigen::Vector3d expected;
    double expectedArray[3];

    auto [testName, cFunction, cppFunction] = GetParam();

    representation << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    cFunction(representation.data(), expectedArray);
    expected = Eigen::Map<Eigen::Vector3d>(expectedArray);

    EXPECT_TRUE((cppFunction(representation) - expected).norm() < kinematicsAccuracy);
}

INSTANTIATE_TEST_SUITE_P(
    RigidBodyKinematics,
    RepresentationTransformationsTestSuite,
    ::testing::Values(
        std::make_tuple("mrpToPrv", MRP2PRV, mrpToPrv),
        std::make_tuple("prvToMrp", PRV2MRP, prvToMrp),
        std::make_tuple("eulerAngles321ToMrp", Euler3212MRP, eulerAngles321ToMrp),
        std::make_tuple("mrpToEulerAngles321", MRP2Euler321, mrpToEulerAngles321),
        std::make_tuple("prvToEulerAngles321", PRV2Euler321, prvToEulerAngles321),
        std::make_tuple("eulerAngles321ToPrv", Euler3212PRV, eulerAngles321ToPrv)),
    RepresentationTransformationsTestSuite::PrintToStringParamName()
);

// Test suite for kinematic representations converted to dcms
class RepresentationDerivativesTestSuite:
public testing::TestWithParam<std::tuple<std::string,
                            void (*)(double*, double*, double*),
                            std::function<Eigen::Vector3d(Eigen::Vector3d, Eigen::Vector3d)>>>{
    public:
        struct PrintToStringParamName {
            template<class ParamType>
            std::string operator()(const testing::TestParamInfo<ParamType> &info) const {
                return std::get<0>(info.param);
            }
        };
};

TEST_P(RepresentationDerivativesTestSuite, representationDerivatives3Vectors)
{
    Eigen::Vector3d representation;
    Eigen::Vector3d rate;
    Eigen::Vector3d expected;
    double expectedArray[3];

    auto [testName, cFunction, cppFunction] = GetParam();

    representation << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    rate << angleDistribution(generator)/10, angleDistribution(generator)/10, angleDistribution(generator)/10;

    cFunction(representation.data(), rate.data(), expectedArray);
    expected = Eigen::Map<Eigen::Vector3d>(expectedArray);

    EXPECT_TRUE((cppFunction(representation, rate) - expected).norm() < kinematicsAccuracy);
}

INSTANTIATE_TEST_SUITE_P(
    RigidBodyKinematics,
    RepresentationDerivativesTestSuite,
    ::testing::Values(
        std::make_tuple("dmrp",dMRP, dmrp),
        std::make_tuple("dmrpToOmega",dMRP2Omega, dmrpToOmega),
        std::make_tuple("dprv",dPRV, dprv),
        std::make_tuple("deuler321",dEuler321, deuler321)),
    RepresentationDerivativesTestSuite::PrintToStringParamName()
);

TEST(RigidBodyKinematics, epDerivativeTest)
{
    Eigen::Vector4d ep;
    Eigen::Vector3d omega;
    Eigen::Vector4d expected;
    double expectedArray[4];

    // Generate a random euler parameter satisfying the holonomic constraint
    double phi = angleDistribution(generator);
    Eigen::Vector3d unitVector;
    unitVector << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    unitVector << unitVector.normalized();
    ep << std::cos(phi/2), std::sin(phi/2)*unitVector(0), std::sin(phi/2)*unitVector(1), std::sin(phi/2)*unitVector(2);
    omega << angleDistribution(generator)/10, angleDistribution(generator)/10, angleDistribution(generator)/10;

    dEP(ep.data(), omega.data(), expectedArray);
    expected = Eigen::Map<Eigen::Vector4d>(expectedArray);

    EXPECT_TRUE((dep(ep, omega) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, dmrpDerivativeTest)
{
    Eigen::Vector3d mrp;
    Eigen::Vector3d dmrp;
    Eigen::Vector3d omega;
    Eigen::Vector3d domega;
    Eigen::Vector3d expected;
    double expectedArray[3];

    mrp << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    dmrp << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    omega << angleDistribution(generator)/10, angleDistribution(generator)/10, angleDistribution(generator)/10;
    domega << angleDistribution(generator)/10, angleDistribution(generator)/10, angleDistribution(generator)/10;

    ddMRP(mrp.data(), dmrp.data(),omega.data(), domega.data(), expectedArray);
    expected = Eigen::Map<Eigen::Vector3d>(expectedArray);

    EXPECT_TRUE((ddmrp(mrp, dmrp, omega, domega) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, dmrpToOmegaDerivativeTest)
{
    Eigen::Vector3d mrp;
    Eigen::Vector3d dmrp;
    Eigen::Vector3d ddmrp;
    Eigen::Vector3d expected;
    double expectedArray[3];

    mrp << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    dmrp << angleDistribution(generator)/10, angleDistribution(generator)/10, angleDistribution(generator)/10;
    ddmrp << angleDistribution(generator)/100, angleDistribution(generator)/100, angleDistribution(generator)/100;

    ddMRP2dOmega(mrp.data(), dmrp.data(), ddmrp.data(), expectedArray);
    expected = Eigen::Map<Eigen::Vector3d>(expectedArray);

    EXPECT_TRUE((ddmrpTodOmega(mrp, dmrp, ddmrp) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, mrpSwitchTest)
{
    Eigen::Vector3d mrp;
    Eigen::Vector3d expected;
    double value = angleDistribution(generator);
    double expectedArray[3];

    mrp << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    MRPswitch(mrp.data(), value, expectedArray);
    expected = Eigen::Map<Eigen::Vector3d>(expectedArray);

    EXPECT_TRUE((mrpSwitch(mrp, value) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, mrpShadowTest)
{
    Eigen::Vector3d mrp;
    Eigen::Vector3d expected;
    double expectedArray[3];

    mrp << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    MRPshadow(mrp.data(), expectedArray);
    expected = Eigen::Map<Eigen::Vector3d>(expectedArray);

    EXPECT_TRUE((mrpShadow(mrp) - expected).norm() < kinematicsAccuracy);
}

TEST(RigidBodyKinematics, rotationMatrixTest)
{
    Eigen::Vector3d mrp;
    Eigen::Matrix3d expected;
    double angle;
    double expectedArray[3][3];

    for (int i=1; i<4; i++){
        angle = angleDistribution(generator);
        Mi(angle, i, expectedArray);
        expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(expectedArray));
        EXPECT_TRUE((rotationMatrix(angle, i) - expected).norm() < kinematicsAccuracy);
    }
}

TEST(RigidBodyKinematics, tildeMatrixTest)
{
    Eigen::Vector3d vector;
    Eigen::Vector3d testVector;
    Eigen::Matrix3d expected;
    double expectedArray[3][3];

    vector << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);
    testVector << angleDistribution(generator), angleDistribution(generator), angleDistribution(generator);

    tilde(vector.data(), expectedArray);
    expected = cArray2EigenMatrix3d(reinterpret_cast<double *>(expectedArray));

    EXPECT_TRUE((tildeMatrix(vector)*vector).norm() < kinematicsAccuracy);
    EXPECT_TRUE((tildeMatrix(vector)*testVector - vector.cross(testVector)).norm() < kinematicsAccuracy);
    EXPECT_TRUE((tildeMatrix(vector) - expected).norm() < kinematicsAccuracy);
}
