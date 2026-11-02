/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include <Eigen/Geometry>

#include "architecture/utilities/linearAlgebra.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using LoopFunction = std::function<double(std::uint64_t)>;

volatile double benchmarkSink = 0.0;
volatile std::uint64_t perturbationSeed = 1;

struct TimedRun
{
    double nanosecondsPerOperation;
    double checksum;
};

struct BenchmarkResult
{
    std::string name;
    std::uint64_t iterations;
    double eigenNanoseconds;
    double linearAlgebraNanoseconds;
    double ratio;
    double relativeChecksumDifference;
};

struct BenchmarkOptions
{
    std::uint64_t iterationScale = 1;
    bool smokeMode = false;
};

void
printUsage(const char* executableName)
{
    std::cout << "Usage: " << executableName << " [iterationScale] [--smoke]\n"
              << "  iterationScale is a positive integer multiplier for the default loop counts.\n"
              << "  --smoke caps each operation's iteration count to verify that the benchmark still executes.\n";
}

BenchmarkOptions
parseOptions(int argc, char** argv)
{
    BenchmarkOptions options;
    bool scaleWasProvided = false;

    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex) {
        const std::string argument = argv[argumentIndex];
        if (argument == "-h" || argument == "--help") {
            printUsage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (argument == "--smoke") {
            options.smokeMode = true;
            continue;
        }
        if (!argument.empty() && argument.front() == '-') {
            throw std::runtime_error("unknown option '" + argument + "'.");
        }
        if (scaleWasProvided) {
            throw std::runtime_error("only one iterationScale argument is allowed.");
        }

        std::size_t parsedCharacters = 0;
        const std::uint64_t scale = std::stoull(argument, &parsedCharacters);
        if (parsedCharacters != argument.size() || scale == 0) {
            throw std::runtime_error("iterationScale must be a positive integer.");
        }
        options.iterationScale = scale;
        scaleWasProvided = true;
    }

    return options;
}

std::uint64_t
scaledIterations(std::uint64_t baseIterations, std::uint64_t scale)
{
    if (baseIterations > std::numeric_limits<std::uint64_t>::max() / scale) {
        throw std::overflow_error("iteration count overflow.");
    }
    return baseIterations * scale;
}

std::uint64_t
benchmarkIterations(std::uint64_t baseIterations, const BenchmarkOptions& options)
{
    if (options.smokeMode) {
        constexpr std::uint64_t smokeIterationCap = 1000;
        return std::min(baseIterations, smokeIterationCap);
    }
    return scaledIterations(baseIterations, options.iterationScale);
}

double
perturbation(std::uint64_t iteration)
{
    constexpr double perturbationStep = 1.0e-12;
    constexpr std::uint64_t perturbationPeriod = 97;
    return perturbationStep * static_cast<double>(((iteration + perturbationSeed) % perturbationPeriod) + 1);
}

std::uint64_t
warmupIterations(std::uint64_t iterations)
{
    constexpr std::uint64_t warmupDivisor = 200;
    constexpr std::uint64_t maxWarmupIterations = 2000;
    return std::max<std::uint64_t>(1, std::min(iterations / warmupDivisor, maxWarmupIterations));
}

TimedRun
timeLoop(const LoopFunction& loopFunction, std::uint64_t iterations)
{
    const auto start = Clock::now();
    const double checksum = loopFunction(iterations);
    const auto finish = Clock::now();
    const double elapsedNanoseconds = std::chrono::duration<double, std::nano>(finish - start).count();

    benchmarkSink += checksum;
    return { elapsedNanoseconds / static_cast<double>(iterations), checksum };
}

double
relativeDifference(double firstValue, double secondValue)
{
    const double scale = std::max({ std::abs(firstValue), std::abs(secondValue), 1.0 });
    return std::abs(firstValue - secondValue) / scale;
}

BenchmarkResult
runBenchmarkPair(const std::string& name,
                 std::uint64_t iterations,
                 const LoopFunction& eigenLoop,
                 const LoopFunction& linearAlgebraLoop)
{
    const std::uint64_t warmup = warmupIterations(iterations);
    benchmarkSink += eigenLoop(warmup);
    benchmarkSink += linearAlgebraLoop(warmup);

    const TimedRun eigenRun = timeLoop(eigenLoop, iterations);
    const TimedRun linearAlgebraRun = timeLoop(linearAlgebraLoop, iterations);
    return { name,
             iterations,
             eigenRun.nanosecondsPerOperation,
             linearAlgebraRun.nanosecondsPerOperation,
             linearAlgebraRun.nanosecondsPerOperation / eigenRun.nanosecondsPerOperation,
             relativeDifference(eigenRun.checksum, linearAlgebraRun.checksum) };
}

double
matrixValue(std::size_t row, std::size_t col, double diagonalBoost)
{
    const double baseValue = 0.05 * static_cast<double>((row + 1) * (col + 2));
    if (row == col) {
        return diagonalBoost + baseValue;
    }
    return baseValue / static_cast<double>(row + col + 2);
}

void
fillMatrix3(double matrix[3][3], double diagonalBoost)
{
    for (std::size_t row = 0; row < 3; row++) {
        for (std::size_t col = 0; col < 3; col++) {
            matrix[row][col] = matrixValue(row, col, diagonalBoost);
        }
    }
}

Eigen::Matrix3d
makeEigenMatrix3(double diagonalBoost)
{
    Eigen::Matrix3d matrix;
    for (Eigen::Index row = 0; row < matrix.rows(); row++) {
        for (Eigen::Index col = 0; col < matrix.cols(); col++) {
            matrix(row, col) = matrixValue(static_cast<std::size_t>(row), static_cast<std::size_t>(col), diagonalBoost);
        }
    }
    return matrix;
}

void
fillMatrix4(double matrix[4][4], double diagonalBoost)
{
    for (std::size_t row = 0; row < 4; row++) {
        for (std::size_t col = 0; col < 4; col++) {
            matrix[row][col] = matrixValue(row, col, diagonalBoost);
        }
    }
}

Eigen::Matrix4d
makeEigenMatrix4(double diagonalBoost)
{
    Eigen::Matrix4d matrix;
    for (Eigen::Index row = 0; row < matrix.rows(); row++) {
        for (Eigen::Index col = 0; col < matrix.cols(); col++) {
            matrix(row, col) = matrixValue(static_cast<std::size_t>(row), static_cast<std::size_t>(col), diagonalBoost);
        }
    }
    return matrix;
}

void
fillMatrix6(double matrix[6][6], double diagonalBoost)
{
    for (std::size_t row = 0; row < 6; row++) {
        for (std::size_t col = 0; col < 6; col++) {
            matrix[row][col] = matrixValue(row, col, diagonalBoost);
        }
    }
}

Matrix6d
makeEigenMatrix6(double diagonalBoost)
{
    Matrix6d matrix;
    for (Eigen::Index row = 0; row < matrix.rows(); row++) {
        for (Eigen::Index col = 0; col < matrix.cols(); col++) {
            matrix(row, col) = matrixValue(static_cast<std::size_t>(row), static_cast<std::size_t>(col), diagonalBoost);
        }
    }
    return matrix;
}

std::vector<double>
makeLinearAlgebraMatrix(std::size_t dimension, double diagonalBoost)
{
    std::vector<double> matrix(dimension * dimension);
    for (std::size_t row = 0; row < dimension; row++) {
        for (std::size_t col = 0; col < dimension; col++) {
            matrix[row * dimension + col] = matrixValue(row, col, diagonalBoost);
        }
    }
    return matrix;
}

Eigen::MatrixXd
makeEigenDynamicMatrix(std::size_t dimension, double diagonalBoost)
{
    Eigen::MatrixXd matrix(static_cast<Eigen::Index>(dimension), static_cast<Eigen::Index>(dimension));
    for (Eigen::Index row = 0; row < matrix.rows(); row++) {
        for (Eigen::Index col = 0; col < matrix.cols(); col++) {
            matrix(row, col) = matrixValue(static_cast<std::size_t>(row), static_cast<std::size_t>(col), diagonalBoost);
        }
    }
    return matrix;
}

double
runEigenVector3Dot(std::uint64_t iterations)
{
    Eigen::Vector3d vectorA(1.1, -2.2, 3.3);
    Eigen::Vector3d vectorB(-4.4, 5.5, -6.6);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        vectorA(0) += offset;
        vectorB(2) -= offset;
        checksum += vectorA.dot(vectorB);
        vectorB(2) += offset;
        vectorA(0) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraVector3Dot(std::uint64_t iterations)
{
    double vectorA[3] = { 1.1, -2.2, 3.3 };
    double vectorB[3] = { -4.4, 5.5, -6.6 };
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        vectorA[0] += offset;
        vectorB[2] -= offset;
        checksum += v3Dot(vectorA, vectorB);
        vectorB[2] += offset;
        vectorA[0] -= offset;
    }
    return checksum;
}

double
runEigenVector3Cross(std::uint64_t iterations)
{
    Eigen::Vector3d vectorA(1.1, -2.2, 3.3);
    Eigen::Vector3d vectorB(-4.4, 5.5, -6.6);
    Eigen::Vector3d result;
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        vectorA(1) += offset;
        vectorB(0) -= offset;
        result = vectorA.cross(vectorB);
        checksum += result(0) + result(1) + result(2);
        vectorB(0) += offset;
        vectorA(1) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraVector3Cross(std::uint64_t iterations)
{
    double vectorA[3] = { 1.1, -2.2, 3.3 };
    double vectorB[3] = { -4.4, 5.5, -6.6 };
    double result[3];
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        vectorA[1] += offset;
        vectorB[0] -= offset;
        v3Cross(vectorA, vectorB, result);
        checksum += result[0] + result[1] + result[2];
        vectorB[0] += offset;
        vectorA[1] -= offset;
    }
    return checksum;
}

double
runEigenMatrix3Multiply(std::uint64_t iterations)
{
    Eigen::Matrix3d matrixA = makeEigenMatrix3(4.0);
    Eigen::Matrix3d matrixB = makeEigenMatrix3(1.0);
    Eigen::Matrix3d result;
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA(0, 0) += offset;
        matrixB(2, 1) -= offset;
        result.noalias() = matrixA * matrixB;
        checksum += result(0, 0) + result(1, 2) + result(2, 1);
        matrixB(2, 1) += offset;
        matrixA(0, 0) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraMatrix3Multiply(std::uint64_t iterations)
{
    double matrixA[3][3];
    double matrixB[3][3];
    double result[3][3];
    fillMatrix3(matrixA, 4.0);
    fillMatrix3(matrixB, 1.0);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA[0][0] += offset;
        matrixB[2][1] -= offset;
        m33MultM33(matrixA, matrixB, result);
        checksum += result[0][0] + result[1][2] + result[2][1];
        matrixB[2][1] += offset;
        matrixA[0][0] -= offset;
    }
    return checksum;
}

double
runEigenMatrix3TransposeMultiply(std::uint64_t iterations)
{
    Eigen::Matrix3d matrixA = makeEigenMatrix3(4.0);
    Eigen::Matrix3d matrixB = makeEigenMatrix3(1.0);
    Eigen::Matrix3d result;
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA(1, 0) += offset;
        matrixB(2, 2) -= offset;
        result.noalias() = matrixA.transpose() * matrixB;
        checksum += result(0, 0) + result(1, 2) + result(2, 1);
        matrixB(2, 2) += offset;
        matrixA(1, 0) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraMatrix3TransposeMultiply(std::uint64_t iterations)
{
    double matrixA[3][3];
    double matrixB[3][3];
    double result[3][3];
    fillMatrix3(matrixA, 4.0);
    fillMatrix3(matrixB, 1.0);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA[1][0] += offset;
        matrixB[2][2] -= offset;
        m33tMultM33(matrixA, matrixB, result);
        checksum += result[0][0] + result[1][2] + result[2][1];
        matrixB[2][2] += offset;
        matrixA[1][0] -= offset;
    }
    return checksum;
}

double
runEigenMatrix3VectorMultiply(std::uint64_t iterations)
{
    Eigen::Matrix3d matrixA = makeEigenMatrix3(4.0);
    Eigen::Vector3d vectorA(1.1, -2.2, 3.3);
    Eigen::Vector3d result;
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA(0, 1) += offset;
        vectorA(2) -= offset;
        result.noalias() = matrixA * vectorA;
        checksum += result(0) + result(1) + result(2);
        vectorA(2) += offset;
        matrixA(0, 1) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraMatrix3VectorMultiply(std::uint64_t iterations)
{
    double matrixA[3][3];
    double vectorA[3] = { 1.1, -2.2, 3.3 };
    double result[3];
    fillMatrix3(matrixA, 4.0);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA[0][1] += offset;
        vectorA[2] -= offset;
        m33MultV3(matrixA, vectorA, result);
        checksum += result[0] + result[1] + result[2];
        vectorA[2] += offset;
        matrixA[0][1] -= offset;
    }
    return checksum;
}

double
runEigenMatrix3Inverse(std::uint64_t iterations)
{
    Eigen::Matrix3d matrixA = makeEigenMatrix3(5.0);
    Eigen::Matrix3d result;
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA(0, 0) += offset;
        matrixA(2, 2) -= offset;
        result = matrixA.inverse();
        checksum += result(0, 0) + result(1, 2) + result(2, 1);
        matrixA(2, 2) += offset;
        matrixA(0, 0) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraMatrix3Inverse(std::uint64_t iterations)
{
    double matrixA[3][3];
    double result[3][3];
    fillMatrix3(matrixA, 5.0);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA[0][0] += offset;
        matrixA[2][2] -= offset;
        m33Inverse(matrixA, result);
        checksum += result[0][0] + result[1][2] + result[2][1];
        matrixA[2][2] += offset;
        matrixA[0][0] -= offset;
    }
    return checksum;
}

double
runEigenMatrix4Inverse(std::uint64_t iterations)
{
    Eigen::Matrix4d matrixA = makeEigenMatrix4(6.0);
    Eigen::Matrix4d result;
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA(0, 0) += offset;
        matrixA(3, 3) -= offset;
        result = matrixA.inverse();
        checksum += result(0, 0) + result(1, 3) + result(3, 1);
        matrixA(3, 3) += offset;
        matrixA(0, 0) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraMatrix4Inverse(std::uint64_t iterations)
{
    double matrixA[4][4];
    double result[4][4];
    fillMatrix4(matrixA, 6.0);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA[0][0] += offset;
        matrixA[3][3] -= offset;
        m44Inverse(matrixA, result);
        checksum += result[0][0] + result[1][3] + result[3][1];
        matrixA[3][3] += offset;
        matrixA[0][0] -= offset;
    }
    return checksum;
}

double
runEigenMatrix6Multiply(std::uint64_t iterations)
{
    Matrix6d matrixA = makeEigenMatrix6(5.0);
    Matrix6d matrixB = makeEigenMatrix6(2.0);
    Matrix6d result;
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA(0, 0) += offset;
        matrixB(5, 2) -= offset;
        result.noalias() = matrixA * matrixB;
        checksum += result(0, 0) + result(3, 5) + result(5, 2);
        matrixB(5, 2) += offset;
        matrixA(0, 0) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraMatrix6Multiply(std::uint64_t iterations)
{
    double matrixA[6][6];
    double matrixB[6][6];
    double result[6][6];
    fillMatrix6(matrixA, 5.0);
    fillMatrix6(matrixB, 2.0);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA[0][0] += offset;
        matrixB[5][2] -= offset;
        m66MultM66(matrixA, matrixB, result);
        checksum += result[0][0] + result[3][5] + result[5][2];
        matrixB[5][2] += offset;
        matrixA[0][0] -= offset;
    }
    return checksum;
}

double
runEigenMatrix6VectorMultiply(std::uint64_t iterations)
{
    Matrix6d matrixA = makeEigenMatrix6(5.0);
    Vector6d vectorA;
    vectorA << 1.1, -2.2, 3.3, -4.4, 5.5, -6.6;
    Vector6d result;
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA(0, 1) += offset;
        vectorA(5) -= offset;
        result.noalias() = matrixA * vectorA;
        checksum += result(0) + result(3) + result(5);
        vectorA(5) += offset;
        matrixA(0, 1) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraMatrix6VectorMultiply(std::uint64_t iterations)
{
    double matrixA[6][6];
    double vectorA[6] = { 1.1, -2.2, 3.3, -4.4, 5.5, -6.6 };
    double result[6];
    fillMatrix6(matrixA, 5.0);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA[0][1] += offset;
        vectorA[5] -= offset;
        m66MultV6(matrixA, vectorA, result);
        checksum += result[0] + result[3] + result[5];
        vectorA[5] += offset;
        matrixA[0][1] -= offset;
    }
    return checksum;
}

double
runEigenDynamicMultiply(std::uint64_t iterations, std::size_t dimension)
{
    Eigen::MatrixXd matrixA = makeEigenDynamicMatrix(dimension, 5.0);
    Eigen::MatrixXd matrixB = makeEigenDynamicMatrix(dimension, 2.0);
    Eigen::MatrixXd result(static_cast<Eigen::Index>(dimension), static_cast<Eigen::Index>(dimension));
    const Eigen::Index last = static_cast<Eigen::Index>(dimension - 1);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA(0, 0) += offset;
        matrixB(last, 2) -= offset;
        result.noalias() = matrixA * matrixB;
        checksum += result(0, 0) + result(last / 2, last) + result(last, 2);
        matrixB(last, 2) += offset;
        matrixA(0, 0) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraDynamicMultiply(std::uint64_t iterations, std::size_t dimension)
{
    std::vector<double> matrixA = makeLinearAlgebraMatrix(dimension, 5.0);
    std::vector<double> matrixB = makeLinearAlgebraMatrix(dimension, 2.0);
    std::vector<double> result(dimension * dimension);
    const std::size_t last = dimension - 1;
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA[0] += offset;
        matrixB[last * dimension + 2] -= offset;
        mMultM(matrixA.data(), dimension, dimension, matrixB.data(), dimension, dimension, result.data());
        checksum += result[0] + result[(last / 2) * dimension + last] + result[last * dimension + 2];
        matrixB[last * dimension + 2] += offset;
        matrixA[0] -= offset;
    }
    return checksum;
}

double
runEigenGeneralMatrix6Inverse(std::uint64_t iterations)
{
    constexpr std::size_t dimension = 6;
    Eigen::MatrixXd matrixA = makeEigenDynamicMatrix(dimension, 8.0);
    Eigen::MatrixXd result(dimension, dimension);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA(0, 0) += offset;
        matrixA(5, 5) -= offset;
        result = matrixA.inverse();
        checksum += result(0, 0) + result(2, 5) + result(5, 2);
        matrixA(5, 5) += offset;
        matrixA(0, 0) -= offset;
    }
    return checksum;
}

double
runLinearAlgebraGeneralMatrix6Inverse(std::uint64_t iterations)
{
    constexpr std::size_t dimension = 6;
    std::vector<double> matrixA = makeLinearAlgebraMatrix(dimension, 8.0);
    std::vector<double> result(dimension * dimension);
    double checksum = 0.0;

    for (std::uint64_t iteration = 0; iteration < iterations; iteration++) {
        const double offset = perturbation(iteration);
        matrixA[0] += offset;
        matrixA[5 * dimension + 5] -= offset;
        mInverse(matrixA.data(), dimension, result.data());
        checksum += result[0] + result[2 * dimension + 5] + result[5 * dimension + 2];
        matrixA[5 * dimension + 5] += offset;
        matrixA[0] -= offset;
    }
    return checksum;
}

void
printResults(const std::vector<BenchmarkResult>& results)
{
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n";
    std::cout << std::left << std::setw(34) << "Operation" << std::right << std::setw(12) << "Iterations"
              << std::setw(15) << "Eigen ns/op" << std::setw(20) << "linearAlg ns/op" << std::setw(18) << "linear/Eigen"
              << std::setw(18) << "checksum diff"
              << "\n";

    for (const BenchmarkResult& result : results) {
        std::cout << std::left << std::setw(34) << result.name << std::right << std::setw(12) << result.iterations
                  << std::setw(15) << result.eigenNanoseconds << std::setw(20) << result.linearAlgebraNanoseconds
                  << std::setw(18) << result.ratio << std::setw(18) << result.relativeChecksumDifference << "\n";
    }
}

} // namespace

int
main(int argc, char** argv)
{
    try {
        const BenchmarkOptions options = parseOptions(argc, argv);
        std::vector<BenchmarkResult> results;

        results.push_back(runBenchmarkPair(
          "v3 dot", benchmarkIterations(20000000, options), runEigenVector3Dot, runLinearAlgebraVector3Dot));
        results.push_back(runBenchmarkPair(
          "v3 cross", benchmarkIterations(10000000, options), runEigenVector3Cross, runLinearAlgebraVector3Cross));
        results.push_back(runBenchmarkPair(
          "3x3 multiply", benchmarkIterations(5000000, options), runEigenMatrix3Multiply, runLinearAlgebraMatrix3Multiply));
        results.push_back(runBenchmarkPair("3x3 transpose multiply",
                                           benchmarkIterations(5000000, options),
                                           runEigenMatrix3TransposeMultiply,
                                           runLinearAlgebraMatrix3TransposeMultiply));
        results.push_back(runBenchmarkPair("3x3 vector multiply",
                                           benchmarkIterations(8000000, options),
                                           runEigenMatrix3VectorMultiply,
                                           runLinearAlgebraMatrix3VectorMultiply));
        results.push_back(runBenchmarkPair(
          "3x3 inverse", benchmarkIterations(1000000, options), runEigenMatrix3Inverse, runLinearAlgebraMatrix3Inverse));
        results.push_back(runBenchmarkPair(
          "4x4 inverse", benchmarkIterations(200000, options), runEigenMatrix4Inverse, runLinearAlgebraMatrix4Inverse));
        results.push_back(runBenchmarkPair(
          "6x6 multiply", benchmarkIterations(800000, options), runEigenMatrix6Multiply, runLinearAlgebraMatrix6Multiply));
        results.push_back(runBenchmarkPair("6x6 vector multiply",
                                           benchmarkIterations(2000000, options),
                                           runEigenMatrix6VectorMultiply,
                                           runLinearAlgebraMatrix6VectorMultiply));
        results.push_back(runBenchmarkPair(
          "dynamic 6x6 multiply",
          benchmarkIterations(500000, options),
          [](std::uint64_t iterations) { return runEigenDynamicMultiply(iterations, 6); },
          [](std::uint64_t iterations) { return runLinearAlgebraDynamicMultiply(iterations, 6); }));
        results.push_back(runBenchmarkPair(
          "dynamic 12x12 multiply",
          benchmarkIterations(100000, options),
          [](std::uint64_t iterations) { return runEigenDynamicMultiply(iterations, 12); },
          [](std::uint64_t iterations) { return runLinearAlgebraDynamicMultiply(iterations, 12); }));
        results.push_back(runBenchmarkPair("generic 6x6 inverse",
                                           benchmarkIterations(200, options),
                                           runEigenGeneralMatrix6Inverse,
                                           runLinearAlgebraGeneralMatrix6Inverse));

        std::cout << "Eigen versus linearAlgebra benchmark\n"
                  << "Build and run this benchmark target in Release mode for meaningful timings.\n"
                  << "The ratio column is linearAlgebra time divided by Eigen time.\n";
        if (options.smokeMode) {
            std::cout << "Smoke mode: each operation uses a capped iteration count.\n";
        }
        printResults(results);
        std::cout << "\nchecksum sink: " << benchmarkSink << "\n";
    } catch (const std::exception& error) {
        std::cerr << "benchmark_eigen_linear_algebra: " << error.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
