/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "sphericalHarmonicsGravityModel.h"
#include "simulation/dynamics/_GeneralModuleFiles/gravityEffector.h"

namespace {
// Computes the term (2 - d_l), where d_l is the kronecker delta.
inline double getK(const size_t degree)
{
    return (degree == 0) ? 1.0 : 2.0;
}
}

std::optional<std::string> SphericalHarmonicsGravityModel::initializeParameters()
{
    if (this->cBar.size() == 0 || this->sBar.size() == 0) {
        return "Could not initialize spherical harmonics: the 'C' or 'S' parameters were not "
               "provided.";
    }

    for (size_t i = 0; i <= this->maxDeg + 1; i++) {
        std::vector<double> aRow, n1Row, n2Row;
        aRow.resize(i + 1, 0.0);
        // Diagonal elements of A_bar
        if (i == 0) { aRow[i] = 1.0; }
        else {
            aRow[i] = sqrt(double((2 * i + 1) * getK(i)) / (2 * i * getK(i - 1))) *
                      this->aBar[i - 1][i - 1];
        }
        n1Row.resize(i + 1, 0.0);
        n2Row.resize(i + 1, 0.0);
        for (size_t m = 0; m <= i; m++) {
            if (i >= m + 2) {
                n1Row[m] = sqrt(double((2 * i + 1) * (2 * i - 1)) / ((i - m) * (i + m)));
                n2Row[m] = sqrt(double((i + m - 1) * (2 * i + 1) * (i - m - 1)) /
                                ((i + m) * (i - m) * (2 * i - 3)));
            }
        }
        this->n1.push_back(n1Row);
        this->n2.push_back(n2Row);
        this->aBar.push_back(aRow);
    }

    for (size_t l = 0; l <= this->maxDeg; l++) // up to _maxDegree-1
    {
        std::vector<double> nq1Row, nq2Row;
        nq1Row.resize(l + 1, 0.0);
        nq2Row.resize(l + 1, 0.0);
        for (size_t m = 0; m <= l; m++) {
            if (m < l) { nq1Row[m] = sqrt(double((l - m) * getK(m) * (l + m + 1)) / getK(m + 1)); }
            nq2Row[m] = sqrt(double((l + m + 2) * (l + m + 1) * (2 * l + 1) * getK(m)) /
                             ((2 * l + 3) * getK(m + 1)));
        }
        this->nQuot1.push_back(nq1Row);
        this->nQuot2.push_back(nq2Row);
    }

    return {}; // No error!
}

std::optional<std::string>
SphericalHarmonicsGravityModel::initializeParameters(const GravBodyData& body)
{
    this->radEquator = body.radEquator;
    this->muBody = body.mu;
    return this->initializeParameters();
}

Eigen::Vector3d
SphericalHarmonicsGravityModel::computeField(const Eigen::Vector3d& position_planetFixed) const
{
    return this->computeField(position_planetFixed, this->maxDeg, true);
}

Eigen::Vector3d
SphericalHarmonicsGravityModel::computeField(const Eigen::Vector3d& position_planetFixed,
                                             size_t degree, bool include_zero_degree) const
{
    if (degree > this->maxDeg) {
        auto errorMsg =
            "Requested degree greater than maximum degree in Spherical Harmonics gravity model";
        bskLogger.bskLog(BSK_ERROR, errorMsg);
    }

    double x = position_planetFixed[0];
    double y = position_planetFixed[1];
    double z = position_planetFixed[2];
    double r, s, t, u;
    double order;
    double rho;
    double a1, a2, a3, a4, sum_a1, sum_a2, sum_a3, sum_a4;
    std::vector<double> rE, iM, rhol;

    // Change of variables: direction cosines
    r = sqrt(x * x + y * y + z * z);
    s = x / r;
    t = y / r;
    u = z / r;

    // Future work: allow maximum order different than maximum degree
    order = degree;

    for (size_t l = 1; l <= degree + 1; l++) {
        // Diagonal terms are computed in initializeParameters()
        //  Low diagonal terms
        this->aBar[l][l - 1] = sqrt(double((2 * l) * getK(l - 1)) / getK(l)) * this->aBar[l][l] * u;
    }

    // Lower terms of A_bar
    for (size_t m = 0; m <= order + 1; m++) {
        for (size_t l = m + 2; l <= degree + 1; l++) {
            this->aBar[l][m] =
                u * this->n1[l][m] * this->aBar[l - 1][m] - this->n2[l][m] * this->aBar[l - 2][m];
        }

        // Computation of real and imaginary parts of (2+j*t)^m
        if (m == 0) {
            rE.push_back(1.0);
            iM.push_back(0.0);
        }
        else {
            rE.push_back(s * rE[m - 1] - t * iM[m - 1]);
            iM.push_back(s * iM[m - 1] + t * rE[m - 1]);
        }
    }

    rho = radEquator / r;
    rhol.resize(degree + 2, 0.0);
    rhol[0] = muBody / r;
    rhol[1] = rhol[0] * rho;

    // Degree 0

    // Gravity field and potential of degree l = 0
    // Gravity components
    a1 = 0.0;
    a2 = 0.0;
    a3 = 0.0;
    a4 = 0.0;

    if (include_zero_degree) {
        a4 = -rhol[1] / radEquator; // * this->_Nquot_2[0][0] * this->_A_bar[1][1]; //This is 1, so
                                    // it's not included!
    }

    for (size_t l = 1; l <= degree; l++) // does not include l = maxDegree
    {
        rhol[l + 1] = rho * rhol[l]; // rho_l computed

        sum_a1 = 0.0;
        sum_a2 = 0.0;
        sum_a3 = 0.0;
        sum_a4 = 0.0;

        for (size_t m = 0; m <= l; m++) {
            double D, E, F;
            D = this->cBar[l][m] * rE[m] + this->sBar[l][m] * iM[m];
            if (m == 0) {
                E = 0.0;
                F = 0.0;
            }
            else {
                E = this->cBar[l][m] * rE[m - 1] + this->sBar[l][m] * iM[m - 1];
                F = this->sBar[l][m] * rE[m - 1] - this->cBar[l][m] * iM[m - 1];
            }

            sum_a1 = sum_a1 + m * this->aBar[l][m] * E;
            sum_a2 = sum_a2 + m * this->aBar[l][m] * F;
            if (m < l) { sum_a3 = sum_a3 + this->nQuot1[l][m] * this->aBar[l][m + 1] * D; }
            sum_a4 = sum_a4 + this->nQuot2[l][m] * this->aBar[l + 1][m + 1] * D;
        }

        a1 = a1 + rhol[l + 1] / radEquator * sum_a1;
        a2 = a2 + rhol[l + 1] / radEquator * sum_a2;
        a3 = a3 + rhol[l + 1] / radEquator * sum_a3;
        a4 = a4 - rhol[l + 1] / radEquator * sum_a4;
    }

    return {a1 + s * a4, a2 + t * a4, a3 + u * a4};
}

double SphericalHarmonicsGravityModel::computePotentialEnergy(
    const Eigen::Vector3d& positionWrtPlanet_N) const
{
    return -this->muBody / positionWrtPlanet_N.norm();
}
