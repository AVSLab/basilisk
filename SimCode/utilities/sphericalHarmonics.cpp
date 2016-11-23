/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "sphericalHarmonics.h"

#include <string>
#include <vector>
#include <math.h>
#include "coeffLoader.h"


///---------------------------Constructors-------------------------///

/*!
 @brief Default constructor.
 */
sphericalHarmonics::sphericalHarmonics(void) :
    _maxDegree(0),
    _referenceRadius(0),
    _mu(0),
    _C_bar(nullptr),
    _S_bar(nullptr),
    _A_bar(nullptr),
    _Re(nullptr),
    _Im(nullptr),
    _N1(nullptr),
    _N2(nullptr),
    _Nquot_1(nullptr),
    _Nquot_2(nullptr),
    _coeffLoader(nullptr),
    _errorMessage("")
{
    return;
}

/*!
 @brief Constructor using parameters. Check getLastError() after calling.
 @param[in] loader Object that loads the coefficients from a file into memory. This scheme provides a file-format independent way to load the coefficients from a file. The implementation is hidden in the loader.
 @param[in] filename Route to the file of coefficients.
 @param[in] max_degree Maximum degree to be loaded into memory. If degree n is loaded, the field will have up to degree n-1.
 @param[in] mu Gravitational parameter.
 @param[in] reference_radius Radius of reference with which the coefficients are computed. It must be given along with the coefficients.
 */
sphericalHarmonics::sphericalHarmonics(coeffLoader* loader, const std::string& filename, const unsigned int max_degree, const double mu, const double reference_radius) :
    _maxDegree(max_degree),
    _referenceRadius(reference_radius),
    _mu(mu),
    _C_bar(nullptr),
    _S_bar(nullptr),
    _A_bar(nullptr),
    _Re(nullptr),
    _Im(nullptr),
    _N1(nullptr),
    _N2(nullptr),
    _Nquot_1(nullptr),
    _Nquot_2(nullptr),
    _errorMessage("")
{
    bool ret = true;
    
    this->_coeffLoader = loader;
    
    ret &= this->allocateArray(&_C_bar, _maxDegree);
    ret &= this->allocateArray(&_S_bar, _maxDegree);
    if (ret != true) {
        this->_errorMessage = "ERROR: Memory could not be allocated.";
        return;
    }

    ret = this->_coeffLoader->load(filename, _C_bar, _S_bar, &_maxDegree); // Load the coefficients allocating C_bar and S_bar
    if (ret != true) {
        this->_errorMessage = this->_coeffLoader->getLastErrorMessage();
        return;
    }
    
    ret = this->initialize();
    if (ret != true) {
        this->_errorMessage = "ERROR: Memory could not be allocated.";
        return;
    }
    
    return;
}

/*!
 @ Copy constructor.
 @param hg Object to be copied.
 */
sphericalHarmonics::sphericalHarmonics(const sphericalHarmonics& hg) :
    _maxDegree(hg._maxDegree),
    _referenceRadius(hg._referenceRadius),
    _mu(hg._mu),
    _A_bar(nullptr),
    _Re(nullptr),
    _Im(nullptr),
    _N1(nullptr),
    _N2(nullptr),
    _Nquot_1(nullptr),
    _Nquot_2(nullptr),
    _errorMessage(hg._errorMessage)
{
    bool ret = true;
    
    _coeffLoader = hg._coeffLoader; // Only copies the pointer (same object)
    
    ret &= sphericalHarmonics::allocateArray(&this->_C_bar, this->_maxDegree);
    ret &= sphericalHarmonics::allocateArray(&this->_S_bar, this->_maxDegree);
    ret &= sphericalHarmonics::allocateArray(&this->_A_bar, this->_maxDegree);
    ret &= sphericalHarmonics::allocateArray(&this->_Re, this->_maxDegree);
    ret &= sphericalHarmonics::allocateArray(&this->_Im, this->_maxDegree);
    ret &= sphericalHarmonics::allocateArray(&this->_N1, this->_maxDegree);
    ret &= sphericalHarmonics::allocateArray(&this->_N2, this->_maxDegree);
    ret &= sphericalHarmonics::allocateArray(&this->_Nquot_1, this->_maxDegree);
    ret &= sphericalHarmonics::allocateArray(&this->_Nquot_2, this->_maxDegree);
    if (ret != true) {
        this->_errorMessage = "ERROR: Memory could not be allocated";
        return;
    }
    
    this->copy(hg);
    
    return;
}


///----------------------------------Destructors------------------------------///
sphericalHarmonics::~sphericalHarmonics()
{
    this->deallocate();
}

///----------------------------Overloaded operators--------------------------///
sphericalHarmonics& sphericalHarmonics::operator=(const sphericalHarmonics& hg)
{
    if (&hg == this)
        return *this;
    
    this->_maxDegree = hg._maxDegree;
    this->_referenceRadius = hg._referenceRadius;
    this->_mu = hg._mu;
    
    this->_errorMessage = hg._errorMessage;
    
    this->_coeffLoader = hg._coeffLoader; // Only copies the pointer
    
    this->copy(hg);
    
    return *this;
}

///------------------------------------Getters--------------------------------///

double sphericalHarmonics::getClm(const unsigned int degree, const unsigned int order) const
{
    return this->_C_bar[degree][order];
}

double sphericalHarmonics::getSlm(const unsigned int degree, const unsigned int order) const
{
    return this->_S_bar[degree][order];
}

unsigned int sphericalHarmonics::getMaxDegree() const
{
    return this->_maxDegree;
}

double sphericalHarmonics::getReferenceRadius() const
{
    return this->_referenceRadius;
}

double sphericalHarmonics::getGravitationalParameter() const
{
    return this->_mu;
}

///------------------------------------Setters--------------------------------///
/*
 @brief This setter does not destroy the loader since its life is independent of the spherical harmonics existance. It's an aggregation more than a composition.
 */
void sphericalHarmonics::setCoefficientLoader(coeffLoader* loader)
{
    this->_coeffLoader = loader;
}

///---------------------------------Main Interface----------------------------///
/*!
 @brief Use to compute the field in position pos, given in a body frame.
 @param[in] pos Position in which the field is to be computed.
 @param[in] degree used to compute the field.
 @param[out] acc Vector including the computed field.
 @param[in] include_zero_degree Boolean that determines whether the zero-degree term is included.
 */
void sphericalHarmonics::computeField(const double pos[3], unsigned int degree, double acc[3], bool include_zero_degree) const
{
    double x = pos[0];
    double y = pos[1];
    double z = pos[2];
    double r, s, t, u;
    double order;
    double rho;
    double a1, a2, a3, a4, sum_a1, sum_a2, sum_a3, sum_a4;
    
    // Change of variables: direction cosines
    r = sqrt(x*x + y*y + z*z);
    s = x/r;
    t = y/r;
    u = z/r;
    
    // maximum degree!
    if (degree > this->_maxDegree)
        degree = this->_maxDegree;
    
    order = degree;
    double *rhol = new double[degree+1];
    
    for (unsigned int l = 1; l <= degree; l++)
    {
        //Diagonal terms are computed in initialize()
        // Low diagonal terms
        this->_A_bar[l][l-1] = sqrt(double((2*l)*getK(l-1))/getK(l)) * this->_A_bar[l][l] * u;
#ifdef DEBUG_SPHERICAL_HARM
        printf("A_bar[%i][%i] = %.10f\n", l, l-1,_A_bar[l][l-1]);
#endif
    }
    
    // Lower terms of A_bar
    for (unsigned int m = 0; m <= order; m++)
    {
        for(unsigned int l = m + 2; l <= degree; l++)
        {
            this->_A_bar[l][m] = u * this->_N1[l][m] * this->_A_bar[l-1][m] - this->_N2[l][m] * this->_A_bar[l-2][m];
#ifdef DEBUG_SPHERICAL_HARM
            printf("A_bar[%i][%i] = %.10f\n", l, m,_A_bar[l][m]);
#endif
        }
        
        // Computation of real and imaginary parts of (2+j*t)^m
        if (m == 0)
        {
            this->_Re[m] = 1.0;
            this->_Im[m] = 0.0;
        }
        else
        {
            this->_Re[m] = s * this->_Re[m-1] - t * this->_Im[m-1];
            this->_Im[m] = s * this->_Im[m-1] + t * this->_Re[m-1];
        }
#ifdef DEBUG_SPHERICAL_HARM
        printf("Re[%i] = %.10f\n", m,_Re[m]);
        printf("Im[%i] = %.10f\n", m,_Im[m]);
#endif
        
    }
    
    rho = this->_referenceRadius/r;
    rhol[0] = this->_mu/r;
    rhol[1] = rhol[0]*rho;
#ifdef DEBUG_SPHERICAL_HARM
    printf("rhol[0] = %.10f\n",rhol[0]);
    printf("rhol[1] = %.10f\n",rhol[1]);
#endif
    
    // Degree 0
    
    // Gravity field and potential of degree l = 0
    // Gravity components
    a1 = 0.0;
    a2 = 0.0;
    a3 = 0.0;
    a4 = 0.0;
    
    if (include_zero_degree == true)
        a4 = -rhol[1]/this->_referenceRadius; // * this->_Nquot_2[0][0] * this->_A_bar[1][1]; //This is 1, so it's not included!
    
    for (unsigned int l = 1; l < degree; l++) // does not include l = maxDegree
    {
        rhol[l+1] =  rho * rhol[l]; // rho_l computed
#ifdef DEBUG_SPHERICAL_HARM
        printf("rhol[%i] = %.10f\n", l+1,rhol[l+1]);
#endif
        sum_a1 = 0.0;
        sum_a2 = 0.0;
        sum_a3 = 0.0;
        sum_a4 = 0.0;
        
        for(unsigned int m = 0; m <= l; m++)
        {
            double D, E, F;
            D = this->_C_bar[l][m] * this->_Re[m] + this->_S_bar[l][m] * this->_Im[m];
            if (m == 0)
            {
                E = 0.0;
                F = 0.0;
            }
            else
            {
                E = this->_C_bar[l][m] * this->_Re[m-1] + this->_S_bar[l][m] * this->_Im[m-1];
                F = this->_S_bar[l][m] * this->_Re[m-1] - this->_C_bar[l][m] * this->_Im[m-1];
            }
            
//            if (l < degree)   // Gravity contains up to max_degree-1 harmonics
//            {
            sum_a1 = sum_a1 + m * this->_A_bar[l][m] * E;
            sum_a2 = sum_a2 + m * this->_A_bar[l][m] * F;
            if (m < l)
            {
                sum_a3 = sum_a3 + this->_Nquot_1[l][m] * this->_A_bar[l][m+1] * D;
            }
            sum_a4 = sum_a4 + this->_Nquot_2[l][m] * this->_A_bar[l+1][m+1] * D;
//            }
            
        }
        
//        if (l < degree)   // Gravity contains up to max_degree-1 harmonics
//        {
        a1 = a1 + rhol[l+1]/this->_referenceRadius * sum_a1;
        a2 = a2 + rhol[l+1]/this->_referenceRadius * sum_a2;
        a3 = a3 + rhol[l+1]/this->_referenceRadius * sum_a3;
        a4 = a4 - rhol[l+1]/this->_referenceRadius * sum_a4;
//        }
    }
    
    acc[0] = a1 + s * a4;
    acc[1] = a2 + t * a4;
    acc[2] = a3 + u * a4;
	delete[] rhol;
}

/*!
 @brief Use to compute the field in position pos, given in a body frame (uses Vectors instead of arrays). It's a wrapper of computeField().
 @param[in] pos Position in which the field is to be computed.
 @param[in] degree used to compute the field.
 @param[out] acc Vector including the computed field.
 @param[in] include_zero_degree Boolean that determines whether the zero-degree term is included.
 @return acc
 */
std::vector<double> sphericalHarmonics::getFieldVector(const std::vector<double>& pos, unsigned int degree, std::vector<double>& acc, bool include_zero_degree) const
{
    double pos_array[3];
    double acc_array[3];
    
    pos_array[0] = pos[0];
    pos_array[1] = pos[1];
    pos_array[2] = pos[2];
    
    this->computeField(pos_array, degree, acc_array, include_zero_degree);
    
    acc[0] = acc_array[0];
    acc[1] = acc_array[1];
    acc[2] = acc_array[2];
    
    return acc;
}


/*!
 @brief Use this method to get the last error message.
 @return A string with the message.
 */
std::string sphericalHarmonics::getLastErrorMessage(void)
{
    return this->_errorMessage;
}

#ifdef DEBUG_SPHERICAL_HARM
void sphericalHarmonics::printCoefficients() const
{
    for (unsigned int i = 0; i <= this->_maxDegree; i++)
    {
        for (unsigned int j = 0; j <= i; j++)
        {
            printf("%i %i %e %e\n", i, j, _C_bar[i][j], _S_bar[i][j]);
        }
    }
}
#endif

///----------------------------Private Methods-------------------------------///
/*
 @brief Computes the term (2 - d_l), where d_l is the kronecker delta.
 */
double sphericalHarmonics::getK(const unsigned int degree)
{
    return ((degree == 0) ? 1.0 : 2.0);
}


///-------------------------------Initializers-------------------------------///
/*
 @brief Allocates arrays and computes terms that are position-independent.
 */
bool sphericalHarmonics::initialize()
{
    bool ret = true;
    
    ret &= this->allocateArray(&this->_A_bar, this->_maxDegree);
    ret &= this->allocateArray(&this->_Re, this->_maxDegree);
    ret &= this->allocateArray(&this->_Im, this->_maxDegree);
    ret &= this->allocateArray(&this->_N1, this->_maxDegree);
    ret &= this->allocateArray(&this->_N2, this->_maxDegree);
    ret &= this->allocateArray(&this->_Nquot_1, this->_maxDegree);
    ret &= this->allocateArray(&this->_Nquot_2, this->_maxDegree);
    
    if (ret != true)
    {
        return ret;
    }
    
    for(unsigned int l = 0; l < this->_maxDegree + 1; l++)
    {
        // Diagonal elements of A_bar
        if (l == 0)
            this->_A_bar[l][l] = 1.0;
        else
            this->_A_bar[l][l] = sqrt(double((2*l+1)*getK(l))/(2*l*getK(l-1))) * this->_A_bar[l-1][l-1];
        
#ifdef DEBUG_SPHERICAL_HARM
        printf("A_bar[%i][%i] = %.10f\n", l, l,_A_bar[l][l]);
#endif
        
        for (unsigned int m = 0; m <= l; m++)
        {
            if (l >= m + 2)
            {
                this->_N1[l][m] = sqrt(double((2*l+1)*(2*l-1))/((l-m)*(l+m)));
                this->_N2[l][m] = sqrt(double((l+m-1)*(2*l+1)*(l-m-1))/((l+m)*(l-m)*(2*l-3)));
#ifdef DEBUG_SPHERICAL_HARM
                printf("N1[%i][%i] = %.10f\n", l, m,_N1[l][m]);
                printf("N2[%i][%i] = %.10f\n", l, m,_N2[l][m]);
#endif
            }
        }
    }
    
    for (unsigned int l = 0; l < this->_maxDegree; l++) // up to _maxDegree-1
    {
        for (unsigned int m = 0; m <= l; m++)
        {
            if (m < l)
            {
                this->_Nquot_1[l][m] = sqrt(double((l-m)*getK(m)*(l+m+1))/getK(m+1));
            }
            this->_Nquot_2[l][m] = sqrt(double((l+m+2)*(l+m+1)*(2*l+1)*getK(m))/((2*l+3)*getK(m+1)));
#ifdef DEBUG_SPHERICAL_HARM
            printf("Nquot_1[%i][%i] = %.10f\n", l, m,_Nquot_1[l][m]);
            printf("Nquot_2[%i][%i] = %.10f\n", l, m,_Nquot_2[l][m]);
#endif
        }
    }
    
    return ret;
}

void sphericalHarmonics::deallocate()
{
    this->deallocateArray(this->_C_bar, this->_maxDegree);
    this->deallocateArray(this->_S_bar, this->_maxDegree);
    this->deallocateArray(this->_A_bar, this->_maxDegree);
    this->deallocateArray(this->_Re, this->_maxDegree);
    this->deallocateArray(this->_Im, this->_maxDegree);
    this->deallocateArray(this->_Nquot_1, this->_maxDegree);
    this->deallocateArray(this->_Nquot_2, this->_maxDegree);
}

///-------------------------------Static Methods-----------------------------///
/*
 @brief Copy the status of x into current object.
 @param x Object to be copied.
 */
void sphericalHarmonics::copy(const sphericalHarmonics& x)
{
    this->copyArray(this->_C_bar,x._C_bar,this->_maxDegree);
    this->copyArray(this->_S_bar,x._S_bar,this->_maxDegree);
    this->copyArray(this->_A_bar,x._A_bar,this->_maxDegree);
    this->copyArray(this->_Re,x._Re,this->_maxDegree);
    this->copyArray(this->_Im,x._Im,this->_maxDegree);
    this->copyArray(this->_N1,x._N1,this->_maxDegree);
    this->copyArray(this->_N2,x._N2,this->_maxDegree);
    this->copyArray(this->_Nquot_1,x._Nquot_1,this->_maxDegree);
    this->copyArray(this->_Nquot_2,x._Nquot_2,this->_maxDegree);
    
    return;
}


/*!
 @brief Allocates vectors of size (degree + 1). Pass the direction of the pointer to allocate: &ptr.
 */
bool sphericalHarmonics::allocateArray(double** a, const unsigned int degree)
{
    bool ret = true;
    
    if (a != nullptr)
    {
        *a = new double[degree + 1];
        if (!*a) // Memory error
            ret = false;
        else
            for (unsigned int l = 0; l < degree + 1; l++)
                (*a)[l] = 0.0;
    }
    else
    {
        //ERROR: The method wasn't called passing &ptr.
        ret = false;
        
    }
    return ret;
}

/*!
    @brief Allocates square matrices of size (degree + 1) x (degree + 1).
 */
bool sphericalHarmonics::allocateArray(double*** a, const unsigned int degree)
{
    bool ret = true;
    
    if (a != nullptr)
    {
        *a = new double*[degree + 1];
        if (!*a) // Memory error
            ret = false;
        else{
            for (unsigned int l = 0; l < degree + 1; l++)
            {
                (*a)[l] = new double[degree + 1];
                if (!(*a)[l]) // Memory error
                    ret = false;
                else
                    for (unsigned int m = 0;  m < degree + 1; m++)
                        (*a)[l][m] = 0.0;
            }
        }
    }
    else
    {
        //ERROR: The method wasn't called passing &ptr.
        ret = false;
        
    }
    return ret;
}


/*!
 @brief Deallocates vectors of size (degree + 1).
 */
void sphericalHarmonics::deallocateArray(double* a, const unsigned int degree)
{
    if (a != nullptr)
    {
        delete[] a;
        a = nullptr;
    }
    
    return;
}

/*!
 @brief Deallocates square matrices of size (degree + 1) x (degree + 1).
 */
void sphericalHarmonics::deallocateArray(double** a, const unsigned int degree)
{
    if (a != nullptr)
    {
        for (unsigned int l = 0; l < degree + 1; l++)
        {
            if (a[l] != nullptr)
            {
                delete[] a[l];
                a[l] = nullptr;
            }
        }
        delete[] a;
        a = nullptr;
    }
    
    return;
}

/*!
 @brief Copy one-dimensional array b to a.
 */
void sphericalHarmonics::copyArray(double* a,  double* b,  const unsigned int degree)
{
    for (unsigned int l = 0; l < degree + 1; l++)
        a[l] = b[l];
    
    return;
}

/*!
 @brief Copy two-dimensional array b to a.
 */
void sphericalHarmonics::copyArray(double** a, double** b, const unsigned int degree)
{
    for (unsigned int l = 0; l < degree + 1; l++)
    {
        for (unsigned int m = 0; m < degree + 1;  ++m)
            a[l][m] = b[l][m];
    }
    
    return;
}