/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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
#include "matrix_operations.h"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace std;

MatrixOperations::MatrixOperations() :
vec_vals(NULL),
dim_length(0),
dim_array(NULL),
init(false)
{
    
    return;
    
}

MatrixOperations::MatrixOperations(int nrow, int ncol, double *matrix)
{
    int *temp_dim = new int[2];
    this->dim_length = 2;
    temp_dim[0] = nrow;
    temp_dim[1] = ncol;
    vec_vals = NULL;
    init = false;
    dim_array = NULL;
    this->MatOps_init(dim_length, temp_dim);
    memcpy(this->vec_vals, matrix, nrow*ncol*sizeof(double));
    delete [] temp_dim;
}

MatrixOperations::MatrixOperations(const MatrixOperations &source)
{
    int size, i;
    vec_vals = NULL;
    init = false;
    dim_array = NULL;
    this->MatOps_init(source.dim_length, source.dim_array);
    size = 1;
    for(i=0; i<this->dim_length; i++)
    {
        size = size*this->dim_array[i];
    }
    memcpy(this->vec_vals, source.vec_vals, size*sizeof(double));
}

MatrixOperations::~MatrixOperations(){
    
    this->MatOps_clear();
    return;
    
}

void MatrixOperations::MatOps_clear()
{
    //If the vec vals is not null, then delete it and null it
    if(this->vec_vals)
    {
        delete [] this->vec_vals;
        this->vec_vals = NULL;
    }
    //If the dimension array is not null then delete and null it
    if(this->dim_array)
    {
        delete [] this->dim_array;
        this->dim_array = NULL;
    }
    return;
}

void MatrixOperations::MatOps_ident()
{
    int i;
    if(this->dim_length != 2)
    {
        cout << "Can't set non-2D matrix to identity\n";
        return;
    }
    if(this->dim_array[0] != this->dim_array[1])
    {
        cout << "Can't set identity matrix if columns != rows\n";
        return;
    }
    //Initialize the matrix and then set diagonals to 1
    this->MatOps_init(this->dim_length, this->dim_array);
    for(i=0; i<this->dim_array[0]; i++)
    {
        this->vec_vals[i*dim_array[0]+i] = 1.0;
    }
    return;
}

void MatrixOperations::MatOps_mult(
                                   MatrixOperations mat_A,             // -- Matrix A in the multiplication scheme
                                   MatrixOperations mat_B)             // -- Matrix B in the mult schemei
{
    int i, j, k;
    int dimi, dimAi, dimji;
    int *DimVector;
    
    //creat temporary matrix copies in case output matrix is an operand
    
    if(mat_A.dim_length != 2 || mat_B.dim_length != 2)
    {
        cout << "I need both of the matrices to be 2D to do multiplication!\n";
        return;
    }
    if(mat_A.dim_array[1] != mat_B.dim_array[0])
    {
        cout << "Dimension mismatch between rows and columns in mult.\n";
        return;
    }
    
    //delete and allocate the new array
    this->MatOps_clear();
    this->dim_length = 2;
    this->dim_array = new int[2];
    this->dim_array[0] = mat_A.dim_array[0];
    this->dim_array[1] = mat_B.dim_array[1];
    this->MatOps_init(this->dim_length, this->dim_array);
    
    DimVector = new int[mat_B.dim_array[0]];
    for(i=0; i<mat_B.dim_array[0]; i++)
    {
        DimVector[i] = i*mat_B.dim_array[1];
    }
    
    //Multiply the two matrices together
    for(i=0; i<this->dim_array[0]; i++)
    {
        dimi = i*this->dim_array[1];
        dimAi = i*mat_A.dim_array[1];
        for(j=0; j<this->dim_array[1]; j++)
        {
            dimji = dimi+j;
            for(k=0; k<mat_B.dim_array[0]; k++)
            {
                this->vec_vals[dimji] += mat_A.vec_vals[dimAi+k]
                * mat_B.vec_vals[DimVector[k]+ j];
            }
        }
    }
    //Clear out the temporary matrices and their memory
    delete [] DimVector;
    
    return;
}

void MatrixOperations::MatOps_transpose(
                                        MatrixOperations mat_A)             // -- Matrix A that willo be transposed
{
    int i, j;
    MatrixOperations temp_A;
    int dim_trans[2];
    
    if(mat_A.dim_length != 2)
    {
        cout << "I can only transpose matrices with two dimensions!\n";
        return;
    }
    //set a temporary matrix in case you are transposing yourself
    temp_A = mat_A;
    dim_trans[0] = temp_A.dim_array[1];
    dim_trans[1] = temp_A.dim_array[0];
    this->MatOps_init(temp_A.dim_length, dim_trans);
    
    //Transpose the matrix values
    for(i=0; i<dim_trans[0]; i++)
    {
        for(j=0; j<dim_trans[1]; j++)
        {
            this->vec_vals[i*dim_trans[1] + j] = temp_A.vec_vals[j*dim_trans[0]+i];
        }
    }
    temp_A.MatOps_clear();
    
}

void MatrixOperations::operator =(const MatrixOperations &source)
{
    int i;
    int size;
    if(this == &source)
    {
        cout << "Can't set a matrix equal to itself!\n";
        return;
    }
    
    //The equals operator is a nice feature
    this->MatOps_clear();
    this->MatOps_init(source.dim_length, source.dim_array);
    size = 1;
    for(i=0; i<this->dim_length; i++)
    {
        size = size*this->dim_array[i];
    }
    memcpy(this->vec_vals, source.vec_vals, size*sizeof(double));
    return;
}

void MatrixOperations::MatOps_add(
                                  MatrixOperations A,             // First operand in additiooon
                                  MatrixOperations B)             // Second operand in addition
{
    int i;
    int size;
    MatrixOperations temp_A, temp_B;
    if(A.dim_length != B.dim_length)
    {
        cout << "Matrices do not even have the same number of dimensions!\n";
        return;
    }
    size = 1;
    for(i=0; i<A.dim_length; i++)
    {
        if(A.dim_array[i] != B.dim_array[i])
        {
            cout << "Matrices dimension arrays do not agree.\n";
            return;
        }
        size = size*A.dim_array[i];
    }
    //Create temp matrices in case output is an operand
    temp_A = A;
    temp_B = B;
    this->MatOps_init(temp_A.dim_length, temp_A.dim_array);
    for(i=0; i<size; i++)
    {
        this->vec_vals[i] = temp_A.vec_vals[i] + temp_B.vec_vals[i];
    }
    
    temp_A.MatOps_clear();
    temp_B.MatOps_clear();
    
    return;
    
}

void MatrixOperations::MatOps_init( // No return for this function
                                   int  num_dim,               // Number of dimensions for the matrix
                                   int *dim_vector)            // dimenstion list for the matrix
{
    int i;
    int size;
    int local_dim;
    int *local_array;
    
    size = 0;
    
    if(num_dim < 1 || !dim_vector)
    {
        cout << "\n In ";
        cout << __FILE__;
        cout << "\nTrying to init matrix with zero dimensions.  Doing nothing.\n";
        return;
    }
    local_dim = num_dim;
    local_array = new int[num_dim];
    size = 1;
    for(i=0; i<num_dim; i++)
    {
        size = size*dim_vector[i];
        local_array[i] = dim_vector[i];
    }
    //Clear out any previously allocated memory and reset
    this->MatOps_clear();
    this->dim_array = new int[local_dim];
    memcpy(this->dim_array, local_array, local_dim*sizeof(int));
    this->vec_vals = new double[size];
    memset((char *) this->vec_vals, '\0', size*sizeof(double));
    this->dim_length = local_dim;
    this->init = true;
    delete [] local_array;
    
    return;
}

void MatrixOperations::MatOps_init(
                                   int nrow,
                                   int ncol)
{
    int dim_vec[2] = {nrow, ncol};
    this->MatOps_init(2, dim_vec);
    return;
}

void MatrixOperations::MatOps_crossprod(
                                        MatrixOperations mat_A,             // -- Matrix A in the multiplication scheme
                                        MatrixOperations mat_B)            // -- Matrix B in the mult schemei
{
    int A_length;
    int B_length;
    int i;
    MatrixOperations temp_A;
    MatrixOperations temp_B;
    if(mat_A.dim_length >2 || mat_B.dim_length > 2)
    {
        cout << "Only vectors can be crossed (3x1)!" << endl;
        return;
    }
    A_length =1;
    B_length =1;
    for(i=0; i<mat_A.dim_length; i++)
    {
        A_length *= mat_A.dim_array[i];
    }
    for(i=0; i<mat_B.dim_length; i++)
    {
        B_length *= mat_B.dim_array[i];
    }
    if(A_length != 3 || B_length != 3)
    {
        cout << "You can only do the cross-product of three-vectors." <<endl;
        return;
    }
    temp_A = mat_A;
    temp_B = mat_B;
    this->MatOps_init(temp_A.dim_length, temp_A.dim_array);
    this->vec_vals[0] = temp_A.vec_vals[1]*temp_B.vec_vals[2] -
    temp_A.vec_vals[2]*temp_B.vec_vals[1];
    this->vec_vals[1] = -(temp_A.vec_vals[0]*temp_B.vec_vals[2] -
                          temp_A.vec_vals[2]*temp_B.vec_vals[0]);
    this->vec_vals[2] = temp_A.vec_vals[0]*temp_B.vec_vals[1] -
    temp_A.vec_vals[1]*temp_B.vec_vals[0];
    
    temp_A.MatOps_clear();
    temp_B.MatOps_clear();
    
    return;
}

double MatrixOperations::MatOps_twonorm_square()
{
    int i;
    int num_indices;
    double norm_value;
    
    num_indices = 1;
    norm_value = 0.0;
    for(i=0; i<this->dim_length; i++)
    {
        num_indices *= this->dim_array[i];
    }
    
    for(i=0; i<num_indices; i++)
    {
        norm_value = norm_value + this->vec_vals[i]*this->vec_vals[i];
    }
    
    return(norm_value);
}

double MatrixOperations::MatOps_twonorm()
{
    double norm_value;
    
    norm_value = MatOps_twonorm_square();
    norm_value = sqrt(norm_value);
    
    return(norm_value);
}

double MatrixOperations::MatOps_onenorm()
{
    double norm_value, column_sum;
    int i, j;
    
    if(this->dim_length != 2)
    {
        cout << "I can only compute this norm for 2D matrices!\n";
        return(0);
    }
    
    norm_value = 0.0;
    for(i=0; i<this->dim_array[1]; i++)
    {
        column_sum = 0.0;
        for(j=0; j<dim_array[0]; j++)
        {
            column_sum += fabs(this->vec_vals[j*this->dim_array[1] + i]);
        }
        if(column_sum > norm_value)
        {
            norm_value = column_sum;
        }
    }
    return(norm_value);
}

void MatrixOperations::MatOps_scale(
                                    double scal_mult)
{
    int i, num_indices;
    num_indices = 1;
    for(i=0; i<this->dim_length; i++)
    {
        num_indices *= this->dim_array[i];
    }
    for(i=0; i<num_indices; i++)
    {
        this->vec_vals[i] = this->vec_vals[i]*scal_mult;
    }
    return;
}

double MatrixOperations::MatOps_dotprod(
                                        MatrixOperations mat_B)             // -- Matrix A in the multiplicatio
{
    int i;
    int size;
    double dot_prod;
    if(this->dim_length != mat_B.dim_length)
    {
        cout << "Matrices do not even have the same number of dimensions!\n";
        return(0);
    }
    size = 1;
    for(i=0; i<this->dim_length; i++)
    {
        if(this->dim_array[i] != this->dim_array[i])
        {
            cout << "Matrices dimension arrays do not agree.\n";
            return(0);
        }
        size = size*this->dim_array[i];
    }
    dot_prod = 0.0;
    for(i=0; i<size; i++)
    {
        dot_prod += this->vec_vals[i]*mat_B.vec_vals[i];
    }
    return(dot_prod);
}

void MatrixOperations::MatOps_Inv3x3(
                                     MatrixOperations mat_A)             // -- Matrix A in the multiplicatio
{
    double determinate;
    MatrixOperations temp_A;
    if(mat_A.dim_length >2 || mat_A.dim_length < 2)
    {
        cout << "Only a 2D matrix can be inverted!" << endl;
        return;
    }
    if(mat_A.dim_array[0] != 3 || mat_A.dim_array[1] != 3)
    {
        cout << "Only 3x3 matrix can be inverted using this function!"<<endl;
        return;
    }
    temp_A = mat_A;
    this->MatOps_init(temp_A.dim_length, temp_A.dim_array);
    this->vec_vals[0] = temp_A.vec_vals[4]*temp_A.vec_vals[8] -
    temp_A.vec_vals[7]*temp_A.vec_vals[5];
    this->vec_vals[1] = temp_A.vec_vals[7]*temp_A.vec_vals[2] -
    temp_A.vec_vals[1]*temp_A.vec_vals[8];
    this->vec_vals[2] = temp_A.vec_vals[1]*temp_A.vec_vals[5] -
    temp_A.vec_vals[4]*temp_A.vec_vals[2];
    determinate = temp_A.vec_vals[0]*this->vec_vals[0] +
    temp_A.vec_vals[3]*this->vec_vals[1] +
    temp_A.vec_vals[6]*this->vec_vals[2];
    if(determinate == 0.0)
    {
        cout << "Your matrix for inversion was not full-rank.  I quit"<<endl;
    }
    this->vec_vals[0] /= determinate;
    this->vec_vals[1] /= determinate;
    this->vec_vals[2] /= determinate;
    this->vec_vals[3] = (temp_A.vec_vals[6]*temp_A.vec_vals[5] -
                         temp_A.vec_vals[3]*temp_A.vec_vals[8])/determinate;
    this->vec_vals[4] = (temp_A.vec_vals[0]*temp_A.vec_vals[8] -
                         temp_A.vec_vals[6]*temp_A.vec_vals[2])/determinate;
    this->vec_vals[5] = (temp_A.vec_vals[3]*temp_A.vec_vals[2] -
                         temp_A.vec_vals[0]*temp_A.vec_vals[5])/determinate;
    this->vec_vals[6] = (temp_A.vec_vals[3]*temp_A.vec_vals[7] -
                         temp_A.vec_vals[6]*temp_A.vec_vals[4])/determinate;
    this->vec_vals[7] = (temp_A.vec_vals[6]*temp_A.vec_vals[1] -
                         temp_A.vec_vals[0]*temp_A.vec_vals[7])/determinate;
    this->vec_vals[8] = (temp_A.vec_vals[0]*temp_A.vec_vals[4] -
                         temp_A.vec_vals[3]*temp_A.vec_vals[1])/determinate;
    
    return;
}
void MatrixOperations::MatOps_Quat2DCM(
                                       MatrixOperations Q_in)
{
    MatrixOperations temp_A;
    if(Q_in.dim_length >2 || Q_in.dim_length < 2)
    {
        cout << "Only a 2D matrix can be inverted!" << endl;
        return;
    }
    if(Q_in.dim_array[0] != 4 || Q_in.dim_array[1] != 1)
    {
        cout << "Only a 4x1 vector (Quaternion) can be inverted"<<endl;
        return;
    }
    temp_A.MatOps_QuatTrans(Q_in); //Kind of cheating
    this->MatOps_init(3,3);
    this->vec_vals[0*3+0] = 1. - 2. * (temp_A.vec_vals[2] * temp_A.vec_vals[2]
                                       + temp_A.vec_vals[3] * temp_A.vec_vals[3]);
    this->vec_vals[1*3+1] = 1. - 2. * (temp_A.vec_vals[3] * temp_A.vec_vals[3]
                                       + temp_A.vec_vals[1] * temp_A.vec_vals[1]);
    this->vec_vals[2*3+2] = 1. - 2. * (temp_A.vec_vals[1] * temp_A.vec_vals[1]
                                       + temp_A.vec_vals[2] * temp_A.vec_vals[2]);
    this->vec_vals[1*3+0] = 2. * (temp_A.vec_vals[1] * temp_A.vec_vals[2]
                                  + temp_A.vec_vals[0] * temp_A.vec_vals[3]);
    this->vec_vals[0*3+1] = 2. * (temp_A.vec_vals[1] * temp_A.vec_vals[2]
                                  - temp_A.vec_vals[0] * temp_A.vec_vals[3]);
    this->vec_vals[2*3+0] = 2. * (temp_A.vec_vals[1] * temp_A.vec_vals[3]
                                  - temp_A.vec_vals[0] * temp_A.vec_vals[2]);
    this->vec_vals[0*3+2] = 2. * (temp_A.vec_vals[1] * temp_A.vec_vals[3]
                                  + temp_A.vec_vals[0] * temp_A.vec_vals[2]);
    this->vec_vals[2*3+1] = 2. * (temp_A.vec_vals[2] * temp_A.vec_vals[3]
                                  + temp_A.vec_vals[0] * temp_A.vec_vals[1]);
    this->vec_vals[1*3+2] = 2. * (temp_A.vec_vals[2] * temp_A.vec_vals[3]
                                  - temp_A.vec_vals[0] * temp_A.vec_vals[1]);
    return;
}
void MatrixOperations::MatOps_DCM2Quat(
                                       MatrixOperations T_in)             // -- Quat to chng (scal, vect)
{
    double qv[3];
    double T, temp, q0;
    int i, j, k, k1, l, n, jki;
    MatrixOperations temp_A;
    if(T_in.dim_length >2 || T_in.dim_length < 2)
    {
        cout << "Only a 2D matrix can be inverted!" << endl;
        return;
    }
    if(T_in.dim_array[0] != 3 || T_in.dim_array[1] != 3)
    {
        cout << "Only a 3x3 Tmat can be converted"<<endl;
        return;
    }
    temp_A = T_in;
    this->MatOps_init(4,1);
    
    i = 0;
    T = temp_A.vec_vals[0*3+0] + temp_A.vec_vals[1*3+1] + temp_A.vec_vals[2*3+2];
    q0 = T;
    
    for (k = 1; k < 4; k++) {
        k1 = k - 1;
        if (temp_A.vec_vals[k1*3+k1] > q0) {
            i = k;
            q0 = temp_A.vec_vals[k1*3+k1];
        }
    }
    
    temp = 1.0 + 2.0 * q0 - T;
    T = sqrt(temp);
    for (n = 1; n < 4; n++) {
        k = (n % 3) + 1;
        j = 6 - n - k;
        l = i * (n - i);
        if (l == 0) {
            qv[n - 1] = (temp_A.vec_vals[(j-1)*3+k-1] -
                         temp_A.vec_vals[(k-1)*3+j-1]) / T;
            q0 = qv[n - 1];
        } else {
            jki = j + k - i;
            qv[jki - 1] = (temp_A.vec_vals[(j-1)*3+k-1] +
                           temp_A.vec_vals[(k-1)*3+j-1]) / T;
        }
    }
    
    if (i == 0)
        q0 = T;
    else
        qv[i - 1] = T;
    
    T = copysign(0.5, q0);
    this->vec_vals[0] = T * q0;
    
    //Minus sign is a hack to right-ize the quaternion
    this->vec_vals[1] = -T * qv[0];
    this->vec_vals[2] = -T * qv[1];
    this->vec_vals[3] = -T * qv[2];
}
void MatrixOperations::MatOps_QuatMult(
                                       MatrixOperations Q_A,               // -- First quat to multiply
                                       MatrixOperations Q_B)
{
    MatrixOperations ATemp, BTemp;
    if((Q_A.dim_length >2 || Q_A.dim_length < 2) ||
       (Q_B.dim_length >2 || Q_B.dim_length < 2))
    {
        cout << "Only a 4x1 quat can be multiplied.  Must be 2D!" << endl;
        return;
    }
    if((Q_A.dim_array[0] != 4 || Q_A.dim_array[1] != 1) ||
       (Q_B.dim_array[0] != 4 || Q_B.dim_array[1] != 1))
    {
        cout << "Only a 4x1 quat can be multiplied.  Must be 4x1!" << endl;
        return;
    }
    BTemp = Q_B;
    ATemp = Q_A;
    this->MatOps_init(4,1);
    this->vec_vals[0] = BTemp.vec_vals[0]*ATemp.vec_vals[0] -
    BTemp.vec_vals[1]*ATemp.vec_vals[1] -
    BTemp.vec_vals[2]*ATemp.vec_vals[2] -
    BTemp.vec_vals[3]*ATemp.vec_vals[3];
    this->vec_vals[1] = BTemp.vec_vals[1]*ATemp.vec_vals[0] +
    BTemp.vec_vals[0]*ATemp.vec_vals[1] -
    BTemp.vec_vals[3]*ATemp.vec_vals[2] +
    BTemp.vec_vals[2]*ATemp.vec_vals[3];
    this->vec_vals[2] = BTemp.vec_vals[2]*ATemp.vec_vals[0] +
    BTemp.vec_vals[3]*ATemp.vec_vals[1] +
    BTemp.vec_vals[0]*ATemp.vec_vals[2] -
    BTemp.vec_vals[1]*ATemp.vec_vals[3];
    this->vec_vals[3] = BTemp.vec_vals[3]*ATemp.vec_vals[0] -
    BTemp.vec_vals[2]*ATemp.vec_vals[1] +
    BTemp.vec_vals[1]*ATemp.vec_vals[2] +
    BTemp.vec_vals[0]*ATemp.vec_vals[3];
    return;
}

void MatrixOperations::MatOps_VecSet(
                                     double *VecData)
{
    int i, size;
    if(!this->vec_vals)
    {
        cout << "It kind of looks like you didn't initialize this matrix";
        cout << endl;
        return;
    }
    size = 1;
    for(i=0; i<this->dim_length; i++)
    {
        size = size*this->dim_array[i];
    }
    memcpy(this->vec_vals, VecData, size*sizeof(double));
}
void MatrixOperations::MatOps_QuatTrans(
                                        MatrixOperations Q_A)              // -- Quaternion to transpose
{
    int i;
    if(Q_A.dim_length >2 || Q_A.dim_length < 2)
    {
        cout << "Only a 2D matrix can be inverted!" << endl;
        return;
    }
    if(Q_A.dim_array[0] != 4 || Q_A.dim_array[1] != 1)
    {
        cout << "Only a 4x1 vector (Quaternion) can be inverted"<<endl;
        return;
    }
    this->MatOps_init(4,1);
    memcpy(this->vec_vals, Q_A.vec_vals, 4*sizeof(double));
    this->MatOps_scale(-1.0);
    this->vec_vals[0] *= -1.0;
}
void  MatrixOperations::MatOps_QRDecomp(
                                        MatrixOperations &Qout,
                                        MatrixOperations &Rout)
{
    MatrixOperations TMat;
    MatrixOperations Uvec, Utrans, T1;
    int cols, mindim;
    double Asum, sigma, beta, gamma, Umag;
    int i, j, k;
    if(this->dim_length != 2)
    {
        cout << "Hey, to do a QR decomp you need a 2D matrix."<<endl;
        return;
    }
    Rout = *this;
    Qout.MatOps_init(this->dim_array[0], this->dim_array[0]);
    cols = this->dim_array[0] <= this->dim_array[1] ? this->dim_array[0] - 1:
    this->dim_array[1];
    TMat.MatOps_init(this->dim_array[0], this->dim_array[0]);
    TMat.MatOps_ident();
    for(i=0; i<cols; i++)
    {
        Asum = 0.0;
        for(j=i; j<this->dim_array[0]; j++)
        {
            Asum += Rout.vec_vals[j*this->dim_array[1] + i] *
            Rout.vec_vals[j*this->dim_array[1] + i];
        }
        sigma = copysign(sqrt(Asum), Rout.vec_vals[this->dim_array[1]*i+i]);
        Uvec.MatOps_init(this->dim_array[0], 1);
        Uvec.vec_vals[i] = Rout.vec_vals[this->dim_array[1]*i+i] + sigma;
        Rout.vec_vals[this->dim_array[1]*i+i] = -sigma;
        for(j=i+1; j<this->dim_array[0]; j++)
        {
            Uvec.vec_vals[j] = Rout.vec_vals[j*this->dim_array[1] + i];
        }
        beta = 1.0/(sigma*Uvec.vec_vals[i]);
        Umag = Uvec.MatOps_twonorm();
        Utrans.MatOps_transpose(Uvec);
        T1.MatOps_mult(Uvec, Utrans);
        T1.MatOps_scale(-2.0/Umag/Umag);
        mindim = T1.dim_array[0] < T1.dim_array[1] ?  T1.dim_array[0] :
        T1.dim_array[1];
        for(j=0; j<mindim; j++)
        {
            T1.vec_vals[T1.dim_array[1]*j+j] += 1.0;
        }
        TMat.MatOps_mult(T1, TMat);
        for(j=i+1; j<this->dim_array[1]; j++)
        {
            Asum = 0.0;
            for(k=i; k<this->dim_array[0]; k++)
            {
                Asum += Uvec.vec_vals[k]*Rout.vec_vals[k*this->dim_array[1]+j];
            }
            gamma = beta*Asum;
            for(k=i; k<this->dim_array[0]; k++)
            {
                Rout.vec_vals[this->dim_array[1]*k+j] -= gamma*Uvec.vec_vals[k];
            }
        }
        for(j=i+1; j<this->dim_array[0]; j++)
        {
            Rout.vec_vals[j*this->dim_array[1] +i] = 0.0;
        }
    }
    Qout.MatOps_transpose(TMat);
    return;
}

void  MatrixOperations::MatOps_QRD_JustR(
                                         MatrixOperations MatIn)
{
    int i, j, k, dimj, dimi;
    double sum;
    MatrixOperations Qmat;
    
    if(MatIn.dim_length != 2)
    {
        cout << "Hey, to do a QR decomp you need a 2D matrix."<<endl;
        return;
    }
    this->MatOps_init(MatIn.dim_array[1], MatIn.dim_array[1]);
    Qmat.MatOps_init(MatIn.dim_array[0], MatIn.dim_array[1]);
    
    for(i=0; i<MatIn.dim_array[1]; i++)
    {
        dimi = i*this->dim_array[1] + i;
        for(j=0; j<MatIn.dim_array[0]; j++)
        {
            dimj = MatIn.dim_array[1]*j;
            this->vec_vals[dimi] += MatIn.vec_vals[dimj+i]*MatIn.vec_vals[dimj+i];
        }
        this->vec_vals[dimi] = sqrt(this->vec_vals[dimi]);
        for(j=0; j<MatIn.dim_array[0]; j++)
        {
            Qmat.vec_vals[j*Qmat.dim_array[1] + i] =
            MatIn.vec_vals[j*MatIn.dim_array[1] +i]/this->vec_vals[dimi];
        }
        
        for(j=i+1; j<MatIn.dim_array[1]; j++)
        {
            sum = 0;
            dimj = Qmat.dim_array[1]*j;
            for(k=0; k<MatIn.dim_array[0]; k++)
            {
                sum += MatIn.vec_vals[k*MatIn.dim_array[1]+j]*
                Qmat.vec_vals[k*MatIn.dim_array[1]+i];
            }
            this->vec_vals[i*this->dim_array[1]+j] = sum;
            for(k=0; k<MatIn.dim_array[0]; k++)
            {
                MatIn.vec_vals[k*MatIn.dim_array[1]+j] -=
                sum*Qmat.vec_vals[k*MatIn.dim_array[1]+i];
            }
        }
    }
    
    return;
}

void MatrixOperations::MatOps_CholDecomp(
                                         MatrixOperations MatIn)
{
    double sigma;
    int i, j, k;
    if(MatIn.dim_length != 2)
    {
        cout << "Only 2D matrices can be cholesky-ed."<<endl;
        return;
    }
    if(MatIn.dim_array[0] != MatIn.dim_array[1])
    {
        cout << "Only square matrices can be cholesky-ed"<<endl;
    }
    this->MatOps_init(MatIn.dim_length, MatIn.dim_array);
    for(i=0; i<MatIn.dim_array[0]; i++)
    {
        for(j=0; j<=i; j++)
        {
            sigma = MatIn.vec_vals[MatIn.dim_array[0]*i+j];
            for(k=0; k<=(j-1); k++)
            {
                sigma -= this->vec_vals[this->dim_array[0]*i+k] *
                this->vec_vals[this->dim_array[0]*j+k];
            }
            if(i==j)
            {
                this->vec_vals[this->dim_array[0]*i+j] = sqrt(sigma);
            }
            else
            {
                this->vec_vals[this->dim_array[0]*i+j] = sigma/
                (this->vec_vals[this->dim_array[0]*j+j]);
            }
        }
    }
    
    return;
}

void MatrixOperations::MatOps_InvLMat(
                                      MatrixOperations MatIn)
{
    int i, j, k, mat_dim;
    if(MatIn.dim_length != 2)
    {
        cout << "Only 2D matrices can be low-inverted."<<endl;
        return;
    }
    if(MatIn.dim_array[0] != MatIn.dim_array[1])
    {
        cout << "Only square matrices can be low-inverted"<<endl;
    }
    this->MatOps_init(MatIn.dim_array[0], MatIn.dim_array[1]);
    mat_dim = this->dim_array[0];
    for(i=mat_dim-1; i>=0; i--)
    {
        this->vec_vals[mat_dim*i+i] = 1.0/MatIn.vec_vals[i*mat_dim+i];
        for(j=mat_dim-1; j>=i+1; j--)
        {
            this->vec_vals[mat_dim*j+i] = 0.0;
            for(k=i+1; k<=j; k++)
            {
                this->vec_vals[j*mat_dim+i] -= MatIn.vec_vals[mat_dim*k+i]*
                this->vec_vals[j*mat_dim+k];
            }
            this->vec_vals[j*mat_dim+i] *= this->vec_vals[mat_dim*i+i];
        }
    }
    
    
    return;
}

void MatrixOperations::MatOps_InvUMat(
                                      MatrixOperations MatIn)
{
    int i, j, k, mat_dim;
    if(MatIn.dim_length != 2)
    {
        cout << "Only 2D matrices can be upper-inverted."<<endl;
        return;
    }
    if(MatIn.dim_array[0] != MatIn.dim_array[1])
    {
        cout << "Only square matrices can be upper-inverted"<<endl;
    }
    this->MatOps_init(MatIn.dim_array[0], MatIn.dim_array[1]);
    mat_dim = this->dim_array[0];
    for(i=mat_dim-1; i>=0; i--)
    {
        this->vec_vals[mat_dim*i+i] = 1.0/MatIn.vec_vals[i*mat_dim+i];
        for(j=mat_dim-1; j>=i+1; j--)
        {
            this->vec_vals[mat_dim*i+j] = 0.0;
            for(k=i+1; k<=j; k++)
            {
                this->vec_vals[i*mat_dim+j] -= MatIn.vec_vals[mat_dim*i+k]*
                this->vec_vals[k*mat_dim+j];
            }
            this->vec_vals[i*mat_dim+j] *= this->vec_vals[mat_dim*i+i];
        }
    }
    
    
    return;
}

void MatrixOperations::MatOps_ShadowMRP(
                                        MatrixOperations MRPIn)
{
    int i;
    double SigSquare;
    MatrixOperations MRPLocal;
    if(MRPIn.dim_length >2 || MRPIn.dim_length < 2)
    {
        cout << "Only a 2D vector can be MRP-ed!" << endl;
        return;
    }
    if(MRPIn.dim_array[0] != 3 || MRPIn.dim_array[1] != 1)
    {
        cout << "Only a 3x1 vector (MRP) can be shadowed"<<endl;
        return;
    }
    MRPLocal = MRPIn;
    SigSquare = MRPLocal.MatOps_twonorm_square();
    this->MatOps_init(3,1);
    for(i=0; i<3; i++)
    {
        this->vec_vals[i] = -MRPLocal.vec_vals[i]/SigSquare;
    }
    return;
}

void MatrixOperations::MatOps_MRP2Quat(
                                       MatrixOperations MRPIn)
{
    int i;
    double SigSquare;
    MatrixOperations MRPLocal;
    if(MRPIn.dim_length >2 || MRPIn.dim_length < 2)
    {
        cout << "Only a 2D vector can be MRP-ed!" << endl;
        return;
    }
    if(MRPIn.dim_array[0] != 3 || MRPIn.dim_array[1] != 1)
    {
        cout << "Only a 3x1 vector (MRP) can be quated"<<endl;
        return;
    }
    MRPLocal = MRPIn;
    SigSquare = MRPLocal.MatOps_twonorm_square();
    this->MatOps_init(4,1);
    this->vec_vals[0] = (1.0-SigSquare)/(1.0+SigSquare);
    for(i=0; i<3; i++)
    {
        this->vec_vals[i+1] = 2.0*MRPLocal.vec_vals[i]/(1+SigSquare);
    }
    this->MatOps_scale(1.0/this->MatOps_twonorm());
    return;
}
void MatrixOperations::MatOps_Quat2MRP(MatrixOperations Q_A)
{
    int i;
    double scalefactor;
    MatrixOperations QALocal;
    if(Q_A.dim_length >2 || Q_A.dim_length < 2)
    {
        cout << "Only a 2D matrix can be inverted!" << endl;
        return;
    }
    if(Q_A.dim_array[0] != 4 || Q_A.dim_array[1] != 1)
    {
        cout << "Only a 4x1 vector (Quaternion) can be inverted"<<endl;
        return;
    }
    QALocal = Q_A;
    this->MatOps_init(3,1);
    scalefactor = 1.0+QALocal.vec_vals[0];
    QALocal.MatOps_scale(1.0/scalefactor);
    this->MatOps_VecSet(&QALocal.vec_vals[1]);
}

void MatrixOperations::MatOps_EulerAng2DCM(
                                           MatOpsEuler EulerAngles,
                                           MatEulerSeq sequence)
{
    double sr;              // SINE OF PITCH
    double cr;              // COSINE OF PITCH
    double sp;              // SINE OF ROLL
    double cp;              // COSINE OF ROLL
    double sy;              // SINE OF YAW
    double cy;              // COSINE OF YAW
    
    MatrixOperations RMat, PMat, YMat;    
    
    sr = sin(EulerAngles.Roll);
    cr = cos(EulerAngles.Roll);
    sp = sin(EulerAngles.Pitch);
    cp = cos(EulerAngles.Pitch);
    sy = sin(EulerAngles.Yaw);
    cy = cos(EulerAngles.Yaw);
    this->MatOps_init(3,3);
    RMat.MatOps_init(3,3);
    PMat.MatOps_init(3,3);
    YMat.MatOps_init(3,3);
    
    RMat.vec_vals[0] = 1.0;
    RMat.vec_vals[4] = cr;
    RMat.vec_vals[5] = sr;
    RMat.vec_vals[7] = -sr;
    RMat.vec_vals[8] = cr;
    
    PMat.vec_vals[4] = 1.0;
    PMat.vec_vals[0] = cp;
    PMat.vec_vals[2] = -sp;
    PMat.vec_vals[6] = sp;
    PMat.vec_vals[8] = cp;
    
    YMat.vec_vals[8] = 1.0;
    YMat.vec_vals[0] = cr;
    YMat.vec_vals[1] = sr;
    YMat.vec_vals[3] = -sr;
    YMat.vec_vals[4] = cr;
    
    switch(sequence)
    {
        case MatOps_RPY:
            *this = RMat;
            this->MatOps_mult(PMat, *this);
            this->MatOps_mult(YMat, *this);
            break;
        default:
            this->MatOps_ident();
            std::cout << "I don't know how to handle that sequence, have identity.";
            std::cout << std::endl;
            break;
    }
    
    return;
}
void MatrixOperations::MatOps_EnforceMinimum(double MinAllow)
{
    int i;
    int NumIndices;
    NumIndices = 1;
    for(i=0; i<dim_length; i++)
    {
        NumIndices*=this->dim_array[i];
    }
    for(i=0; i<NumIndices; i++)
    {
        if(this->vec_vals[i] < MinAllow)
        {
            this->vec_vals[i] = MinAllow;
        }
    }
}
void MatrixOperations::MatOps_EnforceMaximum(double MaxAllow)
{
    int i;
    int NumIndices;
    NumIndices = 1;
    for(i=0; i<dim_length; i++)
    {
        NumIndices*=this->dim_array[i];
    }
    for(i=0; i<NumIndices; i++)
    {
        if(this->vec_vals[i] > MaxAllow)
        {
            this->vec_vals[i] = MaxAllow;
        }
    }
}

void MatrixOperations::MatOps_SortMatrixDescending(MatrixOperations UnsortedMat)
{
    int i, j;
    int NumIndices;
    bool SortUpdated;
    double NumStorage;
    
    *this = UnsortedMat;
    NumIndices = 1;
    for(i=0; i<dim_length; i++)
    {
        NumIndices*=this->dim_array[i];
    }
    SortUpdated = true;
    for(i=1; i<=NumIndices && SortUpdated; i++)
    {
        SortUpdated = false;
        for(j=0; j<NumIndices-1; j++)
        {
            if(this->vec_vals[j+1] > this->vec_vals[j])
            {
                NumStorage = this->vec_vals[j];
                this->vec_vals[j] = this->vec_vals[j+1];
                this->vec_vals[j+1] = NumStorage;
                SortUpdated = true;
            }
        }
    }
    return;
}
