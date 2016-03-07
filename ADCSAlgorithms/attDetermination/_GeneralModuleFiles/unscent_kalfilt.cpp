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
#include <math.h>
#include <cstring>
#include "unscent_kalfilt.h"

//******************************************************************************
// constructor
UnscentKalFilt::UnscentKalFilt()
{
    
    //printf("***UKF constructor***\n");a
    return;
}

//******************************************************************************
// destructor
UnscentKalFilt::~UnscentKalFilt(){
    return;
}

//******************************************************************************
void UnscentKalFilt::UKFInit()
{
    int i;
    this->Wm.MatOps_init(this->CountHalfSPs*2+1, 1);
    this->Wc.MatOps_init(this->CountHalfSPs*2+1, 1);
    this->Sbar.MatOps_init(this->NumStates, this->NumStates);
    this->SP = new MatrixOperations[this->CountHalfSPs*2+1];
    for(i=0; i<this->CountHalfSPs*2+1; i++)
    {
        this->SP[i].MatOps_init(this->NumStates, 1);
    }
    this->SQnoise.MatOps_init(this->NumStates, this->NumStates);
    this->Wm.vec_vals[0] = this->lambdaVal/(this->NumStates+this->lambdaVal);
    this->Wc.vec_vals[0] = this->lambdaVal/(this->NumStates+this->lambdaVal) +
    (1-this->alpha*this->alpha + beta);
    for(i=1; i<this->CountHalfSPs*2+1; i++)
    {
        this->Wm.vec_vals[i] = 1.0/2.0*1.0/(this->NumStates+this->lambdaVal);
        this->Wc.vec_vals[i] = this->Wm.vec_vals[i];
    }
    this->Sbar = Covar;
    this->Sbar.MatOps_CholDecomp(this->Sbar);
    this->SQnoise.MatOps_CholDecomp(this->Qnoise);
    this->SQnoise.MatOps_transpose(this->SQnoise);
    return;
}

MatrixOperations UnscentKalFilt::ComputeSPVector(
                                                 MatrixOperations SPError,
                                                 MatrixOperations StateIn)
{
    MatrixOperations StateSum;
    StateSum.MatOps_add(SPError, StateIn);
    return(StateSum);
}

MatrixOperations UnscentKalFilt::ComputeObsError(
                                                 MatrixOperations SPError,
                                                 MatrixOperations StateIn)
{
    MatrixOperations StateSum;
    StateSum.MatOps_add(SPError, StateIn);
    return(StateSum);
}
void UnscentKalFilt::TimeUpdateFilter(double UpdateTime)
{
    int i, Index;
    MatrixOperations Xbar, SbarT;
    MatrixOperations Xcomp, AT, ARow, QAT, RAT, Xerr, XerrT, SPErrMat, SBarUp;
    SbarT.MatOps_transpose(this->Sbar);
    this->dt = UpdateTime - this->TimeTag;
    this->SP[0] = this->state;
    this->StateProp(this->SP[0]);
    Xbar.MatOps_init(this->NumStates, 1);
    Xbar = this->SP[0];
    Xbar.MatOps_scale(this->Wm.vec_vals[0]);
    for(i=0; i<this->CountHalfSPs; i++)
    {
        Index = i+1;
        this->SP[Index].MatOps_VecSet(&SbarT.vec_vals[i*this->NumStates]);
        this->SP[Index].MatOps_scale(this->gamma);
        this->SP[Index].MatOps_add(this->SP[Index], state);
        this->StateProp(this->SP[Index]);
        Xcomp = this->SP[Index];
        Xcomp.MatOps_scale(this->Wm.vec_vals[Index]);
        Xbar.MatOps_add(Xbar, Xcomp);
        Index = i+1+this->CountHalfSPs;
        this->SP[Index].MatOps_VecSet(&SbarT.vec_vals[i*this->NumStates]);
        this->SP[Index].MatOps_scale(-this->gamma);
        this->SP[Index].MatOps_add(this->SP[Index], state);
        this->StateProp(this->SP[Index]);
        Xcomp = this->SP[Index];
        Xcomp.MatOps_scale(this->Wm.vec_vals[Index]);
        Xbar.MatOps_add(Xbar, Xcomp);
    }
    AT.MatOps_init(2*this->CountHalfSPs+this->NumStates, this->NumStates);
    for(i=0; i<2*this->CountHalfSPs; i++)
    {
        ARow = Xbar;
        ARow.MatOps_scale(-1.0);
        ARow.MatOps_add(ARow, SP[i+1]);
        ARow.MatOps_scale(sqrt(this->Wc.vec_vals[i+1]));
        memcpy((void *)&AT.vec_vals[i*this->NumStates], (void *)ARow.vec_vals,
               this->NumStates*sizeof(double));
    }
    memcpy(&AT.vec_vals[2*this->CountHalfSPs*this->NumStates],
           this->SQnoise.vec_vals, this->NumStates*this->NumStates*sizeof(double));
    RAT.MatOps_QRD_JustR(AT);
    memcpy(SbarT.vec_vals, RAT.vec_vals, this->NumStates*this->NumStates
           *sizeof(double));
    this->Sbar.MatOps_transpose(SbarT);
    Xerr = Xbar;
    Xerr.MatOps_scale(-1.0);
    Xerr.MatOps_add(Xerr, SP[0]);
    XerrT.MatOps_transpose(Xerr);
    SPErrMat.MatOps_mult(Xerr, XerrT);
    SPErrMat.MatOps_scale(this->Wc.vec_vals[0]);
    SBarUp.MatOps_mult(this->Sbar, SbarT);
    SBarUp.MatOps_add(SBarUp, SPErrMat);
    this->Sbar.MatOps_CholDecomp(SBarUp);
    SbarT.MatOps_transpose(this->Sbar);
    this->SP[0] = Xbar;
    for(i=0; i<this->CountHalfSPs; i++)
    {
        Index = i+1;
        this->SP[Index].MatOps_VecSet(&SbarT.vec_vals[i*SbarT.dim_array[0]]);
        this->SP[Index].MatOps_scale(gamma);
        this->SP[Index].MatOps_add(this->SP[Index], Xbar);
        Index = i+1+this->CountHalfSPs;
        this->SP[Index].MatOps_VecSet(&SbarT.vec_vals[i*SbarT.dim_array[0]]);
        this->SP[Index].MatOps_scale(-gamma);
        this->SP[Index].MatOps_add(this->SP[Index], Xbar);
    }
    Covar.MatOps_transpose(Sbar);
    Covar.MatOps_mult(Sbar, Covar);
    state = SP[0];
    TimeTag = UpdateTime;
    
}

void UnscentKalFilt::MeasurementUpdate()
{
    int i;
    MatrixOperations ybar, SyInv, SyTInv, KMat, xhat, SbarT;
    MatrixOperations TempYVec, AT, QChol, QAT, RAT, SyT, Sy, YVecT, UpdMat, Pxy;
    this->ComputeMeasModel();
    ybar.MatOps_init(this->NumObs, 1);
    for(i=0; i<this->CountHalfSPs*2+1; i++)
    {
        TempYVec = this->YMeas[i];
        TempYVec.MatOps_scale(this->Wm.vec_vals[i]);
        ybar.MatOps_add(ybar, TempYVec);
    }
    AT.MatOps_init(this->CountHalfSPs*2+this->NumObs, this->NumObs);
    for(i=0; i<this->CountHalfSPs*2; i++)
    {
        TempYVec = ybar;
        TempYVec.MatOps_scale(-1.0);
        TempYVec.MatOps_add(TempYVec, this->YMeas[i+1]);
        TempYVec.MatOps_scale(sqrt(this->Wc.vec_vals[i+1]));
        memcpy(&AT.vec_vals[i*this->NumObs], TempYVec.vec_vals,
               this->NumObs*sizeof(double));
    }
    QChol.MatOps_CholDecomp(this->Q_obs);
    memcpy(&AT.vec_vals[2*this->CountHalfSPs*this->NumObs],
           QChol.vec_vals, QChol.dim_array[0]*QChol.dim_array[1]*sizeof(double));
    AT.MatOps_QRDecomp(QAT, RAT);
    //RAT.MatOps_QRD_JustR(QChol);
    SyT.MatOps_init(this->NumObs, this->NumObs);
    SyT.MatOps_VecSet(RAT.vec_vals);
    Sy.MatOps_transpose(SyT);
    TempYVec = ybar;
    TempYVec.MatOps_scale(-1.0);
    TempYVec.MatOps_add(TempYVec, this->YMeas[0]);
    YVecT.MatOps_transpose(TempYVec);
    UpdMat.MatOps_mult(TempYVec, YVecT);
    UpdMat.MatOps_scale(this->Wc.vec_vals[0]);
    SyT.MatOps_mult(Sy, SyT);
    UpdMat.MatOps_add(SyT, UpdMat);
    Sy.MatOps_CholDecomp(UpdMat);
    SyT.MatOps_transpose(Sy);
    Pxy.MatOps_init(this->NumStates, this->NumObs);
    for(i=0; i<2*this->CountHalfSPs+1; i++)
    {
        TempYVec = ybar;
        TempYVec.MatOps_scale(-1.0);
        TempYVec.MatOps_add(YMeas[i], TempYVec);
        TempYVec.MatOps_transpose(TempYVec);
        UpdMat = this->SP[0];
        UpdMat.MatOps_scale(-1.0);
        UpdMat.MatOps_add(UpdMat, this->SP[i]);
        UpdMat.MatOps_scale(this->Wc.vec_vals[i]);
        UpdMat.MatOps_mult(UpdMat, TempYVec);
        Pxy.MatOps_add(Pxy, UpdMat);
    }
    SyInv.MatOps_InvLMat(Sy);
    SyTInv.MatOps_InvUMat(SyT);
    KMat.MatOps_mult(Pxy, SyTInv);
    KMat.MatOps_mult(KMat, SyInv);
    xhat = ybar;
    xhat.MatOps_scale(-1.0);
    xhat = this->ComputeObsError(obs, xhat);
    xhat.MatOps_mult(KMat, xhat);
    state = this->ComputeSPVector(xhat, this->SP[0]);
    UpdMat.MatOps_mult(KMat, Sy);
    TempYVec.MatOps_init(this->NumStates, 1);
    for(i=0; i<this->NumObs; i++)
    {
        TempYVec.MatOps_VecSet(&UpdMat.vec_vals[i*this->NumObs]);
        SbarT.MatOps_transpose(this->Sbar);
        this->CholDownDate(SbarT, TempYVec);
        this->Sbar.MatOps_transpose(SbarT);
    }
    
    return;
}

void UnscentKalFilt::CholDownDate(MatrixOperations &R, MatrixOperations &XVec)
{
    int i, j, k, matdim;
    double MatNorm, alpha, scale, a, b, T, xx;
    MatrixOperations RT, S, C;
    
    matdim = R.dim_array[0]; // Square matrix
    RT.MatOps_transpose(R);
    S.MatOps_init(XVec.dim_array[0], XVec.dim_array[1]);
    C.MatOps_init(XVec.dim_array[0], XVec.dim_array[1]);
    for(i=0; i<matdim; i++)
    {
        S.vec_vals[i] = XVec.vec_vals[i];
        for(j=0; j<=i-1; j++)
        {
            S.vec_vals[i] -= RT.vec_vals[i*matdim+j]*S.vec_vals[j];
        }
        S.vec_vals[i] /= RT.vec_vals[i*matdim+i];
    }
    MatNorm = S.MatOps_twonorm();
    if(MatNorm > 1.0)
    {
        return;
    }
    alpha = sqrt(1.0 - MatNorm*MatNorm);
    for(i=1; i<=matdim; i++)
    {
        j = matdim - i + 1;
        scale = alpha + fabs(S.vec_vals[j-1]);
        a = alpha/scale;
        b = S.vec_vals[j-1]/scale;
        MatNorm = sqrt(a*a+b*b);
        C.vec_vals[j-1] = a/MatNorm;
        S.vec_vals[j-1] = b/MatNorm;
        alpha = scale*MatNorm;
    }
    for(i=1; i<=matdim; i++)
    {
        xx = 0.0;
        for(j=1; j<=i; j++)
        {
            k = i-j+1;
            T = C.vec_vals[k-1]*xx + S.vec_vals[k-1] * R.vec_vals[(k-1)*matdim +
                                                                  (i-1)];
            R.vec_vals[(k-1)*matdim+(i-1)] = C.vec_vals[k-1]*R.vec_vals[(k-1)*
                                                                        matdim+(i-1)] - S.vec_vals[k-1]*xx;
            xx = T;
        }
    }
    
    return;
}
