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
#ifndef _MatrixOperations_HH_
#define _MatrixOperations_HH_

typedef enum{// MatEulerSeq ---------------------------------------------------
   MatOps_RPY = 0,
   MatOps_YPR = 1,
   MatOps_PYR = 2,
   MatOps_YRP = 3
}MatEulerSeq; 

typedef struct { //MatOpsEuler ------------------------------------------------
   double Roll;
   double Pitch;
   double Yaw;
}MatOpsEuler;

class MatrixOperations{
   public:
      MatrixOperations();
      MatrixOperations(int nrow, int ncol, double *matrix);
      MatrixOperations(const MatrixOperations &source);
      ~MatrixOperations();
      void     MatOps_clear();  // No return for this function
      void     MatOps_init( // No return for this function
         int  num_dim,          // Number of dimensions for the matrix
         int *dim_vector);      // dimenstion list for the matrix      
      void     MatOps_init(int nrow, int ncol); // 2 dim matrix to zeros 
      void operator =(const MatrixOperations &source); //Assignment operator
      void MatOps_add(
         MatrixOperations A,             // First operand in additiooon
         MatrixOperations B);            // Second operand in addition
      void     MatOps_ident();  // Make Kalman matrix identity with current dimen
      void MatOps_mult(                   // -- multiply A and B to get this
         MatrixOperations mat_A,             // -- Matrix A in the multiplication scheme
         MatrixOperations mat_B);            // -- Matrix B in the mult schemei
      void MatOps_transpose(              // -- Make this the transpose of A
         MatrixOperations mat_A);            // -- Matrix to transpose
      void MatOps_crossprod(
         MatrixOperations mat_A,             // -- Matrix A in the multiplication scheme
         MatrixOperations mat_B);            // -- Matrix B in the mult schemei
      double MatOps_twonorm();            // -- Norm of the matrix
      double MatOps_twonorm_square();            // -- Norm of the matrix^2
      double MatOps_onenorm();            // -- Max column sum norm
      void MatOps_scale(                // Scale entire matrix by factor
         double scal_mult);              
      double MatOps_dotprod(
         MatrixOperations mat_B);            // -- Matrix B in the mult schemei 
      void MatOps_Inv3x3(
         MatrixOperations mat_A);            // -- Matrix B in the mult schemei
      void MatOps_Quat2DCM(
         MatrixOperations Q_in);             // -- Quat to chng (scal, vect)
      void MatOps_DCM2Quat(
         MatrixOperations T_in);             // -- Quat to chng (scal, vect)
      void MatOps_QuatMult(
         MatrixOperations Q_A,               // -- First quat to multiply
         MatrixOperations Q_B);              // -- Second quat to multiply
      void MatOps_VecSet(
         double *VecData);
      void MatOps_QuatTrans(
         MatrixOperations Q_A);              // -- Quaternion to transpose
      void MatOps_QRDecomp(
          MatrixOperations &Qout,
          MatrixOperations &ROut);
      void MatOps_QRD_JustR(
          MatrixOperations MatIn);
      void MatOps_CholDecomp(
         MatrixOperations MatIn);
      void MatOps_InvLMat(
         MatrixOperations MatIn);
      void MatOps_InvUMat(
         MatrixOperations MatIn);
      void MatOps_ShadowMRP(
         MatrixOperations MRPIn);
      void MatOps_MRP2Quat(
         MatrixOperations MRPIn);
      void MatOps_Quat2MRP(
         MatrixOperations Q_A);
      void MatOps_EulerAng2DCM(
         MatOpsEuler EulerAngles,
         MatEulerSeq sequence);
      void MatOps_EnforceMinimum(double MinAllow);
      void MatOps_EnforceMaximum(double MaxAllow);
      void MatOps_SortMatrixDescending(MatrixOperations UnsortedMat);
       

   public:
      double   *vec_vals;       /* --  array of values for Kalman array       */
      int       dim_length;     /* --  Number of dimensions for matrix        */
      int      *dim_array;      /* --  value of matrix dimensions             */
      bool      init;           // --  Matrix has been initialized           
};

template <typename T> int sgn(T val) {
   if(val > 0)
   {
      return(1);
   }
   if(val < 0)
   {
      return(-1);
   }
   return(0);
};

#endif /* _CTRL_CLASS_H_ */
