/*==========================================================
 *
 * This is a MEX-file for MATLAB.
 * Copyright 2007-2012 The MathWorks, Inc.
 *
 *========================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mex.h"

/* The function that computes a dot product of two 3D vector */
double dotProduct(double *a, double *b){
    /* Assign variable to dot product output */
    double c;
    
    /* Compute dot product */
    c = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    
    /* Return output */
    return c;
}

/* The function that computes a cross product of two 3D vectors */
void crossProduct(double *c, double *a, double *b){
    /* Compute cross product for each component */
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2]; 
    c[2] = a[0]*b[1] - a[1]*b[0];
}

/* The function that computes a 3D vector norm L2 */
double normVector(double *a){
    /* Declare variable to loop through its 3 components */
    int i_3D;
    
    /* Declare variable to store the norm */
    double norm = 0.0;
    
    /* Loop through all its 3 components */
    for (i_3D = 0; i_3D < 3; i_3D++){
        /* Summatory add square of each vector component */
        norm += pow(a[i_3D], 2.0);
    }
    
    /* Compute the square root of the root components summatory */
    norm = pow(norm, 0.5);
    
    /* Return norm value */
    return norm;
}

/* The function that computes the min between two integer numbers */
int intmin(int a, int b){
    /* Define output number */
    int c;
    
    /* If statement to check what number is lower */
    if (a < b){
        c = a;
    }
    else if (b < a){
        c = b;
    }
    else{
        c = a;
    }
    
    /* Return min value */
    return c;
}

/* The function that computes the dyad product between two 3D vectors */
void dyadProduct(double c[3][3], double *a, double *b){
    /* Declare auxiliary variables to loop */
    int i, j;
    
    /* Loop through rows */
    for (i = 0; i < 3; i++){
        /*Loop through columns */
        for (j = 0; j < 3; j++){
            c[i][j] = a[i]*b[j]; 
        }
    }
}

/* The function that computes a product between a 3D matrix and vector and
 multiplies the result with a desired scalar if neccesary*/
void matrixvectorProduct(double *c, double a[3][3], double *b,
        double factor, int flag){
    /* Define for loops auxiliary variable */
    int i, j;
    
    /* Reset c values to zero (to not interfere with summatories if c is
     * filled) */
    c[0] = 0.0;
    c[1] = 0.0;
    c[2] = 0.0;
    
    /* If statement to decide if the vector will multiply by the right side
     * or the left side */
    if (flag == 1){
        /* Premultiply by the left side (row vector) */
        for (i = 0; i < 3; i++){
            for (j = 0; j < 3; j++){
                c[i] = c[i] + a[j][i]*b[j];
            }
            
            /* Multiply vector value by multiplication factor */
            c[i] = c[i] * factor;
        }
    }
    else{
        /* By default multiply by the right side (column vector) */
        for (i = 0; i < 3; i++){
            for (j = 0; j < 3; j++){
                c[i] = c[i] + a[i][j]*b[j];
                /*printf("%lf \t %lf \t %lf \t %lf\n", c[i], a[i][j], b[j], factor);*/
            }
            
            /* Multiply vector component value by multiplication factor */
            c[i] = c[i] * factor;
        }
    }
}

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{   
    /* Macros for the input arguments */
    #define r_IN prhs[0]
    #define xyz_IN prhs[1]
    #define order_IN prhs[2]
    #define rho_IN prhs[3]
    #define G_IN prhs[4]
    
    /* Macros for the output arguments */
    #define dU_OUT plhs[0]
    #define lU_OUT plhs[1]
    
    /* Declare input arguments */
    double *r, *xyz, *order, rho, G;
    
    /* Declare parameters for the input arguments */
    int M_xyz, N_xyz;
    int m_xyz, n_xyz;
    double r_xyz_norm;
    int M_order, N_order;
    int m_order, n_order;
    
    /* Declare output arguments */
    double *dU, *lU;
   
    /* Declare auxiliary variables in for loops */
    int i_3D;
    
    int i, j, k;
    
    double ri_idx[3], rj_idx[3], rk_idx[3];
               
    double norm_ri_idx, norm_rj_idx, norm_rk_idx;
    
    double r_edge1[3], r_edge2[3];
    
    double nf_vec[3];
    double norm_nf_vec;
    double nf_idx[3];
    
    int i_min;
    
    double r1[3], r2[3], re[3];
    double a, b;
    
    double r12[3];
    double e;
    
    double n12_vec[3], n12[3];
    double norm_n12_vec;
    
    double Le;
    double Ee[3][3];
    double EereLe[3] = {0, 0, 0};
    
    double dUe[3] = {0, 0, 0};
    
    double cross_rj_rk[3];
    double wf;
           
    double dot_nf_rf;
    double dot_nf_rf_wf;
    
    double dUf[3] = {0, 0, 0};
    
    /* Get the pointers to the data of r, xyz and order */
    r = mxGetPr(r_IN);
    xyz = mxGetPr(xyz_IN);
    order = mxGetPr(order_IN);
    
    /* Get the xyz dimensions (rows and columns) */
    M_xyz = mxGetM(xyz_IN);
    N_xyz = mxGetN(xyz_IN);
    
    /* Get the order dimensions (rows and columns) */
    M_order = mxGetM(order_IN);
    N_order = mxGetN(order_IN);
    
    /* Get the values for rho and G */
    rho = mxGetScalar(rho_IN);
    G = mxGetScalar(G_IN);
    
    /* Create the output vector and scalar */
    dU_OUT = mxCreateDoubleMatrix(3, 1, mxREAL);
    lU_OUT = mxCreateDoubleScalar(0);
    
    /* Get the pointer to the data of dU */
    dU = mxGetPr(dU_OUT);
    
    /* Get lU and initialize it */
    lU = mxGetPr(lU_OUT);
    lU[0] = 0.0;
    
    /* Loop through each facect */
    for (m_order = 0; m_order < M_order; m_order++){
        /* Fill auxiliary variables with vertex order on each facet  
         * (1 has to be substracted because C arrays starts in 0)*/
        i = order[m_order] - 1;
        j = order[m_order + M_order] - 1;
        k = order[m_order + 2*M_order] - 1;   
        
        /* Loop through each 3D position */
        for (i_3D = 0; i_3D < 3; i_3D++){
            /* Compute vectors going from each vertex to the evaluation
             * point */
            ri_idx[i_3D] = xyz[i + M_xyz*i_3D] - r[i_3D];
            rj_idx[i_3D] = xyz[j + M_xyz*i_3D] - r[i_3D];
            rk_idx[i_3D] = xyz[k + M_xyz*i_3D] - r[i_3D];
            
            /* Compute two edge vectors for a-posteriori normal
             * facet computation */
            r_edge1[i_3D] = rj_idx[i_3D] - ri_idx[i_3D];
            r_edge2[i_3D] = rk_idx[i_3D] - rj_idx[i_3D]; 
        }
        
        /* Compute norm of vectors going from each vertex to the evaluation
         * point */
        norm_ri_idx = normVector(ri_idx);
        norm_rj_idx = normVector(rj_idx);
        norm_rk_idx = normVector(rk_idx);
        
        /* Compute facet normal vector */
        crossProduct(nf_vec, r_edge1, r_edge2);
        
        /* Compute facet normal vector norm */
        norm_nf_vec = normVector(nf_vec);
                
        /* Compute normalized facet normal vector */
        nf_idx[0] = nf_vec[0] / norm_nf_vec;
        nf_idx[1] = nf_vec[1] / norm_nf_vec;
        nf_idx[2] = nf_vec[2] / norm_nf_vec;
        
        /* Loop through each facet edge */
        for (n_order = 0; n_order < N_order; n_order++){
            /* Switch to determine edge being computed */
            switch(n_order){
                case 0 :
                    /* Obtain smallest vertex index, , in this way, the
                     * same vertex is used both times the calculations are
                     * performed for that particular edge: one edge belongs
                     * to two faces*/
                    i_min = intmin(i, j);
                    
                    /* Loop through 1,2,3 */
                    for (i_3D = 0; i_3D < 3; i_3D++){
                        r1[i_3D] = ri_idx[i_3D];
                        r2[i_3D] = rj_idx[i_3D];
                        re[i_3D] = xyz[i_min + M_xyz*i_3D] - r[i_3D];
                    }
                    
                    /* Assign norm */
                    a = norm_ri_idx;
                    b = norm_rj_idx;
                    break;
                case 1 :
                    /* Obtain smallest vertex index, , in this way, the
                     * same vertex is used both times the calculations are
                     * performed for that particular edge: one edge belongs
                     * to two faces*/
                    i_min = intmin(j, k);
                    
                    /* Loop through 1,2,3 */
                    for (i_3D = 0; i_3D < 3; i_3D++){
                        r1[i_3D] = rj_idx[i_3D];
                        r2[i_3D] = rk_idx[i_3D];
                        re[i_3D] = xyz[i_min + M_xyz*i_3D] - r[i_3D];
                    }
                    
                    /* Assign norm */
                    a = norm_rj_idx;
                    b = norm_rk_idx;
                    break;
                case 2 :
                    /* Obtain smallest vertex index, , in this way, the
                     * same vertex is used both times the calculations are
                     * performed for that particular edge: one edge belongs
                     * to two faces*/
                    i_min = intmin(i, k);
                    
                    /* Loop through 1,2,3 */
                    for (i_3D = 0; i_3D < 3; i_3D++){
                        r1[i_3D] = rk_idx[i_3D];
                        r2[i_3D] = ri_idx[i_3D];
                        re[i_3D] = xyz[i_min + M_xyz*i_3D] - r[i_3D];
                    }
                    
                    /* Assign norm */
                    a = norm_rk_idx;
                    b = norm_ri_idx;
                    break;
            }
                    
            /* Compute along edge vector and norm */
            r12[0] = r2[0] - r1[0];
            r12[1] = r2[1] - r1[1];
            r12[2] = r2[2] - r1[2];
            e = normVector(r12);
                        
            /* Compute normal vector to edge, n12_vec, and its norm */
            crossProduct(n12_vec, r12, nf_idx);
            norm_n12_vec = normVector(n12_vec);
        
            /* Normalize normal vector to edge */
            n12[0] = n12_vec[0] / norm_n12_vec;
            n12[1] = n12_vec[1] / norm_n12_vec;
            n12[2] = n12_vec[2] / norm_n12_vec;
            
            /* Dimensionless per-edge factor */
            Le = log((a+b+e) / (a+b-e));
        
            /* Compute dyad product between nf_idx and n12 */
            dyadProduct(Ee, nf_idx, n12);
            
            /* Compute Le*Ee*re */
            matrixvectorProduct(EereLe, Ee, re, Le, 0);
                        
            /* Add current facet contribution to dUe */
            dUe[0] = dUe[0] + EereLe[0];
            dUe[1] = dUe[1] + EereLe[1];
            dUe[2] = dUe[2] + EereLe[2];
        }
        /* Compute auxiliary vector to compute then solid angle angle
         * per facet */
        crossProduct(cross_rj_rk, rj_idx, rk_idx);
        
        /* Compute solid angle for the current facet. Dimensionless
         * per-face factor*/
        wf = 2*atan2(dotProduct(ri_idx, cross_rj_rk),
                norm_ri_idx*norm_rj_idx*norm_rk_idx + 
                norm_ri_idx*dotProduct(rj_idx, rk_idx) + 
                norm_rj_idx*dotProduct(rk_idx, ri_idx) + 
                norm_rk_idx*dotProduct(ri_idx, rj_idx));
        
        /* Compute auxiliary dot product for solid angle contribution.
         * rf is taken as ri */
        dot_nf_rf = dotProduct(nf_idx, ri_idx);
        
        /* Auxiliary constant term to compute current solid angle facet
         * contribution */
        dot_nf_rf_wf = dot_nf_rf*wf;
        
        /* Add current solid angle facet contribution to dUf */
        dUf[0] = dUf[0] + nf_idx[0]*dot_nf_rf_wf;
        dUf[1] = dUf[1] + nf_idx[1]*dot_nf_rf_wf;
        dUf[2] = dUf[2] + nf_idx[2]*dot_nf_rf_wf;
        
        /* Update lU with solid angle */
        lU[0] = lU[0] + wf;
     }
    
    /* Potential computation*/
    dU[0] = G*rho*(-dUe[0] + dUf[0]);
    dU[1] = G*rho*(-dUe[1] + dUf[1]);
    dU[2] = G*rho*(-dUe[2] + dUf[2]);
    
    return;
}

