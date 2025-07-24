/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SIM_satellite_others_H_
#define ACADOS_SIM_satellite_others_H_

#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

#define SATELLITE_OTHERS_NX     6
#define SATELLITE_OTHERS_NZ     0
#define SATELLITE_OTHERS_NU     3
#define SATELLITE_OTHERS_NP     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct satellite_others_sim_solver_capsule
{
    // acados objects
    sim_in *acados_sim_in;
    sim_out *acados_sim_out;
    sim_solver *acados_sim_solver;
    sim_opts *acados_sim_opts;
    sim_config *acados_sim_config;
    void *acados_sim_dims;

    /* external functions */
    // ERK
    external_function_param_casadi * sim_expl_vde_forw;
    external_function_param_casadi * sim_vde_adj_casadi;
    external_function_param_casadi * sim_expl_ode_fun_casadi;
    external_function_param_casadi * sim_expl_ode_hess;

    // IRK
    external_function_param_casadi * sim_impl_dae_fun;
    external_function_param_casadi * sim_impl_dae_fun_jac_x_xdot_z;
    external_function_param_casadi * sim_impl_dae_jac_x_xdot_u_z;
    external_function_param_casadi * sim_impl_dae_hess;

    // GNSF
    external_function_param_casadi * sim_gnsf_phi_fun;
    external_function_param_casadi * sim_gnsf_phi_fun_jac_y;
    external_function_param_casadi * sim_gnsf_phi_jac_y_uhat;
    external_function_param_casadi * sim_gnsf_f_lo_jac_x1_x1dot_u_z;
    external_function_param_casadi * sim_gnsf_get_matrices_fun;

} satellite_others_sim_solver_capsule;


ACADOS_SYMBOL_EXPORT int satellite_others_acados_sim_create(satellite_others_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT int satellite_others_acados_sim_solve(satellite_others_sim_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int satellite_others_acados_sim_free(satellite_others_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT int satellite_others_acados_sim_update_params(satellite_others_sim_solver_capsule *capsule, double *value, int np);

ACADOS_SYMBOL_EXPORT sim_config * satellite_others_acados_get_sim_config(satellite_others_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT sim_in * satellite_others_acados_get_sim_in(satellite_others_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT sim_out * satellite_others_acados_get_sim_out(satellite_others_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT void * satellite_others_acados_get_sim_dims(satellite_others_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT sim_opts * satellite_others_acados_get_sim_opts(satellite_others_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT sim_solver * satellite_others_acados_get_sim_solver(satellite_others_sim_solver_capsule *capsule);


ACADOS_SYMBOL_EXPORT satellite_others_sim_solver_capsule * satellite_others_acados_sim_solver_create_capsule(void);
ACADOS_SYMBOL_EXPORT int satellite_others_acados_sim_solver_free_capsule(satellite_others_sim_solver_capsule *capsule);

#ifdef __cplusplus
}
#endif

#endif  // ACADOS_SIM_satellite_others_H_
