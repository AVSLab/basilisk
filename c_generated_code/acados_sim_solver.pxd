#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

cimport acados_sim_solver_common

cdef extern from "acados_sim_solver_satellite_others.h":
    ctypedef struct sim_solver_capsule "satellite_others_sim_solver_capsule":
        pass

    sim_solver_capsule * acados_sim_solver_create_capsule "satellite_others_acados_sim_solver_create_capsule"()
    int acados_sim_solver_free_capsule "satellite_others_acados_sim_solver_free_capsule"(sim_solver_capsule *capsule)

    int acados_sim_create "satellite_others_acados_sim_create"(sim_solver_capsule * capsule)
    int acados_sim_solve "satellite_others_acados_sim_solve"(sim_solver_capsule * capsule)
    int acados_sim_free "satellite_others_acados_sim_free"(sim_solver_capsule * capsule)
    int acados_sim_update_params "satellite_others_acados_sim_update_params"(sim_solver_capsule * capsule, double *value, int np_)
    # int acados_sim_update_params_sparse "satellite_others_acados_sim_update_params_sparse"(sim_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)

    acados_sim_solver_common.sim_in *acados_get_sim_in "satellite_others_acados_get_sim_in"(sim_solver_capsule * capsule)
    acados_sim_solver_common.sim_out *acados_get_sim_out "satellite_others_acados_get_sim_out"(sim_solver_capsule * capsule)
    acados_sim_solver_common.sim_solver *acados_get_sim_solver "satellite_others_acados_get_sim_solver"(sim_solver_capsule * capsule)
    acados_sim_solver_common.sim_config *acados_get_sim_config "satellite_others_acados_get_sim_config"(sim_solver_capsule * capsule)
    acados_sim_solver_common.sim_opts *acados_get_sim_opts "satellite_others_acados_get_sim_opts"(sim_solver_capsule * capsule)
    void *acados_get_sim_dims "satellite_others_acados_get_sim_dims"(sim_solver_capsule * capsule)
