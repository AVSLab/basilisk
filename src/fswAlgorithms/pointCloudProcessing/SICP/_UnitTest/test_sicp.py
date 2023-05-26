# ISC License
#
# Copyright (c) 2023, Laboratory for Atmospheric Space Physics, University of Colorado Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


import glob
import inspect
import numpy as np
import os
import pickle
import pytest
import testFunctions

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

from Basilisk.utilities import RigidBodyKinematics as rbk

@pytest.mark.parametrize("mrp, scale, translations" ,[
    ([0.01, -0.005, 0.1], 0.92, [0.02, 0.01, 0.05]),
    ([0.025, -0.011, 0.04], 0.98, [-0.02, 0.005, -0.05]),
    ([-0.02, -0.03, -0.02], 1.08, [0.1, -0.01, 0.05])

])
def test_module(show_plots, mrp, scale, translations):
    """
    Unit test for SICP algorithm. The unit test specifically tests with different initial conditions using pickled data.

    The drill_bit data is credited to the Stanford University Computer Graphics Laboratory
    This data is being used in a research/scholarly purpose and cannot be used for commercial purposes.

    For more information: www-graphics.stanford.edu
    """
    sicpTest(show_plots, mrp, scale, translations)
    return

def test_pythonComparison():
    """
    Unit test for SICP algorithm. The unit test specifically compares the cpp module results
    to a python implementation.
    Given the speed of the python test, it is only run once with general errors,
    and all three outputs must be within a percent of the inputs
    """
    pythonTest()
    return

"""
The following three tests isolate each transformation 
1. The first test only translates the data, all three outputs must be within a percent of the inputs
2. The second test only rotates the data, all three outputs must be within a percent of the inputs
3. The third test only scales the data, all three outputs must be within a percent of the inputs
"""
def test_translation(show_plots):
    sicpTest(show_plots, [0., 0., 0.], 1, [0.02, 0.001, 0.005])
    return

def test_rotation(show_plots):
    sicpTest(show_plots, [0.1, -0.05, 0.1], 1, [0, 0, 0])
    return

def test_scale(show_plots):
    sicpTest(show_plots, [0., 0., 0.], 1.05, [0.0, 0.0, 0.0])
    return

def pythonTest():
    R_test = rbk.MRP2C( [0.01, -0.02, 0.01])
    s_test = 1.1
    t_test = np.array([0.02, 0.001, 0.005])

    # Truth data
    file1 = open(path + '/Stanford_Computer_Graphics_Laboratory_Data/drill_1.6mm_0_cyb.pickle', 'rb')
    data_file = pickle.load(file1)
    file1.close()

    data = np.zeros([5000, 3])
    reference = np.zeros(np.shape(data))
    reference_file = np.zeros(np.shape(data_file))
    for j in range(len(data_file[:,0])):
        reference[j,:] = s_test*np.dot(R_test, data_file[j,:]) + t_test
        reference_file[j,:] = s_test*np.dot(R_test, data_file[j,:]) + t_test
        data[j, :] = data_file[j,:]

    module_Rs, module_Ts, module_Ss, module_cloud = testFunctions.runModule(data, reference, len(data_file[:,0]), iters= 10)
    s, R, t, list = testFunctions.pythonSICP(data, reference, len(data_file[:,0]), scale_minMax = [0.9,1.1])

    accuracy = 1E-2
    np.testing.assert_allclose(rbk.C2MRP(R),
                               rbk.C2MRP(module_Rs[-1,:,:]),
                               atol=accuracy,
                               err_msg=('Variable: Rotation matrix'),
                               verbose=True)
    np.testing.assert_allclose(s,
                               module_Ss[-1],
                               atol=accuracy,
                               err_msg=('Variable: Scale factor'),
                               verbose=True)
    np.testing.assert_allclose(t,
                               module_Ts[-1,:],
                               atol=accuracy,
                               err_msg=('Variable: Translation vector'),
                               verbose=True)
    return

def sicpTest(show_plots, mrp, scale, translations):
    R_test = rbk.MRP2C(mrp)
    s_test = scale
    t_test = np.array(translations)

    # Truth data
    file1 = open(path + '/Stanford_Computer_Graphics_Laboratory_Data/drill_1.6mm_0_cyb.pickle', 'rb')
    data_file = pickle.load(file1)
    file1.close()

    data = np.zeros([5000, 3])
    reference = np.zeros(np.shape(data))
    reference_file = np.zeros(np.shape(data_file))
    for j in range(len(data_file[:,0])):
        reference[j,:] = s_test*np.dot(R_test, data_file[j,:]) + t_test
        reference_file[j,:] = s_test*np.dot(R_test, data_file[j,:]) + t_test
        data[j, :] = data_file[j,:]

    module_Rs, module_Ts, module_Ss, module_cloud = testFunctions.runModule(data, reference,len(data_file[:,0]))

    costFunction = np.zeros(len(module_Ss))
    for j in range(len(module_Ss)):
        for i in range(len(data_file[:,0])):
            costFunction[j] += np.linalg.norm(reference_file[i,:] -
                                              (module_Ss[j]*np.dot(module_Rs[j,:,:], data_file[i,:]) + module_Ts[j,:]))**2

    if show_plots:
        testFunctions.saveImages(data_file, reference_file, module_Ss, module_Rs, module_Ts)
        testFunctions.saveCostFunctions(costFunction, "Test")

    accuracy = 1E-2
    np.testing.assert_allclose(module_cloud,
                               reference_file,
                               atol=accuracy,
                               rtol = 0,
                               err_msg=('Output: point cloud vs expected'),
                               verbose=True)
    np.testing.assert_allclose(s_test,
                               module_Ss[-1],
                               atol=accuracy,
                               rtol = 0,
                               err_msg=('Variable: Scale factor'),
                               verbose=True)
    np.testing.assert_allclose(t_test,
                               module_Ts[-1,:],
                               rtol = 0,
                               atol=accuracy,
                               err_msg=('Variable: Translation vector'),
                               verbose=True)
    np.testing.assert_allclose(rbk.C2MRP(R_test),
                               rbk.C2MRP(module_Rs[-1,:,:]),
                               atol=accuracy,
                               rtol = 0,
                               err_msg=('Variable: Rotation matrix'),
                               verbose=True)
    np.testing.assert_array_less(np.diff(costFunction),
                               0,
                               err_msg=('Metric: Cost function is not monotonously decreasing'),
                               verbose=True)

    return

if __name__ == "__main__":
    sicpTest(True, [0.01, -0.02, 0.01], 1.1, [0.02, 0.001, 0.005])
