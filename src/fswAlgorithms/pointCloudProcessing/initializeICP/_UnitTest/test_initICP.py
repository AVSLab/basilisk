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

import inspect
import numpy as np
import os

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, macros

from Basilisk.fswAlgorithms import initializeICP
from Basilisk.architecture import messaging


def test_all_valid():
    """
    Unit test with valid ICP message and valid point cloud
    """
    icp_init_sim(validICP = True, validCloud = True, normalize = True)

def test_all_valid_no_normalization():
    """
    Unit test with valid ICP message and valid point cloud without normalization
    """
    icp_init_sim(validICP = True, validCloud = True, normalize = False)

def test_valid_cloud():
    """
    Unit test with valid ICP message and valid point cloud
    """
    icp_init_sim(validICP = False, validCloud = True, normalize = True)

def test_valid_ICP():
    """
    Unit test with valid ICP message and valid point cloud
    """
    icp_init_sim(validICP = True, validCloud = False, normalize = True)


def icp_init_sim(validICP = True, validCloud = True, normalize = True):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(0.5)
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, process_rate))

    # setup module to be tested
    module = initializeICP.InitializeICP()
    # module.ModelTag = 'initializationICP'
    module.normalizeMeasuredCloud = normalize
    unit_test_sim.AddModelToTask(unit_task_name, module)

    ephemeris_input_msg_buffer = messaging.EphemerisMsgPayload()
    ephemeris_input_msg_buffer.r_BdyZero_N = [100, 1, 0]  # use v_C1_N instead of v_BN_N for accurate unit test
    ephemeris_input_msg_buffer.sigma_BN = [0.1, 0., 1]  # use v_C1_N instead of v_BN_N for accurate unit test
    ephemeris_input_msg_buffer.timeTag = 1 * 1E9
    ephemeris_input_msg = messaging.EphemerisMsg().write(ephemeris_input_msg_buffer)
    module.ephemerisInMsg.subscribeTo(ephemeris_input_msg)

    camera_input_msg_buffer = messaging.CameraConfigMsgPayload()
    camera_input_msg_buffer.cameraID = 1
    camera_input_msg_buffer.sigma_CB = [0.1, 0.2, 0.3]
    camera_input_msg = messaging.CameraConfigMsg().write(camera_input_msg_buffer)
    module.cameraConfigInMsg.subscribeTo(camera_input_msg)

    input_points = np.array([[1, 1, 1], [2, 2, 2], [5, 1, 3], [0, 2, 0], [0, 0, 1], [0, 0, -1]])
    pointcloud_input_msg_buffer = messaging.PointCloudMsgPayload()
    pointcloud_input_msg_buffer.points = input_points.flatten().tolist()
    pointcloud_input_msg_buffer.numberOfPoints = len(input_points)
    pointcloud_input_msg_buffer.timeTag = 1
    pointcloud_input_msg_buffer.valid = validCloud
    pointcloud_input_msg = messaging.PointCloudMsg().write(pointcloud_input_msg_buffer)
    module.inputMeasuredPointCloud.subscribeTo(pointcloud_input_msg)

    icp_input_msg_buffer = messaging.SICPMsgPayload()
    icp_input_msg_buffer.rotationMatrix = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    icp_input_msg_buffer.translation = [1, 0, 0]
    icp_input_msg_buffer.scaleFactor = [1.05]
    icp_input_msg_buffer.numberOfIteration = 1
    icp_input_msg_buffer.valid = validICP
    icp_input_msg_buffer.timeTag = 500
    icp_input_msg = messaging.SICPMsg().write(icp_input_msg_buffer)
    module.inputSICPData.subscribeTo(icp_input_msg)

    icp_msg_log = module.initializeSICPMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, icp_msg_log)
    cloud_msg_log = module.measuredPointCloud.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, cloud_msg_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(process_rate)
    unit_test_sim.ExecuteSimulation()

    output_iterations = icp_msg_log.numberOfIteration[-1] + 1
    output_Ss = icp_msg_log.scaleFactor[-1][:output_iterations]
    output_Rs = icp_msg_log.rotationMatrix[-1][:9*output_iterations]
    output_Ts = icp_msg_log.translation[-1][:3*output_iterations]

    output_cloud_valid = cloud_msg_log.valid[-1]
    output_cloud_timetag = cloud_msg_log.timeTag[-1]
    output_cloud_numberpoints = cloud_msg_log.numberOfPoints[-1]
    output_pointcloud = cloud_msg_log.points[-1][:3*output_cloud_numberpoints]

    point_cloud_module = np.array(output_pointcloud).reshape([output_cloud_numberpoints, 3])

    accuracy = 1E-10
    if module.normalizeMeasuredCloud:
        avg = np.mean(np.linalg.norm(input_points, axis=1))
    else:
        avg = 1
    if validCloud and validICP:
        np.testing.assert_allclose(point_cloud_module,
                                   input_points/avg,
                                   rtol=0,
                                   atol=accuracy,
                                   err_msg=('Normalized point cloud vs expected'),
                                   verbose=True)
        np.testing.assert_allclose(output_Ss,
                                   icp_input_msg_buffer.scaleFactor[0:1],
                                   rtol=0,
                                   atol=accuracy,
                                   err_msg=('Scale: ICP parameters vs expected'),
                                   verbose=True)
        np.testing.assert_allclose(output_Ts,
                                   icp_input_msg_buffer.translation[0:3],
                                   rtol=0,
                                   atol=accuracy,
                                   err_msg=('Translation: ICP parameters vs expected'),
                                   verbose=True)
        np.testing.assert_allclose(output_Rs,
                                   icp_input_msg_buffer.rotationMatrix[0:9],
                                   rtol=0,
                                   atol=accuracy,
                                   err_msg=('Rotation: ICP parameters vs expected'),
                                   verbose=True)
    if not validCloud:
        np.testing.assert_allclose(point_cloud_module,
                                   np.zeros(np.shape(point_cloud_module)),
                                   rtol=0,
                                   atol=accuracy,
                                   err_msg=('Normalized point cloud vs expected'),
                                   verbose=True)
    if not validICP and validCloud:
        np.testing.assert_allclose(output_Ss,
                                   [1],
                                   rtol=0,
                                   atol=accuracy,
                                   err_msg=('Invalid ICP: ICP parameters vs expected'),
                                   verbose=True)
        np.testing.assert_allclose(output_Ts,
                                   np.array(ephemeris_input_msg_buffer.r_BdyZero_N),
                                   rtol=0,
                                   atol=accuracy,
                                   err_msg=('Invalid ICP: ICP parameters vs expected'),
                                   verbose=True)
        BN = rbk.MRP2C(ephemeris_input_msg_buffer.sigma_BN)
        CB = rbk.MRP2C(camera_input_msg_buffer.sigma_CB)
        np.testing.assert_allclose(np.array(output_Rs).reshape([3, 3]),
                                   np.dot(CB, BN),
                                   rtol=0,
                                   atol=accuracy,
                                   err_msg=('Invalid ICP: ICP parameters vs expected'),
                                   verbose=True)


if __name__ == "__main__":
    icp_init_sim()
