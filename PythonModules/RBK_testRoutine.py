'''
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

'''
import sys, os, inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import numpy as np
import ctypes
import RigidBodyKinematics as rbk

R2D = 180.0 / np.pi  # [deg]
D2R = np.pi / 180.0  # [rad]
e_count = 0
a_tol = 1e-14


def equalCheck(v1, v2, methodName, e_count):
    if len(v1) == len(v2):
        if np.allclose(v1, v2, atol=a_tol):
            return
        print methodName + 'failed'
        e_count += 1
    else:
        print 'Mismatching lengths.'


v1 = np.array([0.45226701686665, 0.75377836144441, 0.15075567228888, 0.45226701686665])
v2 = np.array([-0.18663083698528, 0.46657709246321, 0.83983876643378, -0.20529392068381])
v_true = np.array([-0.46986547690254, -0.34044145332460, 0.71745926113861, 0.38545850500388])
v = rbk.addEP(v1, v2)
equalCheck(v, v_true, 'addEP', e_count)

v1 = np.array([10 * D2R, 20 * D2R, 30 * D2R])
v2 = np.array([-30 * D2R, 200 * D2R, 81 * D2R])
v_true = np.array([-2.96705972839036, 2.44346095279206, 1.41371669411541])
v = rbk.addEuler121(v1, v2)
equalCheck(v, v_true, 'addEuler121', e_count)
v = rbk.addEuler131(v1, v2)
equalCheck(v, v_true, 'addEuler131', e_count)
v = rbk.addEuler121(v1, v2)
equalCheck(v, v_true, 'addEuler121', e_count)
v = rbk.addEuler212(v1, v2)
equalCheck(v, v_true, 'addEuler212', e_count)
v = rbk.addEuler232(v1, v2)
equalCheck(v, v_true, 'addEuler232', e_count)
v = rbk.addEuler313(v1, v2)
equalCheck(v, v_true, 'addEuler313', e_count)
v = rbk.addEuler323(v1, v2)
equalCheck(v, v_true, 'addEuler323', e_count)
v_true = np.array([2.65556257351773, -0.34257634487528, -2.38843896474589])
v = rbk.addEuler123(v1, v2)
equalCheck(v, v_true, 'addEuler123', e_count)
v = rbk.addEuler231(v1, v2)
equalCheck(v, v_true, 'addEuler231', e_count)
v = rbk.addEuler312(v1, v2)
equalCheck(v, v_true, 'addEuler312', e_count)
v_true = np.array([2.93168877067466, -0.89056295435594, -2.11231276758895])
v = rbk.addEuler132(v1, v2)
equalCheck(v, v_true, 'addEuler132', e_count)
v = rbk.addEuler213(v1, v2)
equalCheck(v, v_true, 'addEuler213', e_count)
v = rbk.addEuler321(v1, v2)
equalCheck(v, v_true, 'addEuler321', e_count)

v1 = np.array([1.5, 0.5, 0.5])
v2 = np.array([-0.5, 0.25, 0.15])
v_true = np.array([0.58667769962764, -0.34919321472900, 0.43690525444766])
v = rbk.addMRP(v1, v2)
equalCheck(v, v_true, 'addMRP', e_count)

v_true = np.array([1.00227389370983, 0.41720669426711, 0.86837149207759])
v = rbk.addPRV(v1, v2)
equalCheck(v, v_true, 'addPRV', e_count)

v_true = np.array([0.61290322580645, 0.17741935483871, 0.82258064516129])
v = rbk.addGibbs(v1, v2)
equalCheck(v, v_true, 'addGibbs', e_count)

v = np.array([30 * D2R, -40 * D2R, 15 * D2R])
C = rbk.BinvEuler121(v)
C_true = np.array([
    [0.76604444311898, 0.0, 1.0],
    [-0.16636567534280, 0.96592582628907, 0.],
    [-0.62088515301485, -0.25881904510252, 0.]
])
equalCheck(C, C_true, 'BinvEuler121', e_count)

C = rbk.BinvEuler123(v)
C_true = np.array([
    [0.73994211169385, 0.25881904510252, 0.],
    [-0.19826689127415, 0.96592582628907, 0.],
    [-0.64278760968654, 0, 1.0]
])
equalCheck(C, C_true, 'BinvEuler123', e_count)

C = rbk.BinvEuler131(v)
C_true = np.array([
    [0.76604444311898, 0, 1.00000000000000],
    [0.62088515301485, 0.25881904510252, 0],
    [-0.16636567534280, 0.96592582628907, 0]
])
equalCheck(C, C_true, 'BinvEuler131', e_count)

C = rbk.BinvEuler132(v)
C_true = np.array([
    [0.73994211169385, -0.25881904510252, 0.],
    [0.64278760968654, 0., 1.00000000000000],
    [0.19826689127415, 0.96592582628907, 0.]
])
equalCheck(C, C_true, 'BinvEuler132', e_count)

C = rbk.BinvEuler212(v)
C_true = np.array([
    [-0.16636567534280, 0.96592582628907, 0],
    [0.76604444311898, 0, 1.0],
    [0.62088515301485, 0.25881904510252, 0]
])
equalCheck(C, C_true, 'BinvEuler212', e_count)

C = rbk.BinvEuler213(v)
C_true = np.array([
    [0.19826689127415, 0.96592582628907, 0],
    [0.73994211169385, -0.25881904510252, 0],
    [0.64278760968654, 0, 1.0]
])
equalCheck(C, C_true, 'BinvEuler213', e_count)

C = rbk.BinvEuler231(v)
C_true = np.array([
    [-0.64278760968654, 0, 1.00000000000000],
    [0.73994211169385, 0.25881904510252, 0],
    [-0.19826689127415, 0.96592582628907, 0]
])
equalCheck(C, C_true, 'BinvEuler231', e_count)

C = rbk.BinvEuler232(v)
C_true = np.array([
    [-0.62088515301485, -0.25881904510252, 0],
    [0.76604444311898, 0, 1.0],
    [-0.16636567534280, 0.96592582628907, 0]
])
equalCheck(C, C_true, 'BinvEuler232', e_count)

C = rbk.BinvEuler312(v)
C_true = np.array([
    [-0.19826689127415, 0.96592582628907, 0],
    [-0.64278760968654, 0, 1.0],
    [0.73994211169385, 0.25881904510252, 0]
])
equalCheck(C, C_true, 'BinvEuler312', e_count)

C = rbk.BinvEuler313(v)
C_true = np.array([
    [-0.16636567534280, 0.96592582628907, 0],
    [-0.62088515301485, -0.25881904510252, 0],
    [0.76604444311898, 0, 1.0]
])

equalCheck(C, C_true, 'BinvEuler313', e_count)

C = rbk.BinvEuler321(v)
C_true = np.array([
    [0.64278760968654, 0, 1.00000000000000],
    [0.19826689127415, 0.96592582628907, 0],
    [0.73994211169385, -0.25881904510252, 0]
])
equalCheck(C, C_true, 'BinvEuler321', e_count)

C = rbk.BinvEuler323(v)
C_true = np.array([
    [0.62088515301485, 0.25881904510252, 0],
    [-0.16636567534280, 0.96592582628907, 0],
    [0.76604444311898, 0, 1.0]
])
equalCheck(C, C_true, 'BinvEuler323', e_count)

v1 = np.array([0.2, -0.25, 0.3])
C = rbk.PRV2C(v1)
C_true = np.array([
    [0.9249653552860658, 0.2658656942983466, 0.2715778417245783],
    [-0.3150687400124018, 0.9360360405717283, 0.1567425271513747],
    [-0.212534186867712, -0.2305470957224576, 0.9495668781430935]
])
equalCheck(C, C_true, 'PRV2C', e_count)

v = rbk.PRV2EP(v1)
v_true = np.array([0.9760338459808767, 0.09919984446969178, -0.1239998055871147, 0.1487997667045377])
equalCheck(v, v_true, 'PRV2EP', e_count)

v1 = np.array([])
v2 = np.array([])
v_true = np.array([])
