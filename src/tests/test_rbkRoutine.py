#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#



import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

R2D = 180.0 / np.pi  # [deg]
D2R = np.pi / 180.0  # [rad]
a_tol = 1e-14


def equalCheck(v1, v2, methodName):
    if len(v1) == len(v2):
        if np.allclose(v1, v2, atol=a_tol):
            return 0
        print(methodName + ' failed')
    else:
        print('Mismatching lengths.')
    return 1

def test_rigidBodyKinematics(show_plots):
    e_count = 0

    v1 = np.array([0.45226701686665, 0.75377836144441, 0.15075567228888, 0.45226701686665])
    v2 = np.array([-0.18663083698528, 0.46657709246321, 0.83983876643378, -0.20529392068381])
    v_true = np.array([-0.46986547690254, -0.34044145332460, 0.71745926113861, 0.38545850500388])
    v = rbk.addEP(v1, v2)
    e_count += equalCheck(v, v_true, 'addEP')

    v1 = np.array([10 * D2R, 20 * D2R, 30 * D2R])
    v2 = np.array([-30 * D2R, 200 * D2R, 81 * D2R])
    v_true = np.array([-2.96705972839036, 2.44346095279206, 1.41371669411541])
    v = rbk.addEuler121(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler121')
    v = rbk.addEuler131(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler131')
    v = rbk.addEuler121(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler121')
    v = rbk.addEuler212(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler212')
    v = rbk.addEuler232(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler232')
    v = rbk.addEuler313(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler313')
    v = rbk.addEuler323(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler323')
    v_true = np.array([2.65556257351773, -0.34257634487528, -2.38843896474589])
    v = rbk.addEuler123(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler123')
    v = rbk.addEuler231(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler231')
    v = rbk.addEuler312(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler312')
    v_true = np.array([2.93168877067466, -0.89056295435594, -2.11231276758895])
    v = rbk.addEuler132(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler132')
    v = rbk.addEuler213(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler213')
    v = rbk.addEuler321(v1, v2)
    e_count += equalCheck(v, v_true, 'addEuler321')

    v1 = np.array([1.5, 0.5, 0.5])
    v2 = np.array([-0.5, 0.25, 0.15])
    v_true = np.array([0.58667769962764, -0.34919321472900, 0.43690525444766])
    v = rbk.addMRP(v1, v2)
    e_count += equalCheck(v, v_true, 'addMRP')

    v_true = np.array([1.00227389370983, 0.41720669426711, 0.86837149207759])
    v = rbk.addPRV(v1, v2)
    e_count += equalCheck(v, v_true, 'addPRV')

    v_true = np.array([0.61290322580645, 0.17741935483871, 0.82258064516129])
    v = rbk.addGibbs(v1, v2)
    e_count += equalCheck(v, v_true, 'addGibbs')

    v = np.array([30 * D2R, -40 * D2R, 15 * D2R])
    C = rbk.BmatEuler121(v)
    C_true = np.array([
        [0, -0.40265095531125, -1.50271382293774],
        [0, 0.96592582628907, -0.25881904510252],
        [1.00000000000000, 0.30844852683273, 1.15114557365953]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler121')
    C = rbk.BmatEuler123(v)
    C_true = np.array([
        [1.26092661459205, -0.33786426809485, 0],
        [0.25881904510252, 0.96592582628907, 0],
        [0.81050800458377, -0.21717496528718, 1.00000000000000]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler123')
    C = rbk.BmatEuler131(v)
    C_true = np.array([
        [0, 1.50271382293774, -0.40265095531125],
        [0, 0.25881904510252, 0.96592582628907],
        [1.00000000000000, -1.15114557365953, 0.30844852683273, ]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler131')
    C = rbk.BmatEuler132(v)
    C_true = np.array([
        [1.26092661459205, 0, 0.33786426809485],
        [-0.25881904510252, 0, 0.96592582628907],
        [-0.81050800458377, 1.00000000000000, -0.21717496528718]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler132')
    C = rbk.BmatEuler212(v)
    C_true = np.array([
        [-0.40265095531125, 0, 1.50271382293774],
        [0.96592582628907, 0, 0.25881904510252],
        [0.30844852683273, 1.00000000000000, -1.15114557365953]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler212')
    C = rbk.BmatEuler213(v)
    C_true = np.array([
        [0.33786426809485, 1.26092661459205, 0],
        [0.96592582628907, -0.25881904510252, 0],
        [-0.21717496528718, -0.81050800458377, 1.00000000000000]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler213')
    C = rbk.BmatEuler231(v)
    C_true = np.array([
        [0, 1.26092661459205, -0.33786426809485],
        [0, 0.25881904510252, 0.96592582628907],
        [1.00000000000000, 0.81050800458377, -0.21717496528718]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler231')
    C = rbk.BmatEuler232(v)
    C_true = np.array([
        [-1.50271382293774, 0, -0.40265095531125],
        [-0.25881904510252, 0, 0.96592582628907],
        [1.15114557365953, 1.00000000000000, 0.30844852683273]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler232')
    C = rbk.BmatEuler312(v)
    C_true = np.array([
        [-0.33786426809485, 0, 1.26092661459205],
        [0.96592582628907, 0, 0.25881904510252],
        [-0.21717496528718, 1.00000000000000, 0.81050800458377]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler312')
    C = rbk.BmatEuler313(v)
    C_true = np.array([
        [-0.40265095531125, -1.50271382293774, 0],
        [0.96592582628907, -0.25881904510252, 0],
        [0.30844852683273, 1.15114557365953, 1.00000000000000]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler313')
    C = rbk.BmatEuler321(v)
    C_true = np.array([
        [0, 0.33786426809485, 1.26092661459205],
        [0, 0.96592582628907, -0.25881904510252],
        [1.00000000000000, -0.21717496528718, -0.81050800458377]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler321')
    C = rbk.BmatEuler323(v)
    C_true = np.array([
        [1.50271382293774, -0.40265095531125, 0],
        [0.25881904510252, 0.96592582628907, 0],
        [-1.15114557365953, 0.30844852683273, 1.00000000000000]
    ])
    e_count += equalCheck(C, C_true, 'BmatEuler323')

    C = rbk.BinvEuler121(v)
    C_true = np.array([
        [0.76604444311898, 0.0, 1.0],
        [-0.16636567534280, 0.96592582628907, 0.],
        [-0.62088515301485, -0.25881904510252, 0.]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler121')

    C = rbk.BinvEuler123(v)
    C_true = np.array([
        [0.73994211169385, 0.25881904510252, 0.],
        [-0.19826689127415, 0.96592582628907, 0.],
        [-0.64278760968654, 0, 1.0]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler123')

    C = rbk.BinvEuler131(v)
    C_true = np.array([
        [0.76604444311898, 0, 1.00000000000000],
        [0.62088515301485, 0.25881904510252, 0],
        [-0.16636567534280, 0.96592582628907, 0]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler131')

    C = rbk.BinvEuler132(v)
    C_true = np.array([
        [0.73994211169385, -0.25881904510252, 0.],
        [0.64278760968654, 0., 1.00000000000000],
        [0.19826689127415, 0.96592582628907, 0.]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler132')

    C = rbk.BinvEuler212(v)
    C_true = np.array([
        [-0.16636567534280, 0.96592582628907, 0],
        [0.76604444311898, 0, 1.0],
        [0.62088515301485, 0.25881904510252, 0]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler212')

    C = rbk.BinvEuler213(v)
    C_true = np.array([
        [0.19826689127415, 0.96592582628907, 0],
        [0.73994211169385, -0.25881904510252, 0],
        [0.64278760968654, 0, 1.0]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler213')

    C = rbk.BinvEuler231(v)
    C_true = np.array([
        [-0.64278760968654, 0, 1.00000000000000],
        [0.73994211169385, 0.25881904510252, 0],
        [-0.19826689127415, 0.96592582628907, 0]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler231')

    C = rbk.BinvEuler232(v)
    C_true = np.array([
        [-0.62088515301485, -0.25881904510252, 0],
        [0.76604444311898, 0, 1.0],
        [-0.16636567534280, 0.96592582628907, 0]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler232')

    C = rbk.BinvEuler312(v)
    C_true = np.array([
        [-0.19826689127415, 0.96592582628907, 0],
        [-0.64278760968654, 0, 1.0],
        [0.73994211169385, 0.25881904510252, 0]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler312')

    C = rbk.BinvEuler313(v)
    C_true = np.array([
        [-0.16636567534280, 0.96592582628907, 0],
        [-0.62088515301485, -0.25881904510252, 0],
        [0.76604444311898, 0, 1.0]
    ])

    e_count += equalCheck(C, C_true, 'BinvEuler313')

    C = rbk.BinvEuler321(v)
    C_true = np.array([
        [0.64278760968654, 0, 1.00000000000000],
        [0.19826689127415, 0.96592582628907, 0],
        [0.73994211169385, -0.25881904510252, 0]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler321')

    C = rbk.BinvEuler323(v)
    C_true = np.array([
        [0.62088515301485, 0.25881904510252, 0],
        [-0.16636567534280, 0.96592582628907, 0],
        [0.76604444311898, 0, 1.0]
    ])
    e_count += equalCheck(C, C_true, 'BinvEuler323')

    v = np.array([0.25, 0.5, -0.5])
    C = rbk.BinvGibbs(v)
    C_true = np.array([
        [0.64, -0.32, -0.32],
        [0.32, 0.64, 0.16],
        [0.32, -0.16, 0.64]
    ])
    e_count += equalCheck(C, C_true, 'BinvGibbs')

    v = np.array([0.25, 0.5, -0.5])
    C = rbk.BinvMRP(v)
    C_true = np.array([
        [0.2304, -0.3072, -0.512],
        [0.512, 0.384, 0],
        [0.3072, -0.4096, 0.3840]
    ])
    e_count += equalCheck(C, C_true, 'BinvMRP')
    C = rbk.BinvPRV(v)
    C_true = np.array([
        [0.91897927113877, -0.21824360100796, -0.25875396543858],
        [0.25875396543858, 0.94936204446173, 0.07873902718102],
        [0.21824360100796, -0.15975975604225, 0.94936204446173]
    ])
    e_count += equalCheck(C, C_true, 'BinvPRV')
    C = rbk.BmatGibbs(v)
    C_true = np.array([
        [1.06250000000000, 0.62500000000000, 0.37500000000000],
        [-0.37500000000000, 1.25000000000000, -0.50000000000000],
        [-0.62500000000000, 0, 1.25000000000000]
    ])
    e_count += equalCheck(C, C_true, 'BmatGibbs')
    C = rbk.BmatMRP(v)
    C_true = np.array([
        [0.56250000000000, 1.25000000000000, 0.75000000000000],
        [-0.75000000000000, 0.93750000000000, -1.00000000000000],
        [-1.25000000000000, 0, 0.93750000000000]
    ])
    e_count += equalCheck(C, C_true, 'BmatMRP')
    C = rbk.BmatPRV(v)
    C_true = np.array([
        [0.95793740211924, 0.26051564947019, 0.23948435052981],
        [-0.23948435052981, 0.97371087632453, -0.14603129894038],
        [-0.26051564947019, 0.10396870105962, 0.97371087632453]
    ])
    e_count += equalCheck(C, C_true, 'BmatPRV')

    C = np.array([
        [-0.506611258027956, -0.05213449187759728, 0.860596902153381],
        [-0.7789950887797505, -0.4000755572346052, -0.4828107291273137],
        [0.3694748772194938, -0.9149981110691346, 0.1620702682281828]
    ])
    v = rbk.C2EP(C)
    v_true = np.array([0.2526773896521122, 0.4276078901804977, -0.4859180570232927, \
                       0.7191587243944733])
    e_count += equalCheck(v, v_true, 'C2EP')
    v = rbk.C2Euler121(C)
    v_true = np.array([-3.081087141428621, 2.102046098550739, -1.127921895439695])
    e_count += equalCheck(v, v_true, 'C2Euler121')
    v = rbk.C2Euler123(C)
    v_true = np.array([1.395488250243478, 0.3784438476398376, 2.147410157986089])
    e_count += equalCheck(v, v_true, 'C2Euler123')
    v = rbk.C2Euler131(C)
    v_true = np.array([1.631301838956069, 2.102046098550739, 0.4428744313552013])
    e_count += equalCheck(v, v_true, 'C2Euler131')
    v = rbk.C2Euler132(C)
    v_true = np.array([-2.262757475208626, 0.8930615653924096, 2.511467464302149])
    e_count += equalCheck(v, v_true, 'C2Euler132')
    v = rbk.C2Euler212(C)
    v_true = np.array([-2.125637903992466, 1.982395614047245, -0.05691616561213509])
    e_count += equalCheck(v, v_true, 'C2Euler212')
    v = rbk.C2Euler213(C)
    v_true = np.array([1.157420789791818, 1.155503238813826, -3.012011225795042])
    e_count += equalCheck(v, v_true, 'C2Euler213')
    v = rbk.C2Euler231(C)
    v_true = np.array([-2.102846464319881, -0.05215813778076988, 1.982990154077466])
    e_count += equalCheck(v, v_true, 'C2Euler231')
    v = rbk.C2Euler232(C)
    v_true = np.array([-0.5548415771975691, 1.982395614047245, -1.627712492407032])
    e_count += equalCheck(v, v_true, 'C2Euler232')
    v = rbk.C2Euler312(C)
    v_true = np.array([2.045248068737305, -0.5038614866151004, -1.384653359078797])
    e_count += equalCheck(v, v_true, 'C2Euler312')
    v = rbk.C2Euler313(C)
    v_true = np.array([0.3837766626244829, 1.408008028147626, 2.082059614484753])
    e_count += equalCheck(v, v_true, 'C2Euler313')
    v = rbk.C2Euler321(C)
    v_true = np.array([-3.039045355374235, -1.036440549977791, -1.246934586231547])
    e_count += equalCheck(v, v_true, 'C2Euler321')
    v = rbk.C2Euler323(C)
    v_true = np.array([-1.187019664170414, 1.408008028147626, -2.630329365899936])
    e_count += equalCheck(v, v_true, 'C2Euler323')
    v = rbk.C2Gibbs(C)
    v_true = np.array([1.692307692307693, -1.923076923076923, 2.846153846153846])
    e_count += equalCheck(v, v_true, 'C2Gibbs')
    v = rbk.C2MRP(C)
    v_true = np.array([0.3413551595269481, -0.3879035903715318, 0.5740973137498672])
    e_count += equalCheck(v, v_true, 'C2MRP')
    v = rbk.C2PRV(C)
    v_true = np.array([1.162634795241009, -1.321175903682964, 1.955340337450788])
    e_count += equalCheck(v, v_true, 'C2PRV')

    q = np.array([0.2526773896521122, 0.4276078901804977, -0.4859180570232927, 0.7191587243944733])
    w = np.array([0.2, 0.1, -0.5])
    v = rbk.dEP(q, w)
    v_true = np.array([0.1613247949317332, 0.1107893170013107, 0.1914517144671774, 0.006802852798326098])
    e_count += equalCheck(v, v_true, 'dEP')

    q = np.array([30 * D2R, -40 * D2R, 15 * D2R])
    v = rbk.dEuler121(q, w)
    v_true = np.array([0.7110918159377425, 0.2260021051801672, -0.3447279341464908])
    e_count += equalCheck(v, v_true, 'dEuler121')
    v = rbk.dEuler123(q, w)
    v_true = np.array([0.2183988961089258, 0.148356391649411, -0.3596158956119647])
    e_count += equalCheck(v, v_true, 'dEuler123')
    v = rbk.dEuler131(q, w)
    v_true = np.array([0.3515968599493992, -0.4570810086342821, -0.06933882078231876])
    e_count += equalCheck(v, v_true, 'dEuler131')
    v = rbk.dEuler132(q, w)
    v_true = np.array([0.08325318887098565, -0.5347267221650382, 0.04648588172683711])
    e_count += equalCheck(v, v_true, 'dEuler132')
    v = rbk.dEuler212(q, w)
    v_true = np.array([-0.8318871025311179, 0.06377564270655334, 0.7372624921963103])
    e_count += equalCheck(v, v_true, 'dEuler212')
    v = rbk.dEuler213(q, w)
    v_true = np.array([0.1936655150781755, 0.1673032607475616, -0.6244857935158128])
    e_count += equalCheck(v, v_true, 'dEuler213')
    v = rbk.dEuler231(q, w)
    v_true = np.array([0.2950247955066306, -0.4570810086342821, 0.3896382831019671])
    e_count += equalCheck(v, v_true, 'dEuler231')
    v = rbk.dEuler232(q, w)
    v_true = np.array([-0.09921728693192147, -0.5347267221650384, 0.1760048513155397])
    e_count += equalCheck(v, v_true, 'dEuler232')
    v = rbk.dEuler312(q, w)
    v_true = np.array([-0.6980361609149971, 0.06377564270655331, -0.3486889953493196])
    e_count += equalCheck(v, v_true, 'dEuler312')
    v = rbk.dEuler313(q, w)
    v_true = np.array([-0.2308015733560238, 0.1673032607475616, -0.3231957372675008])
    e_count += equalCheck(v, v_true, 'dEuler313')
    v = rbk.dEuler321(q, w)
    v_true = np.array([-0.596676880486542, 0.2260021051801672, 0.5835365057631652])
    e_count += equalCheck(v, v_true, 'dEuler321')
    v = rbk.dEuler323(q, w)
    v_true = np.array([0.260277669056422, 0.148356391649411, -0.6993842620486324])
    e_count += equalCheck(v, v_true, 'dEuler323')
    v = rbk.dGibbs(q, w)
    v_true = np.array([0.236312018677072, 0.2405875488560276, -0.1665723597065136])
    e_count += equalCheck(v, v_true, 'dGibbs')
    v = rbk.dMRP(q, w)
    v_true = np.array([0.144807895231133, 0.1948354871330581, 0.062187948908334])
    e_count += equalCheck(v, v_true, 'dMRP')
    v = rbk.dPRV(q, w)
    v_true = np.array([0.34316538031149, 0.255728121815202, -0.3710557691157747])
    e_count += equalCheck(v, v_true, 'dPRV')

    q = np.array([0.9110886174894189, 0.5746957711326909, -0.7662610281769212, 0.2873478855663454])
    v = rbk.elem2PRV(q)
    v_true = np.array([0.5235987755982988, -0.6981317007977318, 0.2617993877991494])
    e_count += equalCheck(v, v_true, 'elem2PRV')

    q = np.array([0.2526773896521122, 0.4276078901804977, -0.4859180570232927, 0.7191587243944733])
    C = rbk.EP2C(q)
    C_true = np.array([
        [-0.506611258027956, -0.05213449187759728, 0.860596902153381],
        [-0.7789950887797505, -0.4000755572346052, -0.4828107291273137],
        [0.3694748772194938, -0.9149981110691346, 0.1620702682281828]
    ])
    e_count += equalCheck(C, C_true, 'EP2C')
    v = rbk.EP2Euler121(q)
    v_true = np.array([3.202098165750965, 2.102046098550739, -1.127921895439695])
    e_count += equalCheck(v, v_true, 'EP2Euler121')
    v = rbk.EP2Euler123(q)
    v_true = np.array([1.395488250243478, 0.3784438476398376, 2.147410157986089])
    e_count += equalCheck(v, v_true, 'EP2Euler123')
    v = rbk.EP2Euler131(q)
    v_true = np.array([1.631301838956069, 2.102046098550739, 0.4428744313552013])
    e_count += equalCheck(v, v_true, 'EP2Euler131')
    v = rbk.EP2Euler132(q)
    v_true = np.array([-2.262757475208626, 0.8930615653924096, 2.511467464302149])
    e_count += equalCheck(v, v_true, 'EP2Euler132')
    v = rbk.EP2Euler212(q)
    v_true = np.array([-2.125637903992466, 1.982395614047245, -0.05691616561213508])
    e_count += equalCheck(v, v_true, 'EP2Euler212')
    v = rbk.EP2Euler213(q)
    v_true = np.array([1.157420789791818, 1.155503238813826, -3.012011225795042])
    e_count += equalCheck(v, v_true, 'EP2Euler213')
    v = rbk.EP2Euler231(q)
    v_true = np.array([-2.102846464319881, -0.05215813778076988, 1.982990154077466])
    e_count += equalCheck(v, v_true, 'EP2Euler231')
    v = rbk.EP2Euler232(q)
    v_true = np.array([-0.5548415771975691, 1.982395614047245, -1.627712492407032])
    e_count += equalCheck(v, v_true, 'EP2Euler232')
    v = rbk.EP2Euler312(q)
    v_true = np.array([2.045248068737305, -0.5038614866151004, -1.384653359078797])
    e_count += equalCheck(v, v_true, 'EP2Euler312')
    v = rbk.EP2Euler313(q)
    v_true = np.array([0.3837766626244828, 1.408008028147627, 2.082059614484753])
    e_count += equalCheck(v, v_true, 'EP2Euler313')
    v = rbk.EP2Euler321(q)
    v_true = np.array([-3.039045355374235, -1.036440549977791, -1.246934586231547])
    e_count += equalCheck(v, v_true, 'EP2Euler321')
    v = rbk.EP2Euler323(q)
    v_true = np.array([-1.187019664170414, 1.408008028147627, 3.65285594127965])
    e_count += equalCheck(v, v_true, 'EP2Euler323')
    v = rbk.EP2Gibbs(q)
    v_true = np.array([1.692307692307693, -1.923076923076923, 2.846153846153846])
    e_count += equalCheck(v, v_true, 'EP2Gibbs')
    v = rbk.EP2MRP(q)
    v_true = np.array([0.3413551595269481, -0.3879035903715319, 0.5740973137498672])
    e_count += equalCheck(v, v_true, 'EP2MRP')
    v = rbk.EP2PRV(q)
    v_true = np.array([1.162634795241009, -1.321175903682965, 1.955340337450788])
    e_count += equalCheck(v, v_true, 'EP2PRV')

    x = 1.3
    C = rbk.euler1(x)
    C_true = np.array([
        [1., 0., 0.],
        [0., 0.2674988286245874, 0.963558185417193],
        [0., -0.963558185417193, 0.2674988286245874]
    ])
    e_count += equalCheck(C, C_true, 'euler1')
    C = rbk.euler2(x)
    C_true = np.array([
        [0.2674988286245874, 0, -0.963558185417193],
        [0., 1., 0.],
        [0.963558185417193, 0, 0.2674988286245874]
    ])
    e_count += equalCheck(C, C_true, 'euler2')
    C = rbk.euler3(x)
    C_true = np.array([
        [0.2674988286245874, 0.963558185417193, 0],
        [-0.963558185417193, 0.2674988286245874, 0],
        [0, 0, 1]
    ])
    e_count += equalCheck(C, C_true, 'euler3')

    e = np.array([0.5746957711326909, -0.7662610281769212, 0.2873478855663454])
    C = rbk.euler1212C(e)
    C_true = np.array([
        [0.7205084754311385, -0.3769430728235922, 0.5820493593177511],
        [-0.1965294640304305, 0.6939446195986547, 0.692688266609151],
        [-0.6650140649638986, -0.6134776155495705, 0.4259125598286639]
    ])
    e_count += equalCheck(C, C_true, 'euler1212C')
    v = rbk.euler1212EP(e)
    v_true = np.array([0.8426692196316502, 0.3875084824890354, -0.3699741829975614, -0.05352444488005169])
    e_count += equalCheck(v, v_true, 'euler1212EP')
    v = rbk.euler1212Gibbs(e)
    v_true = np.array([0.4598583565902931, -0.4390503110571495, -0.06351774057138154])
    e_count += equalCheck(v, v_true, 'euler1212Gibbs')
    v = rbk.euler1212MRP(e)
    v_true = np.array([0.2102973655610845, -0.2007816590497557, -0.02904723447366817])
    e_count += equalCheck(v, v_true, 'euler1212MRP')
    v = rbk.euler1212PRV(e)
    v_true = np.array([0.8184049632304388, -0.7813731087574279, -0.1130418386266624])
    e_count += equalCheck(v, v_true, 'euler1212PRV')

    C = rbk.euler1232C(e)
    C_true = np.array([
        [0.6909668228739537, -0.1236057418710468, 0.7122404581768593],
        [-0.2041991989591971, 0.9117724894309838, 0.3563335721781613],
        [-0.6934461311680212, -0.391653607277317, 0.6047643467291773]
    ])
    e_count += equalCheck(C, C_true, 'euler1232C')
    v = rbk.euler1232EP(e)
    v_true = np.array([0.8954752451958283, 0.2088240806958052, -0.3924414987701519, 0.02250019124496444])
    e_count += equalCheck(v, v_true, 'euler1232EP')
    v = rbk.euler1232Gibbs(e)
    v_true = np.array([0.2331991663824702, -0.4382494109977661, 0.02512653628972619])
    e_count += equalCheck(v, v_true, 'euler1232Gibbs')
    v = rbk.euler1232MRP(e)
    v_true = np.array([0.1101697746911123, -0.2070412155288303, 0.01187047485953311])
    e_count += equalCheck(v, v_true, 'euler1232MRP')
    v = rbk.euler1232PRV(e)
    v_true = np.array([0.4328366663508259, -0.8134266388215754, 0.04663690000825693])
    e_count += equalCheck(v, v_true, 'euler1232PRV')

    C = rbk.euler1312C(e)
    C_true = np.array([
        [0.7205084754311385, -0.5820493593177511, -0.3769430728235922],
        [0.6650140649638986, 0.4259125598286639, 0.6134776155495705],
        [-0.1965294640304305, -0.692688266609151, 0.6939446195986547]
    ])
    e_count += equalCheck(C, C_true, 'euler1312C')
    v = rbk.euler1312EP(e)
    v_true = np.array([0.8426692196316502, 0.3875084824890354, 0.05352444488005169, -0.3699741829975614])
    e_count += equalCheck(v, v_true, 'euler1312EP')
    v = rbk.euler1312Gibbs(e)
    v_true = np.array([0.4598583565902931, 0.06351774057138154, -0.4390503110571495])
    e_count += equalCheck(v, v_true, 'euler1312Gibbs')
    v = rbk.euler1312MRP(e)
    v_true = np.array([0.2102973655610845, 0.02904723447366817, -0.2007816590497557])
    e_count += equalCheck(v, v_true, 'euler1312MRP')
    v = rbk.euler1312PRV(e)
    v_true = np.array([0.8184049632304388, 0.1130418386266624, -0.7813731087574279])
    e_count += equalCheck(v, v_true, 'euler1312PRV')

    C = rbk.euler1322C(e)
    C_true = np.array([
        [0.6909668228739537, -0.404128912281835, -0.5993702294453531],
        [0.6934461311680212, 0.6047643467291773, 0.391653607277317],
        [0.2041991989591971, -0.6862506154337003, 0.6981137299618809]
    ])
    e_count += equalCheck(C, C_true, 'euler1322C')
    v = rbk.euler1322EP(e)
    v_true = np.array([0.8651365354042408, 0.3114838463640192, 0.2322088466732818, -0.3171681574333834])
    e_count += equalCheck(v, v_true, 'euler1322EP')
    v = rbk.euler1322Gibbs(e)
    v_true = np.array([0.3600401018996109, 0.2684071671586273, -0.3666105226791566])
    e_count += equalCheck(v, v_true, 'euler1322Gibbs')
    v = rbk.euler1322MRP(e)
    v_true = np.array([0.1670032410235906, 0.1244996504360223, -0.1700509058789317])
    e_count += equalCheck(v, v_true, 'euler1322MRP')
    v = rbk.euler1322PRV(e)
    v_true = np.array([0.6525765328552258, 0.4864908592507521, -0.6644854907437873])
    e_count += equalCheck(v, v_true, 'euler1322PRV')

    C = rbk.euler2122C(e)
    C_true = np.array([
        [0.6939446195986547, -0.1965294640304305, -0.692688266609151],
        [-0.3769430728235922, 0.7205084754311385, -0.5820493593177511],
        [0.6134776155495705, 0.6650140649638986, 0.4259125598286639]
    ])
    e_count += equalCheck(C, C_true, 'euler2122C')
    v = rbk.euler2122EP(e)
    v_true = np.array([0.8426692196316502, -0.3699741829975614, 0.3875084824890354, 0.05352444488005169])
    e_count += equalCheck(v, v_true, 'euler2122EP')
    v = rbk.euler2122Gibbs(e)
    v_true = np.array([-0.4390503110571495, 0.4598583565902931, 0.06351774057138154])
    e_count += equalCheck(v, v_true, 'euler2122Gibbs')
    v = rbk.euler2122MRP(e)
    v_true = np.array([-0.2007816590497557, 0.2102973655610845, 0.02904723447366817])
    e_count += equalCheck(v, v_true, 'euler2122MRP')
    v = rbk.euler2122PRV(e)
    v_true = np.array([-0.7813731087574279, 0.8184049632304388, 0.1130418386266624])
    e_count += equalCheck(v, v_true, 'euler2122PRV')

    C = rbk.euler2132C(e)
    C_true = np.array([
        [0.6981137299618809, 0.2041991989591971, -0.6862506154337003],
        [-0.5993702294453531, 0.6909668228739537, -0.404128912281835],
        [0.391653607277317, 0.6934461311680212, 0.6047643467291773]
    ])
    e_count += equalCheck(C, C_true, 'euler2132C')
    v = rbk.euler2132EP(e)
    v_true = np.array([0.8651365354042408, -0.3171681574333834, 0.3114838463640192, 0.2322088466732818])
    e_count += equalCheck(v, v_true, 'euler2132EP')
    v = rbk.euler2132Gibbs(e)
    v_true = np.array([-0.3666105226791566, 0.3600401018996109, 0.2684071671586273])
    e_count += equalCheck(v, v_true, 'euler2132Gibbs')
    v = rbk.euler2132MRP(e)
    v_true = np.array([-0.1700509058789317, 0.1670032410235906, 0.1244996504360223])
    e_count += equalCheck(v, v_true, 'euler2132MRP')
    v = rbk.euler2132PRV(e)
    v_true = np.array([-0.6644854907437873, 0.6525765328552258, 0.4864908592507521])
    e_count += equalCheck(v, v_true, 'euler2132PRV')

    C = rbk.euler2312C(e)
    C_true = np.array([
        [0.6047643467291773, -0.6934461311680212, -0.391653607277317],
        [0.7122404581768593, 0.6909668228739537, -0.1236057418710468],
        [0.3563335721781613, -0.2041991989591971, 0.9117724894309838]
    ])
    e_count += equalCheck(C, C_true, 'euler2312C')
    v = rbk.euler2312EP(e)
    v_true = np.array([0.8954752451958283, 0.02250019124496444, 0.2088240806958052, -0.3924414987701519])
    e_count += equalCheck(v, v_true, 'euler2312EP')
    v = rbk.euler2312Gibbs(e)
    v_true = np.array([0.02512653628972619, 0.2331991663824702, -0.4382494109977661])
    e_count += equalCheck(v, v_true, 'euler2312Gibbs')
    v = rbk.euler2312MRP(e)
    v_true = np.array([0.01187047485953311, 0.1101697746911123, -0.2070412155288303])
    e_count += equalCheck(v, v_true, 'euler2312MRP')
    v = rbk.euler2312PRV(e)
    v_true = np.array([0.04663690000825693, 0.4328366663508259, -0.8134266388215754])
    e_count += equalCheck(v, v_true, 'euler2312PRV')

    C = rbk.euler2322C(e)
    C_true = np.array([
        [0.4259125598286639, -0.6650140649638986, -0.6134776155495705],
        [0.5820493593177511, 0.7205084754311385, -0.3769430728235922],
        [0.692688266609151, -0.1965294640304305, 0.6939446195986547]
    ])
    e_count += equalCheck(C, C_true, 'euler2322C')
    v = rbk.euler2322EP(e)
    v_true = np.array([0.8426692196316502, -0.05352444488005169, 0.3875084824890354, -0.3699741829975614])
    e_count += equalCheck(v, v_true, 'euler2322EP')
    v = rbk.euler2322Gibbs(e)
    v_true = np.array([-0.06351774057138154, 0.4598583565902931, -0.4390503110571495])
    e_count += equalCheck(v, v_true, 'euler2322Gibbs')
    v = rbk.euler2322MRP(e)
    v_true = np.array([-0.02904723447366817, 0.2102973655610845, -0.2007816590497557])
    e_count += equalCheck(v, v_true, 'euler2322MRP')
    v = rbk.euler2322PRV(e)
    v_true = np.array([-0.1130418386266624, 0.8184049632304388, -0.7813731087574279])
    e_count += equalCheck(v, v_true, 'euler2322PRV')

    C = rbk.euler3122C(e)
    C_true = np.array([
        [0.9117724894309838, 0.3563335721781613, -0.2041991989591971],
        [-0.391653607277317, 0.6047643467291773, -0.6934461311680212],
        [-0.1236057418710468, 0.7122404581768593, 0.6909668228739537]
    ])
    e_count += equalCheck(C, C_true, 'euler3122C')
    v = rbk.euler3122EP(e)
    v_true = np.array([0.8954752451958283, -0.3924414987701519, 0.02250019124496444, 0.2088240806958052])
    e_count += equalCheck(v, v_true, 'euler3122EP')
    v = rbk.euler3122Gibbs(e)
    v_true = np.array([-0.4382494109977661, 0.02512653628972619, 0.2331991663824702])
    e_count += equalCheck(v, v_true, 'euler3122Gibbs')
    v = rbk.euler3122MRP(e)
    v_true = np.array([-0.2070412155288303, 0.01187047485953311, 0.1101697746911123])
    e_count += equalCheck(v, v_true, 'euler3122MRP')
    v = rbk.euler3122PRV(e)
    v_true = np.array([-0.8134266388215754, 0.04663690000825693, 0.4328366663508259])
    e_count += equalCheck(v, v_true, 'euler3122PRV')

    C = rbk.euler3132C(e)
    C_true = np.array([
        [0.6939446195986547, 0.692688266609151, -0.1965294640304305],
        [-0.6134776155495705, 0.4259125598286639, -0.6650140649638986],
        [-0.3769430728235922, 0.5820493593177511, 0.7205084754311385]
    ])
    e_count += equalCheck(C, C_true, 'euler3132C')
    v = rbk.euler3132EP(e)
    v_true = np.array([0.8426692196316502, -0.3699741829975614, -0.05352444488005169, 0.3875084824890354])
    e_count += equalCheck(v, v_true, 'euler3132EP')
    v = rbk.euler3132Gibbs(e)
    v_true = np.array([-0.4390503110571495, -0.06351774057138154, 0.4598583565902931])
    e_count += equalCheck(v, v_true, 'euler3132Gibbs')
    v = rbk.euler3132MRP(e)
    v_true = np.array([-0.2007816590497557, -0.02904723447366817, 0.2102973655610845])
    e_count += equalCheck(v, v_true, 'euler3132MRP')
    v = rbk.euler3132PRV(e)
    v_true = np.array([-0.7813731087574279, -0.1130418386266624, 0.8184049632304388])
    e_count += equalCheck(v, v_true, 'euler3132PRV')

    C = rbk.euler3212C(e)
    C_true = np.array([
        [0.6047643467291773, 0.391653607277317, 0.6934461311680212],
        [-0.6862506154337003, 0.6981137299618809, 0.2041991989591971],
        [-0.404128912281835, -0.5993702294453531, 0.6909668228739537]
    ])
    e_count += equalCheck(C, C_true, 'euler3212C')
    v = rbk.euler3212EP(e)
    v_true = np.array([0.8651365354042408, 0.2322088466732818, -0.3171681574333834, 0.3114838463640192])
    e_count += equalCheck(v, v_true, 'euler3212EP')
    v = rbk.euler3212Gibbs(e)
    v_true = np.array([0.2684071671586273, -0.3666105226791566, 0.3600401018996109])
    e_count += equalCheck(v, v_true, 'euler3212Gibbs')
    v = rbk.euler3212MRP(e)
    v_true = np.array([0.1244996504360223, -0.1700509058789317, 0.1670032410235906])
    e_count += equalCheck(v, v_true, 'euler3212MRP')
    v = rbk.euler3212PRV(e)
    v_true = np.array([0.4864908592507521, -0.6644854907437873, 0.6525765328552258])
    e_count += equalCheck(v, v_true, 'euler3212PRV')

    C = rbk.euler3232C(e)
    C_true = np.array([
        [0.4259125598286639, 0.6134776155495705, 0.6650140649638986],
        [-0.692688266609151, 0.6939446195986547, -0.1965294640304305],
        [-0.5820493593177511, -0.3769430728235922, 0.7205084754311385]
    ])
    e_count += equalCheck(C, C_true, 'euler3232C')
    v = rbk.euler3232EP(e)
    v_true = np.array([0.8426692196316502, 0.05352444488005169, -0.3699741829975614, 0.3875084824890354])
    e_count += equalCheck(v, v_true, 'euler3232EP')
    v = rbk.euler3232Gibbs(e)
    v_true = np.array([0.06351774057138154, -0.4390503110571495, 0.4598583565902931])
    e_count += equalCheck(v, v_true, 'euler3232Gibbs')
    v = rbk.euler3232MRP(e)
    v_true = np.array([0.02904723447366817, -0.2007816590497557, 0.2102973655610845])
    e_count += equalCheck(v, v_true, 'euler3232MRP')
    v = rbk.euler3232PRV(e)
    v_true = np.array([0.1130418386266624, -0.7813731087574279, 0.8184049632304388])
    e_count += equalCheck(v, v_true, 'euler3232PRV')

    q = np.array([0.5746957711326909, -0.7662610281769212, 0.2873478855663454])
    C = rbk.gibbs2C(q)
    C_true = np.array([
        [0.3302752293577981, -0.1530190869107189, 0.9313986428558203],
        [-0.7277148580434096, 0.5871559633027522, 0.3545122848941588],
        [-0.6011234134980221, -0.794879257371223, 0.08256880733944938]
    ])
    e_count += equalCheck(C, C_true, 'gibbs2C')
    v = rbk.gibbs2EP(q)
    v_true = np.array([0.7071067811865475, 0.4063712768871578, -0.5418283691828771, 0.2031856384435789])
    e_count += equalCheck(v, v_true, 'gibbs2EP')
    v = rbk.gibbs2Euler121(q)
    v_true = np.array([3.304427597008361, 1.234201174364066, -2.26121636963008])
    e_count += equalCheck(v, v_true, 'gibbs2Euler121')
    v = rbk.gibbs2Euler123(q)
    v_true = np.array([1.467291629150036, -0.6449061163953342, 1.144743256726005])
    e_count += equalCheck(v, v_true, 'gibbs2Euler123')
    v = rbk.gibbs2Euler131(q)
    v_true = np.array([1.733631270213465, 1.234201174364066, -0.6904200428351842])
    e_count += equalCheck(v, v_true, 'gibbs2Euler131')
    v = rbk.gibbs2Euler132(q)
    v_true = np.array([0.54319335066115, 0.8149843403384446, -1.068390851022488])
    e_count += equalCheck(v, v_true, 'gibbs2Euler132')
    v = rbk.gibbs2Euler212(q)
    v_true = np.array([-1.117474807766432, 0.9432554204540935, -0.1901795897648197])
    e_count += equalCheck(v, v_true, 'gibbs2Euler212')
    v = rbk.gibbs2Euler213(q)
    v_true = np.array([-1.434293025994105, 0.9188085603647974, -0.2549399408440935])
    e_count += equalCheck(v, v_true, 'gibbs2Euler213')
    v = rbk.gibbs2Euler231(q)
    v_true = np.array([-1.230028192223063, -0.1536226209659692, 0.9345839026955233])
    e_count += equalCheck(v, v_true, 'gibbs2Euler231')
    v = rbk.gibbs2Euler232(q)
    v_true = np.array([0.4533215190284649, 0.9432554204540935, -1.760975916559716])
    e_count += equalCheck(v, v_true, 'gibbs2Euler232')
    v = rbk.gibbs2Euler312(q)
    v_true = np.array([0.8918931304028546, 0.3623924238788913, -1.482377127697951])
    e_count += equalCheck(v, v_true, 'gibbs2Euler312')
    v = rbk.gibbs2Euler313(q)
    v_true = np.array([-0.6474859022891233, 1.488133410155628, 1.207104533714101])
    e_count += equalCheck(v, v_true, 'gibbs2Euler313')
    v = rbk.gibbs2Euler321(q)
    v_true = np.array([-0.4338654111289937, -1.198236565236741, 1.341967642658489])
    e_count += equalCheck(v, v_true, 'gibbs2Euler321')
    v = rbk.gibbs2Euler323(q)
    v_true = np.array([-2.21828222908402, 1.488133410155628, 2.777900860508998])
    e_count += equalCheck(v, v_true, 'gibbs2Euler323')
    v = rbk.gibbs2MRP(q)
    v_true = np.array([0.2380467826416248, -0.3173957101888331, 0.1190233913208124])
    e_count += equalCheck(v, v_true, 'gibbs2MRP')
    v = rbk.gibbs2PRV(q)
    v_true = np.array([0.9027300063197914, -1.203640008426389, 0.4513650031598956])
    e_count += equalCheck(v, v_true, 'gibbs2PRV')

    v1 = np.array([0.2, -0.25, 0.3])
    C = rbk.MRP2C(v1)
    C_true = np.array([
        [0.1420873822677549, 0.4001248192538094, 0.9053790945330048],
        [-0.9626904702257736, 0.2686646537364468, 0.03234752493088797],
        [-0.2303003133666478, -0.876196001388834, 0.4233702077537369]
    ])
    e_count += equalCheck(C, C_true, 'MRP2C')

    v = rbk.MRP2EP(v1)
    v_true = np.array([0.6771488469601677, 0.3354297693920336, -0.419287211740042, 0.5031446540880503])
    e_count += equalCheck(v, v_true, 'MRP2EP')

    v = rbk.MRP2Euler121(v1)
    v_true = np.array([2.725460144813494, 1.428226451915784, -1.805609061169705])
    e_count += equalCheck(v, v_true, 'MRP2Euler121')
    v = rbk.MRP2Euler123(v1)
    v_true = np.array([1.120685944613971, -0.2323862804943196, 1.424260216144192])
    e_count += equalCheck(v, v_true, 'MRP2Euler123')
    v = rbk.MRP2Euler131(v1)
    v_true = np.array([1.154663818018597, 1.428226451915784, -0.2348127343748092])
    e_count += equalCheck(v, v_true, 'MRP2Euler131')
    v = rbk.MRP2Euler132(v1)
    v_true = np.array([0.1198243320629901, 1.296774918090265, -1.017995395279125])
    e_count += equalCheck(v, v_true, 'MRP2Euler132')
    v = rbk.MRP2Euler212(v1)
    v_true = np.array([-1.537207795170527, 1.298789879764913, 0.4283796513241308])
    e_count += equalCheck(v, v_true, 'MRP2Euler212')
    v = rbk.MRP2Euler213(v1)
    v_true = np.array([-0.4982011776145131, 1.067911809027856, 0.979488037955722])
    e_count += equalCheck(v, v_true, 'MRP2Euler213')
    v = rbk.MRP2Euler231(v1)
    v_true = np.array([-1.415129132201094, 0.4116530390866675, 1.273271587093173])
    e_count += equalCheck(v, v_true, 'MRP2Euler231')
    v = rbk.MRP2Euler232(v1)
    v_true = np.array([0.03358853162436948, 1.298789879764913, -1.142416675470766])
    e_count += equalCheck(v, v_true, 'MRP2Euler232')
    v = rbk.MRP2Euler312(v1)
    v_true = np.array([1.298643836753137, 0.03235316879424937, -1.133389474325039])
    e_count += equalCheck(v, v_true, 'MRP2Euler312')
    v = rbk.MRP2Euler313(v1)
    v_true = np.array([-0.257027406977469, 1.133634172515794, 1.535083362165219])
    e_count += equalCheck(v, v_true, 'MRP2Euler313')
    v = rbk.MRP2Euler321(v1)
    v_true = np.array([1.22957853325386, -1.13227169191098, 0.0762566635156139])
    e_count += equalCheck(v, v_true, 'MRP2Euler321')
    v = rbk.MRP2Euler323(v1)
    v_true = np.array([-1.827823733772366, 1.133634172515794, 3.105879688960115])
    e_count += equalCheck(v, v_true, 'MRP2Euler323')
    v = rbk.MRP2Gibbs(v1)
    v_true = np.array([0.4953560371517029, -0.6191950464396285, 0.7430340557275542])
    e_count += equalCheck(v, v_true, 'MRP2Gibbs')
    v = rbk.MRP2PRV(v1)
    v_true = np.array([0.7538859486650076, -0.9423574358312593, 1.130828922997511])
    e_count += equalCheck(v, v_true, 'MRP2PRV')
    v = rbk.MRPswitch(v1, 1)
    v_true = v1
    e_count += equalCheck(v, v_true, 'MRPswitch')
    v = rbk.MRPswitch(v1, 0.4)
    v_true = np.array([-1.038961038961039, 1.298701298701299, -1.558441558441558])
    e_count += equalCheck(v, v_true, 'MRPswitch')

    if rbk.Picheck(1.2) != 1.2:
        print('Picheck failed')
        e_count += 1
    if rbk.Picheck(4.2) != -2.083185307179586:
        print('Picheck failed')
        e_count += 1
    if rbk.Picheck(-4.2) != 2.083185307179586:
        print('Picheck failed')
        e_count += 1

    C = rbk.PRV2C(v1)
    C_true = np.array([
        [0.9249653552860658, 0.2658656942983466, 0.2715778417245783],
        [-0.3150687400124018, 0.9360360405717283, 0.1567425271513747],
        [-0.212534186867712, -0.2305470957224576, 0.9495668781430935]
    ])
    e_count += equalCheck(C, C_true, 'PRV2C')
    v = rbk.PRV2EP(v1)
    v_true = np.array([0.9760338459808767, 0.09919984446969178, -0.1239998055871147, 0.1487997667045377])
    e_count += equalCheck(v, v_true, 'PRV2EP')
    v = rbk.PRV2Euler121(v1)
    v_true = np.array([2.366822457545908, 0.3898519008736288, -2.164246748437291])
    e_count += equalCheck(v, v_true, 'PRV2Euler121')
    v = rbk.PRV2Euler123(v1)
    v_true = np.array([0.2381830975647435, -0.2141676691157164, 0.3283009769818029])
    e_count += equalCheck(v, v_true, 'PRV2Euler123')
    v = rbk.PRV2Euler131(v1)
    v_true = np.array([0.796026130751012, 0.3898519008736288, -0.5934504216423945])
    e_count += equalCheck(v, v_true, 'PRV2Euler131')
    v = rbk.PRV2Euler132(v1)
    v_true = np.array([0.1659141638227202, 0.3205290820781828, -0.2258549616703266])
    e_count += equalCheck(v, v_true, 'PRV2Euler132')
    v = rbk.PRV2Euler212(v1)
    v_true = np.array([-1.109161329065078, 0.3596045976550934, 0.8564261174295806])
    e_count += equalCheck(v, v_true, 'PRV2Euler212')
    v = rbk.PRV2Euler213(v1)
    v_true = np.array([-0.2201931522496843, 0.2326398873102022, 0.2767451364802878])
    e_count += equalCheck(v, v_true, 'PRV2Euler213')
    v = rbk.PRV2Euler231(v1)
    v_true = np.array([-0.2855829177825758, 0.269101825006778, 0.2414947191533679])
    e_count += equalCheck(v, v_true, 'PRV2Euler231')
    v = rbk.PRV2Euler232(v1)
    v_true = np.array([0.4616349977298192, 0.3596045976550934, -0.714370209365316])
    e_count += equalCheck(v, v_true, 'PRV2Euler232')
    v = rbk.PRV2Euler312(v1)
    v_true = np.array([0.3246867163622526, 0.1573915425330904, -0.2785654591200913])
    e_count += equalCheck(v, v_true, 'PRV2Euler312')
    v = rbk.PRV2Euler313(v1)
    v_true = np.array([-0.7447668031423726, 0.3189446151924337, 1.047343966000315])
    e_count += equalCheck(v, v_true, 'PRV2Euler313')
    v = rbk.PRV2Euler321(v1)
    v_true = np.array([0.2798880637473677, -0.2750321114914171, 0.1635922230133545])
    e_count += equalCheck(v, v_true, 'PRV2Euler321')
    v = rbk.PRV2Euler323(v1)
    v_true = np.array([-2.315563129937269, 0.3189446151924337, 2.618140292795212])
    e_count += equalCheck(v, v_true, 'PRV2Euler323')
    v = rbk.PRV2Gibbs(v1)
    v_true = np.array([0.1016356603597079, -0.1270445754496348, 0.1524534905395618])
    e_count += equalCheck(v, v_true, 'PRV2Gibbs')
    v = rbk.PRV2MRP(v1)
    v_true = np.array([0.05020149056224809, -0.06275186320281011, 0.07530223584337212])
    e_count += equalCheck(v, v_true, 'PRV2MRP')

    v1 = np.array([0.45226701686665, 0.75377836144441, 0.15075567228888, 0.45226701686665])
    v2 = np.array([-0.18663083698528, 0.46657709246321, 0.83983876643378, -0.20529392068381])
    v = rbk.subEP(v1, v2)
    v_true = np.array([0.3010515331052196, -0.762476312817895, -0.0422034859493331, 0.5711538431809339])
    e_count += equalCheck(v, v_true, 'subEP')

    e1 = np.array([10, 20, 30]) * D2R
    e2 = np.array([-30, 200, 81]) * D2R
    v_true = np.array([3.116108453572625, -0.6539785291371149, -0.9652248604105184])
    v = rbk.subEuler123(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler123')
    v = rbk.subEuler231(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler231')
    v = rbk.subEuler312(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler312')
    v_true = np.array([2.932019083757663, 0.6246626379494424, -1.519867235625338])
    v = rbk.subEuler132(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler132')
    v = rbk.subEuler213(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler213')
    v = rbk.subEuler321(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler321')
    v_true = np.array([2.969124082346242, 2.907100217278789, 2.423943306316236])
    v = rbk.subEuler121(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler121')
    v = rbk.subEuler212(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler212')
    v = rbk.subEuler232(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler232')
    v = rbk.subEuler313(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler313')
    v = rbk.subEuler323(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler323')
    v = rbk.subEuler131(e1, e2)
    e_count += equalCheck(v, v_true, 'subEuler131')

    q1 = np.array([1.5, 0.5, 0.5])
    q2 = np.array([-0.5, 0.25, 0.15])
    v = rbk.subGibbs(q1, q2)
    v_true = np.array([4.333333333333333, -0.5, 2.166666666666667])
    e_count += equalCheck(v, v_true, 'subGibbs')
    v = rbk.subMRP(q1, q2)
    v_true = np.array([-0.00537634, 0.04301075, -0.44086022])
    e_count += equalCheck(v, v_true, 'subMRP')
    v = rbk.subPRV(q1, q2)
    v_true = np.array([1.899971363060601, 0.06138537390284331, 0.7174863730592785])
    e_count += equalCheck(v, v_true, 'subPRV')

    theta = 30 * D2R
    C = rbk.Mi(theta, 3)
    C_true = np.array([
        [0.8660254037844387, 0.4999999999999999, 0],
        [-0.4999999999999999, 0.8660254037844387, 0],
        [0, 0, 1.0]
    ])
    e_count += equalCheck(C, C_true, 'Mi')
    C = rbk.Mi(theta, 2)
    C_true = np.array([
        [0.8660254037844387, 0, -0.4999999999999999],
        [0, 1.00000000000000, 0],
        [0.4999999999999999, 0, 0.8660254037844387]
    ])
    e_count += equalCheck(C, C_true, 'Mi')
    C = rbk.Mi(theta, 1)
    C_true = np.array([
        [1.0000000000000000, 0, 0],
        [0, 0.8660254037844387, 0.4999999999999999],
        [0, -0.4999999999999999, 0.8660254037844387]
    ])
    e_count += equalCheck(C, C_true, 'Mi')

    assert e_count < 1, str(e_count) + " functions failed in RigidBodyKinematics.py script"


if __name__ == "__main__":
    test_rigidBodyKinematics(False)