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


from Basilisk.utilities import macros as mc

def Honeywell_HR16(maxMomentum_level):
    """
    Honeywell HR16 (100Nm, 75Nm, 50Nm)

    RW Information Source:
    http://www51.honeywell.com/aero/common/documents/Constellation_Series_Reaction_Wheels.pdf

    There are 3 momentum capacity options for this RW type.  The maximum momentum
    capacity must be set prior to creating the HR16 RW type using::

        maxMomentum = 100, 75 or 50

    :param maxMomentum_level:
    :return:
    """
    Omega_max = 6000.0 * mc.rpm2radsec  # maximum speed
    u_max = 0.200  # maximum RW torque [Nm]
    u_min = 0.00001  # minimum RW torque [Nm]
    u_f = 0.0005  # static friction torque [Nm]
    maxMomentum_large = 100
    maxMomentum_medium = 75
    maxMomentum_small = 50


    # mass = RW rotor mass [kg]. Note: the rotor mass here is set equal to the RW mass of the above spec sheet.
    # U_s = static RW imbalance [kg*m]
    # U_d = dynamic RW imbalance [kg*m^2]
    if maxMomentum_level == 'large':
        mass = 12.0
        U_s = 4.8E-6
        U_d = 15.4E-7
    elif maxMomentum_level == 'medium':
        mass = 10.4
        U_s = 3.8E-6
        U_d = 11.5E-7
    elif maxMomentum_level == 'small':
        mass = 9.0
        U_s = 2.8E-6
        U_d = 7.7E-7
    else:
        raise ValueError('Honeywell_HR16(maxMomentum_level) only has arg options maxMomentum = [large, medium, small]')

    maxMomentum = globals().get('maxMomentum_' + maxMomentum_level)
    if maxMomentum is None:
        raise ValueError(f"maxMomentum_{maxMomentum_level} not found")

    return (Omega_max, u_max, u_min, u_f, mass, U_s, U_d, maxMomentum)



def Honeywell_HR14(maxMomentum):
    """
    Honeywell HR14 (25Nm, 50Nm, 75Nm)

    RW Information Source:
    http://www51.honeywell.com/aero/common/documents/Constellation_Series_Reaction_Wheels.pdf

    There are 3 momentum capacity options for this RW type.  The maximum momentum
    capacity must be set prior to creating the HR14 RW type using::

        options.maxMomentum = 75, 50 or 25

    :param maxMomentum:
    :return:
    """
    Omega_max = 6000.0 * mc.rpm2radsec  # maximum speed
    u_max = 0.200  # maximum RW torque [Nm]
    u_min = 0.00001  # minimum RW torque [Nm]
    u_f = 0.0005  # static friction torque [Nm]

    # mass = RW rotor mass [kg]. Note: the rotor mass here is set equal to the RW mass of the above spec sheet.
    # U_s = static RW imbalance [kg*m]
    # U_d = dynamic RW imbalance [kg*m^2]
    large = 75
    medium = 50
    small = 25
    if maxMomentum == large:
        mass = 10.6
        U_s = 4.8E-6
        U_d = 13.7E-7
    elif maxMomentum == medium:
        mass = 8.5
        U_s = 3.5E-6
        U_d = 9.1E-7
    elif maxMomentum == small:
        mass = 7.5
        U_s = 2.2E-6
        U_d = 4.6E-7
    else:
        raise ValueError('Honeywell_HR14(maxMomentum) only has arg options maxMomentum = [large, medium, small]')

    return (Omega_max, u_max, u_min, u_f, mass, U_s, U_d)



def Honeywell_HR12(maxMomentum):
    """
    Honeywell HR12 (12Nm, 25Nm, 50Nm)

    RW Information Source:
    http://www51.honeywell.com/aero/common/documents/Constellation_Series_Reaction_Wheels.pdf

    There are 3 momentum capacity options for this RW type.  The maximum momentum
    capacity must be set prior to creating the HR12 RW type using::

        options.maxMomentum = 12, 25 or 50

    :param maxMomentum:
    :return:
    """
    Omega_max = 6000.0 * mc.rpm2radsec  # maximum speed
    u_max = 0.200  # maximum RW torque [Nm]
    u_min = 0.00001  # minimum RW torque [Nm]
    u_f = 0.0005  # static friction torque [Nm]

    # mass = RW rotor mass [kg]. Note: the rotor mass here is set equal to the RW mass of the above spec sheet.
    # U_s = static RW imbalance [kg*m]
    # U_d = dynamic RW imbalance [kg*m^2]
    large = 50
    medium = 25
    small = 12
    if maxMomentum == large:
        mass = 9.5
        U_s = 4.4E-6
        U_d = 9.1E-7
    elif maxMomentum == medium:
        mass = 7.0
        U_s = 2.4E-6
        U_d = 4.6E-7
    elif maxMomentum == small:
        mass = 6.0
        U_s = 1.5E-6
        U_d = 2.2E-7
    else:
        raise ValueError('Honeywell_HR12(maxMomentum) only has arg options maxMomentum = [large, medium, small]')

    return (Omega_max, u_max, u_min, u_f, mass, U_s, U_d)
