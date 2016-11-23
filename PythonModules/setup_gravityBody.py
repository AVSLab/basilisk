''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
import csv
import math
import six_dof_eom

def LoadGravFromFile(FileName, GravBody, JParamsSelect):
    csvfile = open(FileName, 'rb')
    csvreader = csv.reader(csvfile)
    FirstLine = True
    NextJindex = 0
    AllJParams = []
    for row in csvreader:
        if (FirstLine == True):
            GravBody.mu = float(row[1])
            GravBody.radEquator = float(row[0])
            FirstLine = False
        elif (int(row[0]) == JParamsSelect[NextJindex]):
            LocalJParam = -math.sqrt(2 * JParamsSelect[NextJindex] + 1) * float(row[2])
            AllJParams.append(LocalJParam)
            NextJindex += 1
            if (NextJindex >= len(JParamsSelect)):
                break
    return (AllJParams)

def Add_sunGravityBody(sim, sun):
    sun.BodyMsgName = "sun_planet_data"
    sun.outputMsgName = "sun_display_frame_data"
    sun.mu = 1.32712440018E20  # meters!
    sun.IsCentralBody = True
    sun.IsDisplayBody = True
    sun.UseJParams = False
    sim.VehDynObject.AddGravityBody(sun)

def Add_earthGravityBody(sim, earth):
    JParamsSelect = [2, 3, 4, 5, 6]
    EarthGravFile = sim.simBasePath + '/External/LocalGravData/GGM03S.txt'
    earth.BodyMsgName = "earth_planet_data"
    earth.outputMsgName = "earth_display_frame_data"
    earth.IsCentralBody = False
    earth.UseJParams = False
    JParams = LoadGravFromFile(EarthGravFile, earth, JParamsSelect)
    earth.JParams = six_dof_eom.DoubleVector(JParams)
    sim.VehDynObject.AddGravityBody(earth)

def Add_marsGravityBody(sim, mars):
    JParamsSelect = [2, 3, 4, 5, 6]
    MarsGravFile = sim.simBasePath + '/External/LocalGravData/GGM2BData.txt'
    mars.BodyMsgName = "mars barycenter_planet_data"
    mars.outputMsgName = "mars barycenter_display_frame_data"
    mars.IsCentralBody = False
    mars.UseJParams = False
    JParams = LoadGravFromFile(MarsGravFile, mars, JParamsSelect)
    mars.JParams = six_dof_eom.DoubleVector(JParams)
    sim.VehDynObject.AddGravityBody(mars)

def Init_spacecraftVehicle(sc, r0, v0, sigma0_BN, omega0_BN, m, I, DCM_BS, CoM):
    sc.PositionInit = six_dof_eom.DoubleVector(r0)
    sc.VelocityInit = six_dof_eom.DoubleVector(v0)
    sc.AttitudeInit = six_dof_eom.DoubleVector(sigma0_BN)
    sc.AttRateInit = six_dof_eom.DoubleVector(omega0_BN)
    sc.baseMass = m
    sc.baseInertiaInit = six_dof_eom.DoubleVector(I)
    sc.T_Str2BdyInit = six_dof_eom.DoubleVector(DCM_BS)
    sc.baseCoMInit = six_dof_eom.DoubleVector(CoM)
