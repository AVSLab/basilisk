

# import basic libraries
import os
from matplotlib import animation
import matplotlib.pyplot as plt
import numpy as np

# import message declarations
from Basilisk.architecture import messaging
from Basilisk.architecture import astroConstants

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import locationPointing
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import mrpSteering
from Basilisk.fswAlgorithms import rateServoFullNonlinear
from Basilisk.fswAlgorithms import rwMotorTorque

# import simulation related support (especially stripLocation)
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import stripLocation
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spacecraft

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import astroFunctions
from Basilisk.utilities import macros
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simIncludeRW
from Basilisk.utilities import (
    unitTestSupport,
)  # general support file with common unit test functions

# attempt to import vizard
from Basilisk.utilities import vizSupport

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import LightSource
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import get_cmap, ScalarMappable
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

#
#  Plotting functions
# 

def transform_body_to_inertial(mrp, body_vector):
    """
    Transforms a vector from the body frame to the inertial frame using the spacecraft's MRP attitude.
    
    Parameters:
    - mrp: A 3-element array representing the spacecraft's MRP vector.
    - body_vector: A 3-element array representing the vector in the body frame to be transformed.
    - spacecraft_position: A 3-element array representing the spacecraft's position in the inertial frame.
    
    Returns:
    - A 3-element array representing the vector in the inertial frame.
    """
    # use scipy to convert MRP to a rotation matrix
    r = Rotation.from_mrp(mrp)
    rotation_body_to_inertial = r.as_matrix()

    # Transform the vector from body frame to inertial frame
    inertial_vector = np.dot(rotation_body_to_inertial, body_vector)

    return inertial_vector

def ray_sphere_intersection(camera_position, camera_direction, earth_radius):
    """
    Computes the intersection points between a ray (from the camera) and a sphere (Earth).
    
    Parameters:
    - camera_position: A 3-element array representing the camera's position in the inertial frame.
    - camera_direction: A 3-element array representing the unit direction vector of the ray (camera's pointing direction).
    - earth_radius: The radius of the Earth (in the same units as the position).
    
    Returns:
    - A list of intersection points (if they exist). If no intersection, returns an empty list.
    """

    
    # Vector from the camera to the Earth's center
    L = camera_position
    
    # Coefficients for the quadratic equation
    a = np.dot(camera_direction, camera_direction)
    b = 2.0 * np.dot(camera_direction, L)
    c = np.dot(L, L) - earth_radius**2
    
    # Discriminant of the quadratic equation
    discriminant = b**2 - 4 * a * c
    
    # No intersection if the discriminant is negative
    if discriminant < 0:
        return [0,0,0]
    
    # Compute the two possible solutions for t
    t1 = (-b + np.sqrt(discriminant)) / (2 * a)
    t2 = (-b - np.sqrt(discriminant)) / (2 * a)
    
    # Compute the intersection points
    intersection = camera_position + min(t1,t2) * camera_direction
    return intersection

def plot_attitude_reference_vs_true_attitude(timeLineSet, refSigmaBR, att, strip_start_time:float):
    """Plot the attitude result."""
    plt.figure(1)
    vectorDataRef = refSigmaBR
    v0r = np.array([v[0] for v in vectorDataRef])
    v1r = np.array([v[1] for v in vectorDataRef])
    v2r = np.array([v[2] for v in vectorDataRef])
    plt.plot(timeLineSet, v0r, color='red',linestyle='--', label='$\sigma_{xref}$')
    plt.plot(timeLineSet, v1r, color='blue',linestyle='--',label='$\sigma_{yref}$')
    plt.plot(timeLineSet, v2r, color='green',linestyle='--', label='$\sigma_{zref}$')

    vectorTrueAtt=att
    v0t = np.array([v[0] for v in vectorTrueAtt])
    v1t = np.array([v[1] for v in vectorTrueAtt])
    v2t = np.array([v[2] for v in vectorTrueAtt])
    plt.plot(timeLineSet, v0t, color='red',label='$\sigma_{x}$')
    plt.plot(timeLineSet, v1t, color='blue',label='$\sigma_{y}$')
    plt.plot(timeLineSet, v2t, color='green',label='$\sigma_{z}$')

    plt.xlabel('Time [min]')
    plt.ylabel(r'MRP Att Ref vs True Att')
    # Plot the vertical dashed line
    plt.axvline(x=strip_start_time, color='black', linestyle=':', alpha=0.5)
    plt.axvline(x=0, color='black', linestyle=':', alpha=0.7)
    # Add vertical text on the line
    plt.text(strip_start_time, refSigmaBR.max() * 0.01, 'Start strip imaging', 
             rotation=90, verticalalignment='center', horizontalalignment='right', 
             color='black')
    plt.text(0, refSigmaBR.max() * 0.01, 'Pre-imaging', 
             rotation=90, verticalalignment='center', horizontalalignment='right', 
             color='black')
    plt.legend(loc='lower right')

def plot_attitude_error(timeLineSet, dataSigmaBR, strip_start_time:float):
    """Plot the attitude result."""
    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    vectorData = dataSigmaBR
    sNorm = np.array([np.linalg.norm(v) for v in vectorData])
    v0 = np.array([np.linalg.norm(v[0]) for v in vectorData])
    v1 = np.array([np.linalg.norm(v[1]) for v in vectorData])
    v2 = np.array([np.linalg.norm(v[2]) for v in vectorData])
    plt.plot(timeLineSet, sNorm,
             color='black',label=r'$|\Delta \sigma_{tot}|$')
    plt.plot(timeLineSet, v0,
             color='red',label=r'$|\Delta \sigma_{x}|$')
    plt.plot(timeLineSet, v1,
             color='blue',label=r'$|\Delta \sigma_{y}|$')
    plt.plot(timeLineSet, v2,
             color='green',label=r'$|\Delta \sigma_{z}|$')
             
    plt.xlabel('Time [min]')
    plt.ylabel(r'MRP Attitude Error')
    ax.set_yscale('log')
    # Plot the vertical dashed line
    plt.axvline(x=strip_start_time, color='black', linestyle=':', alpha=0.5)
    plt.axvline(x=0, color='black', linestyle=':', alpha=0.7)
    # Add vertical text on the line
    plt.text(strip_start_time, dataSigmaBR.max() * 0.0001, 'Start strip imaging', 
             rotation=90, verticalalignment='center', horizontalalignment='right', 
             color='black')
    plt.text(0, dataSigmaBR.max() * 0.001, 'Pre-imaging', 
             rotation=90, verticalalignment='center', horizontalalignment='right', 
             color='black') 
    plt.legend(loc='upper right')   

def plot_pointing_error_km(pre_imaging_time,timeLineSet,r_LP_N,intersections):
    plt.figure(3)
    error_m = np.linalg.norm(r_LP_N - intersections, axis=1)
    plt.plot(timeLineSet[int((pre_imaging_time/1e9)/3):],error_m[int((pre_imaging_time/1e9)/3):]/1000,color='black')
    plt.xlabel('Time [min]')
    plt.ylabel('Error pointing vector [km]')
    plt.ylim(0, 1)

def plot_scanline_angle_degrees(pre_imaging_time,timeLineSet,angle,angle_ref):
    plt.figure(4)
    plt.plot(timeLineSet[int((pre_imaging_time/1e9)/3):],angle[int((pre_imaging_time/1e9)/3):]*180/np.pi,color='black',label='Scan vector')
    plt.plot(timeLineSet[int((pre_imaging_time/1e9)/3):],angle_ref[int((pre_imaging_time/1e9)/3):]*180/np.pi,color='black', linestyle='--', label='Scan vector ref')
    plt.xlabel('Time [min]')
    plt.ylabel('Angle scan vector [degrees]')
    plt.legend(loc='lower right')  
    plt.ylim(89, 91)

def plot_pointing_error_velocity(pre_imaging_time, timeLineSet, r_LP_N, intersections):
    plt.figure(5)
    # Calculate position differences
    delta_r_LP_N = np.diff(r_LP_N, axis=0)
    delta_intersections = np.diff(intersections, axis=0)
    
    # Calculate time differences
    delta_t = 3  # Sampling time
    
    # Calculate velocities (m/s)
    velocity_r_LP_N = np.linalg.norm(delta_r_LP_N, axis=1) /delta_t/1000
    velocity_intersections = np.linalg.norm(delta_intersections, axis=1) /delta_t/1000
    
    # Plot velocities
    start_index = int((pre_imaging_time / 1e9) / 3)
    plt.plot(timeLineSet[1:][start_index:], velocity_r_LP_N[start_index:], label='Ref acq speed', color='black', linestyle='--')
    plt.plot(timeLineSet[1:][start_index:], velocity_intersections[start_index:], label='True acq speed', color='black')
    
    plt.xlabel('Time [min]')
    plt.ylabel('Velocity [km/s]')
    plt.legend()

def plot_rw_motor_torque(timeData, dataRW,dataUsReq,strip_start_time):
    """Plot the actual RW motor torque."""
    plt.figure(6)
    plt.semilogy(timeData, np.abs(dataRW[0]),color='red',label='$|u_{s,' + 'x' + '}|$')
    plt.semilogy(timeData, np.abs(dataRW[1]),color='blue',label='$|u_{s,' + 'y' + '}|$')
    plt.semilogy(timeData, np.abs(dataRW[2]),color='green',label='$|u_{s,' + 'z' + '}|$')
    plt.semilogy(timeData, np.abs(dataRW[3]),color='black',label='$|u_{s,' + 'skewed' + '}|$')
    plt.semilogy(timeData, np.abs(dataUsReq[:, 0]),color='red',linestyle='--',label='$|u_{s,' + 'xref' + '}|$')
    plt.semilogy(timeData, np.abs(dataUsReq[:, 1]),color='blue',linestyle='--',label='$|u_{s,' + 'yref' + '}|$')
    plt.semilogy(timeData, np.abs(dataUsReq[:, 2]),color='green',linestyle='--',label='$|u_{s,' + 'zref' + '}|$')
    plt.semilogy(timeData, np.abs(dataUsReq[:, 3]),color='black',linestyle='--',label='$|u_{s,' + 'skewed_ref' + '}|$')
    plt.axvline(x=strip_start_time, color='black', linestyle=':', alpha=0.5)
    plt.axvline(x=0, color='black', linestyle=':', alpha=0.7)
    plt.text(strip_start_time, dataRW[0].max() * 0.01, 'Start strip imaging', 
             rotation=90, verticalalignment='center', horizontalalignment='right', 
             color='black')
    plt.text(0, dataRW[0].max() * 0.01, 'Pre-imaging', 
             rotation=90, verticalalignment='center', horizontalalignment='right', 
             color='black') 
    plt.legend(loc='lower right', ncol=4)
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')

def plot_rw_speeds(timeData, dataOmegaRW, strip_start_time):
    """Plot the RW speeds."""
    plt.figure(7)
    plt.plot(timeData, dataOmegaRW[:, 0] / macros.RPM,color='red',label=r'$\Omega_{' + 'x' + '}$')
    plt.plot(timeData, dataOmegaRW[:, 1] / macros.RPM,color='blue',label=r'$\Omega_{' + 'y' + '}$')
    plt.plot(timeData, dataOmegaRW[:, 2] / macros.RPM,color='green',label=r'$\Omega_{' + 'z' + '}$')
    plt.plot(timeData, dataOmegaRW[:, 3] / macros.RPM,color='black',label=r'$\Omega_{' + 'skewed' + '}$')
    plt.axvline(x=strip_start_time, color='black', linestyle=':', alpha=0.5)
    plt.axvline(x=0, color='black', linestyle=':', alpha=0.7)
    plt.text(strip_start_time, dataOmegaRW[:, 0].max() * 0.01, 'Start strip imaging', 
             rotation=90, verticalalignment='center', horizontalalignment='right', 
             color='black')
    plt.text(0, dataOmegaRW[:, 0].max() * 0.01, 'Pre-imaging', 
             rotation=90, verticalalignment='center', horizontalalignment='right', 
             color='black') 
    plt.legend(loc='upper right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')

def plot_access(timeLineSet, hasAccess, strip_start_time:float):
    plt.figure(8)
    plt.plot(timeLineSet, hasAccess, color='black')
    # Plot the vertical dashed line
    plt.axvline(x=strip_start_time, color='black', linestyle=':', alpha=0.5)
    plt.axvline(x=0, color='black', linestyle=':', alpha=0.7)
    # Add vertical text on the line
    plt.text(strip_start_time, max(hasAccess) * 0.4, 'Start strip imaging', 
             rotation=90, verticalalignment='center', horizontalalignment='right', 
             color='black')
    plt.text(0, max(hasAccess) * 0.4, 'Pre-imaging', 
             rotation=90, verticalalignment='center', horizontalalignment='right', 
             color='black')
    plt.xlabel("Time [min]")
    plt.ylabel("Imaging Target Access")

def create_earth(ax, earth_radius):
    # Create a sphere representing the Earth
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = earth_radius * np.outer(np.cos(u), np.sin(v))
    y = earth_radius * np.outer(np.sin(u), np.sin(v))
    z = earth_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    
    # Create a light source object for shading
    ls = LightSource(azdeg=0, altdeg=65)
    rgb = ls.shade(z, cmap=plt.cm.gray, vert_exag=0.1, blend_mode='soft')
    
    # Plot the Earth as a surface with grid lines
    ax.plot_surface(x, y, z, facecolors=rgb, rstride=5, cstride=5, alpha=0.1, edgecolor='k')
    
    # Set the axes to have the same proportion
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
    mid_x = (x.max()+x.min()) * 0.5
    mid_y = (y.max()+y.min()) * 0.5
    mid_z = (z.max()+z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

def set_zoom(ax, r_LP_N):
    # Set the axes to zoom in on the trajectory
    max_range = np.array([r_LP_N[:, 0].max()-r_LP_N[:, 0].min(), 
                          r_LP_N[:, 1].max()-r_LP_N[:, 1].min(), 
                          r_LP_N[:, 2].max()-r_LP_N[:, 2].min()]).max() / 2.0
    mid_x = (r_LP_N[:, 0].max()+r_LP_N[:, 0].min()) * 0.5
    mid_y = (r_LP_N[:, 1].max()+r_LP_N[:, 1].min()) * 0.5
    mid_z = (r_LP_N[:, 2].max()+r_LP_N[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

def clear_previous_plot(ax):
    # Clear the previous trajectory
    for collection in ax.collections:
        collection.remove()
    for line in ax.lines:
        line.remove()

def plot_trajectory(ax, trajectory_points, current_step, camera_vectors=None, camera_n_skip=5, name='Trajectory', style='-', color='blue', color_by_iteration=False,linewidth=1,point=True):
    if color_by_iteration:
        # Create a colormap
        cmap = plt.get_cmap('viridis')
        colors = cmap(np.linspace(0, 1, current_step))
        
        # Plot the trajectory with colors based on iteration
        for i in range(current_step):
            if i==0:
                ax.plot(trajectory_points[i:i+2, 0], trajectory_points[i:i+2, 1], trajectory_points[i:i+2, 2], style, label='Intersection between the camera vectors and the surface of Earth', color='aquamarine', markersize=2, linewidth=1)
            else:
                ax.plot(trajectory_points[i:i+2, 0], trajectory_points[i:i+2, 1], trajectory_points[i:i+2, 2], style, color='aquamarine', markersize=2, linewidth=1)
    else:
        # Plot the trajectory up to the current step with circle markers
        ax.plot(trajectory_points[:current_step, 0], trajectory_points[:current_step, 1], trajectory_points[:current_step, 2], style, label=name, color=color, markersize=2, linewidth=linewidth)
    
    if point==True:
        label1=f"Initial {name}"
        label2=f"Final {name}"
        if f'{name}'=='Target Strip':
            label1='Boulder'
            label2='Pheonix'

            # Plot the initial point with a triangle marker in red
            ax.scatter(trajectory_points[0, 0], trajectory_points[0, 1], trajectory_points[0, 2], color='black', marker='^', label=label1)
            
            # Always plot the final point with a square marker in green
            ax.scatter(trajectory_points[-1, 0], trajectory_points[-1, 1], trajectory_points[-1, 2], color='black', marker='s', label=label2)
    
    if camera_vectors is not None:
        # Plot the camera vectors as arrows for every camera_n_skip points
        ax.quiver(trajectory_points[80:current_step:camera_n_skip, 0], trajectory_points[80:current_step:camera_n_skip, 1], trajectory_points[80:current_step:camera_n_skip, 2],
                  camera_vectors[80:current_step:camera_n_skip, 0], camera_vectors[80:current_step:camera_n_skip, 1], camera_vectors[80:current_step:camera_n_skip, 2],
                  length=astroConstants.REQ_EARTH * 1e3 * 0.08, normalize=True, color=color, label='Pointing camera vectors (z-axis of the body frame)')
    
    ax.legend()

def plot_3D_R_LP_N(ax,pre_imaging_time,r_LP_N, earth_radius, tran, camera_vectors, intersections, current_step,ref_line_vectors, init=False, plot_earth=True, zoom_trajectory=False):
    if init:
        if plot_earth:
            create_earth(ax, earth_radius)
        if zoom_trajectory:
            set_zoom(ax, r_LP_N)
        
        ax.set_box_aspect([1,1,1])  # Aspect ratio is 1:1:1
        
        # Remove the figure grid
        ax.grid(False)
        
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        #ax.set_title('Imaging of the Strip Boulder-Pheonix')
    
    clear_previous_plot(ax)
    
    if plot_earth:
        create_earth(ax, earth_radius)
    
    if zoom_trajectory:
        set_zoom(ax, r_LP_N)
    
    plot_trajectory(ax, r_LP_N[int((pre_imaging_time/1e9)/3):], current_step, name='Target Strip', style='-', color=(1, 0, 0, 0.4),linewidth=10)
    plot_trajectory(ax, r_LP_N, current_step, name='Pre-imaging of the strip', style='-', color=(1, 0, 0, 0.2),linewidth=10,point=False)
    plot_trajectory(ax, intersections[int((pre_imaging_time/1e9)/3):], current_step, name='Intersection',style='o', color='red', color_by_iteration=True)
    plot_trajectory(ax, tran, current_step, camera_vectors=camera_vectors, name='Satellite orbit', camera_n_skip=20, style='-', color='green',linewidth=3)

    # Plot every 25th reference line vector starting from the intersection points
    start_index = int((pre_imaging_time / 1e9) / 3)
    intersection_points = intersections[start_index:]
    ref_line_vectors = ref_line_vectors[start_index:]
    
    for i in range(0, len(ref_line_vectors), 20):
        if i < len(intersection_points):
            start_point = intersection_points[i]
            vector = ref_line_vectors[i]
            end_point = start_point + vector
            
            # Plot the vector
            ax.quiver(start_point[0], start_point[1], start_point[2],
                      vector[0], vector[1], vector[2], length=astroConstants.REQ_EARTH * 1e3 * 0.08, normalize=True, color='blue', label='Scan line vectors (y-axis of the body frame)' if i == 0 else "")
            ax.legend()

def plot_full_trajectory(r_LP_N,pre_imaging_time, earth_radius, tran, camera_vectors, intersections,ref_line_vectors, plot_earth=True, zoom_trajectory=False):
    fig, ax = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(12.8, 7.2))
    plot_3D_R_LP_N(ax,pre_imaging_time, r_LP_N, earth_radius, tran, camera_vectors,intersections ,current_step=len(r_LP_N),ref_line_vectors=ref_line_vectors, init=True, plot_earth=plot_earth, zoom_trajectory=zoom_trajectory)


#
#  Running function 
# 

def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """
    #
    #  Create the simulation process
    # 

    #Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #Set the simulation time
    pre_imaging_time = macros.min2nano(5)
    simulationTime = macros.min2nano(11.0)+ pre_imaging_time

    #Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    #Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.01)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    # Setup the simulation tasks/objects
    #

    # Initialize spacecraft object 
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Define the simulation inertia
    I = [500., 0., 0.,
         0., 300., 0.,
         0., 0., 200.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    #Attach gravity model to the spacecraft
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu
    gravFactory.addBodiesTo(scObject)

    #Create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments (Same parameters as BSK-RL)
    rwFactory = simIncludeRW.rwFactory()
    initOmega = [500.0, 500.0, 500.0, 500.0]
    #Skewed configuration usually in super-agile satellites such as Pleiades
    RW1 = rwFactory.create('Honeywell_HR16'
                           , [1, 0, 0]
                           , maxMomentum=100.
                           , Omega=initOmega[0] # RPM
                           , u_max=1.0 # From 1 to 10 Nm for super-agile satellites
                           )
    RW2 = rwFactory.create('Honeywell_HR16'
                           , [0, 1, 0]
                           , maxMomentum=100.
                           , Omega=initOmega[1]  # RPM
                           , u_max=1.0
                           )
    RW3 = rwFactory.create('Honeywell_HR16'
                           , [0, 0, 1]
                           , maxMomentum=100.
                           , Omega=initOmega[2]  # RPM
                           , u_max=1.0
                           )
    RW4 = rwFactory.create('Honeywell_HR16'
                           , [1 / np.sqrt(3), 1 / np.sqrt(3), 1 / np.sqrt(3)]
                           , maxMomentum=100.
                           , Omega=initOmega[3]  # RPM
                           , u_max=1.0
                           )
    numRW = rwFactory.getNumOfDevices()

    # Create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    # Add RW object array to the simulation process
    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)
    # Add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, 1)

    # Add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # Define the location of the strip
    striptarget = stripLocation.StripLocation()
    striptarget.ModelTag = "ImagingBoulderDenverStrip"
    striptarget.planetRadius = astroConstants.REQ_EARTH * 1e3
    striptarget.acquisition_speed = 6*1e-6
    striptarget.pre_imaging_time= pre_imaging_time;
    striptarget.specifyLocationStart(np.radians(39.99), np.radians(-105.26), 0)
    striptarget.specifyLocationEnd(np.radians(33.48), np.radians(-112.078232), 0)
    # striptarget.specifyLocationEnd(np.radians(0), np.radians(-136.629), 0)
    # striptarget.specifyLocationStart(np.radians(0), np.radians(-142.078232), 0)
    striptarget.newpstart()
    striptarget.minimumElevation = np.radians(10.0)
    striptarget.maximumRange = 1e9
    striptarget.OldSimNanos=0
    striptarget.duration_strip_imaging=0
    striptarget.addSpacecraftToModel(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, striptarget)

    # Setup guidance module
    locPoint = locationPointing.locationPointing()
    locPoint.ModelTag = "locPoint"
    scSim.AddModelToTask(simTaskName, locPoint)
    locPoint.pHat_B = [0, 0, 1]
    locPoint.cHat_B = [0, 1, 0]
    locPoint.useBoresightRateDamping = 1

    # Setup steering control module
    mrpControl = mrpSteering.mrpSteering()
    mrpControl.ModelTag = "MRP_Steering"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.K1 = 0.9
    mrpControl.K3 = 15
    mrpControl.ignoreOuterLoopFeedforward = False
    mrpControl.omega_max = np.radians(3)

    # Setup Rate servo module
    servo = rateServoFullNonlinear.rateServoFullNonlinear()
    servo.ModelTag = "rate_servo"
    servo.Ki = 5.0
    servo.P = 150.0
    servo.integralLimit = 2. / servo.Ki * 0.1
    servo.knownTorquePntB_B = [0., 0., 0.]
    scSim.AddModelToTask(simTaskName, servo)

    # Add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)
    controlAxes_B = [
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    ]
    rwMotorTorqueObj.controlAxes_B = controlAxes_B
    
    # Create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # FSW RW configuration message
    fswSetupRW.clearSetup()
    for key, rw in rwFactory.rwList.items():
        fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
    fswRwParamMsg = fswSetupRW.writeConfigMessage()

    # setup message connections
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    locPoint.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    locPoint.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    locPoint.locationstripInMsg.subscribeTo(striptarget.currentStripStateOutMsg)
    mrpControl.guidInMsg.subscribeTo(locPoint.attGuidOutMsg)
    servo.guidInMsg.subscribeTo(locPoint.attGuidOutMsg)
    servo.vehConfigInMsg.subscribeTo(vcMsg)
    servo.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    servo.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    servo.rateSteeringInMsg.subscribeTo(mrpControl.rateCmdOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(servo.cmdTorqueOutMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)

    #
    #   Setup data logging before the simulation is initialized
    #

    samplingTime=3000000000 #3s
    mrpLog = mrpControl.rateCmdOutMsg.recorder(samplingTime)
    rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    rwStateLog = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    attErrLog = locPoint.attGuidOutMsg.recorder(samplingTime)
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    locationLog = striptarget.accessOutMsgs[-1].recorder(samplingTime)
    target = striptarget.currentStripStateOutMsg.recorder(samplingTime)
    attref = locPoint.attRefOutMsg.recorder(samplingTime)
    
    scSim.AddModelToTask(simTaskName, mrpLog)
    scSim.AddModelToTask(simTaskName, attErrLog)
    scSim.AddModelToTask(simTaskName, snAttLog)
    scSim.AddModelToTask(simTaskName, snTransLog)
    scSim.AddModelToTask(simTaskName, locationLog)
    scSim.AddModelToTask(simTaskName, rwMotorLog)
    scSim.AddModelToTask(simTaskName, rwStateLog)
    scSim.AddModelToTask(simTaskName, target)
    scSim.AddModelToTask(simTaskName, attref)
    rwLogs = []
    for item in range(numRW):
        rwLogs.append(rwStateEffector.rwOutMsgs[item].recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, rwLogs[item])

    #
    #   initialize Spacecraft States with initialization variables
    #

    oe = orbitalMotion.ClassicElements()
    oe.a = (6378 + 600) * 1000.0  # meters
    oe.e = 0.1
    oe.i = 45 * macros.D2R
    oe.Omega = 88.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 125.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[-0.1], [-0.2], [-0.25]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B


    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()


    #
    #   Run the simulation to point the camera towards Boulder 
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    # dataLr = mrpLog.torqueRequestBody
    refSigmaBR=attref.sigma_RN
    dataOmegaRW = rwStateLog.wheelSpeeds[:, :numRW]
    dataSigmaBR = attErrLog.sigma_BR
    dataOmegaBR = attErrLog.omega_BR_B
    dataUsReq = rwMotorLog.motorTorque[:, :numRW]
    hasAccess = locationLog.hasAccess
    r_LP_N = target.r_LP_N
    v_LP_N = target.v_LP_N
    att=snAttLog.sigma_BN
    tran=snTransLog.r_BN_N
    att_ref=attref.sigma_RN
    dataRW = []
    for i in range(numRW):
        dataRW.append(rwLogs[i].u_current)
    np.set_printoptions(precision=16)
 
    # Intersection points between the camera vectors and the surface of the Earth
    intersections=np.array([ray_sphere_intersection(tran[i], transform_body_to_inertial(att[i], locPoint.pHat_B), astroConstants.REQ_EARTH * 1e3) for i in range(len(tran))])
    camera_vectors = np.array([transform_body_to_inertial(att[i], locPoint.pHat_B) for i in range(len(att))])
       
    # Angle between the scanning line vector and the central line of the strip
    line_vectors = np.array([transform_body_to_inertial(att[i], locPoint.cHat_B) for i in range(len(att))])
    angle = np.array([np.arccos(np.dot(line_vectors[i],v_LP_N[i])/(np.linalg.norm(line_vectors[i])*np.linalg.norm(v_LP_N[i]))) for i in range(len(att))])
    line_vectors_ref= np.array([transform_body_to_inertial(att_ref[i], locPoint.cHat_B) for i in range(len(att_ref))])
    angle_ref = np.array([np.arccos(np.dot(line_vectors_ref[i],v_LP_N[i])/(np.linalg.norm(line_vectors_ref[i])*np.linalg.norm(v_LP_N[i]))) for i in range(len(att_ref))])

    ## Plot results   
    if show_plots:
        timeLineSet = attErrLog.times() * macros.NANO2MIN
        plt.close("all")  # clears out plots from earlier test runs

        figureList = {}
        plot_attitude_reference_vs_true_attitude(timeLineSet,refSigmaBR,att,pre_imaging_time*macros.NANO2MIN)
        pltName = fileName + "1"
        figureList[pltName] = plt.figure(1)

        plot_attitude_error(timeLineSet,dataSigmaBR,pre_imaging_time*macros.NANO2MIN)
        pltName = fileName + "2"
        figureList[pltName] = plt.figure(2)

        plot_pointing_error_km(pre_imaging_time,timeLineSet,r_LP_N,intersections)
        pltName = fileName + "3"
        figureList[pltName] = plt.figure(3)

        plot_scanline_angle_degrees(pre_imaging_time,timeLineSet,angle,angle_ref)
        pltName = fileName + "4"
        figureList[pltName] = plt.figure(4)

        plot_pointing_error_velocity(pre_imaging_time, timeLineSet, r_LP_N, intersections)
        pltName = fileName + "5"
        figureList[pltName] = plt.figure(5)

        plot_rw_motor_torque(timeLineSet,dataRW,dataUsReq,pre_imaging_time*macros.NANO2MIN)
        pltName = fileName + "6" 
        figureList[pltName] = plt.figure(6)

        plot_rw_speeds(timeLineSet, dataOmegaRW, pre_imaging_time*macros.NANO2MIN)
        pltName = fileName + "7" 
        figureList[pltName] = plt.figure(7)

        plot_access(timeLineSet, hasAccess, pre_imaging_time*macros.NANO2MIN)
        pltName = fileName + "8" 
        figureList[pltName] = plt.figure(8)

        plot_full_trajectory(r_LP_N, pre_imaging_time,astroConstants.REQ_EARTH * 1e3,tran, camera_vectors,intersections,line_vectors)
        pltName = fileName + "9" 
        figureList[pltName] = plt.figure(9)

        plt.show()

        
 #
 # This statement below ensures that the unit test scrip can be run as a
 # stand-along python script
 #
if __name__ == "__main__":
    run(
         True  # show_plots
     )

