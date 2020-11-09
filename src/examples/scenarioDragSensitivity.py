import multiprocessing as mp
import numpy as np
from matplotlib import pyplot as plt
import pickle

from scenarioDragRendezvous import drag_simulator

""" 
This scenario demonstrates how a basilisk scenario can be used to evaluate the sensitivity of a drag-driven rendezvous controller.
"""

def sim_wrapper(arg):
    """
    This function wraps the basilisk simulator, mapping from one """
    args, kwargs = arg

    result =  drag_simulator(*args, **kwargs)

    return result

def drag_sensitivity_analysis(ctrlType, rerunSims=False):

    alt_offsets= np.arange(-100,100,10)
    nu_offsets = np.arange(0.001,0.04,0.005)
    density_multipliers = np.logspace(-1,0.5,num=20)
    pool = mp.Pool(mp.cpu_count() - 2)


    X,Y,Z = np.meshgrid(alt_offsets, nu_offsets, density_multipliers)

    positions = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T
    kwargs = {'ctrlType':ctrlType, 'makeViz':False}
    arg = [(position, kwargs) for position in positions]
    if rerunSims:
        sim_results = pool.map(sim_wrapper, arg)

        with open(ctrlType+"_sweep_results.pickle", "wb") as output_file:
                pickle.dump(sim_results, output_file,-1)
    else:
        with open(ctrlType+"_sweep_results.pickle","rb") as fp:
            sim_results = pickle.load(fp)
    dens_mults = []
    pos_errs = np.empty(len(sim_results))
    vel_errs = np.empty(len(sim_results))
    init_relpos = np.empty(len(sim_results))
    init_relvel = np.empty(len(sim_results))
    for ind,result in enumerate(sim_results):
        hill_pos = result['dep_hill_nav.r_DC_H']
        hill_vel = result['dep_hill_nav.v_DC_H']
        init_relpos[ind] = np.linalg.norm(hill_pos[0,1:4])
        init_relvel[ind] = np.linalg.norm(hill_vel[0,1:4])
        pos_errs[ind] = np.linalg.norm(hill_pos[-1,1:4])
        vel_errs[ind] = np.linalg.norm(hill_vel[-1,1:4])


    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.scatter(init_relpos,np.log10(Z.ravel()),np.log10(pos_errs))
    plt.savefig('scatter.png')
    ax.set_xlabel('Maneuver Displacement (m)')
    ax.set_ylabel('Log Density Multiplier')

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    plt.contour(init_relpos.reshape(X.shape)[5,:,:],Z[5,:,:],np.log10(pos_errs.reshape(Z.shape)[5,:,:]))
    plt.savefig('scatter.png')
    ax.set_xlabel('Maneuver Displacement (m)')
    ax.set_ylabel('Log Density Multiplier')

    fig = plt.figure()
    plt.contour(X[:,:,5], Y[:,:,5], np.log10(pos_errs.reshape(X.shape)[:,:,5]))
    plt.savefig('scatter.png')
    ax.set_xlabel('Altitude offset (m)')
    ax.set_ylabel('True anomaly offset (deg)')

    fig = plt.figure()
    plt.contour(X[:,:,8], Y[:,:,8], np.log10(pos_errs.reshape(X.shape)[:,:,8]))
    plt.savefig('scatter.png')
    ax.set_xlabel('Altitude offset (m)')
    ax.set_ylabel('True anomaly offset (deg)')
    
    fig = plt.figure()
    plt.contour(X[:,:,3], Y[:,:,3], np.log10(pos_errs.reshape(X.shape)[:,:,3]))
    plt.savefig('scatter.png')
    ax.set_xlabel('Altitude offset (m)')
    ax.set_ylabel('True anomaly offset (deg)')

    
    fig = plt.figure()
    plt.contour(Y[:,5,:], np.log10(Z[:,5,:]), np.log10(pos_errs.reshape(X.shape)[:,5,:]))
    plt.savefig('scatter.png')
    ax.set_xlabel('Altitude offset (m)')
    ax.set_ylabel('True anomaly offset (deg)')
    plt.show()
    print('done!')

if __name__=='__main__':
    drag_sensitivity_analysis('desen', rerunSims=True)