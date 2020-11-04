import multiprocessing as mp
import numpy as np
from matplotlib import pyplot as plt

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

def drag_sensitivity_analysis(ctrlType):

    alt_offsets= np.arange(-1000,1000,500)
    nu_offsets = np.arange(-0.05,0.05,0.01)
    density_multipliers = np.logspace(-1,1,num=10)
    pool = mp.Pool(mp.cpu_count() - 2)


    X,Y,Z = np.meshgrid(alt_offsets, nu_offsets, density_multipliers)

    positions = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T
    kwargs = {'ctrlType':ctrlType, 'makeViz':False}
    arg = [(position, kwargs) for position in positions]
    sim_results = pool.map(sim_wrapper, arg)

    pos_errs = np.empty(len(sim_results))
    vel_errs = np.empty(len(sim_results))
    for ind,result in enumerate(sim_results):
        hill_pos = result['dep_hill_nav.r_DC_H']
        hill_vel = result['dep_hill_nav.v_DC_H']
        pos_errs[ind] = np.linalg.norm(hill_pos[-1,1:4])
        vel_errs[ind] = np.linalg.norm(hill_vel[-1,1:4])

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot_surface(X,Y,pos_errs.reshape(X.shape))
    plt.show()
    print('done!')

if __name__=='__main__':
    drag_sensitivity_analysis('lqr')