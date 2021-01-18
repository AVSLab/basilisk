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

    alt_offsets= [0]#np.arange(-100,100,10)
    nu_offsets = np.arange(0.001,0.1,0.005)
    density_multipliers = np.logspace(-1,0.4,num=20)
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
    # dens_mults = []
    # pos_errs = np.empty(len(sim_results))
    # vel_errs = np.empty(len(sim_results))
    # init_relpos = np.empty(len(sim_results))
    # init_relvel = np.empty(len(sim_results))
    # dens_list = np.empty(len(sim_results))
    
    # plt.figure()
    # for ind,result in enumerate(sim_results):
    #     hill_pos = result['dep_hill_nav.r_DC_H']
    #     hill_vel = result['dep_hill_nav.v_DC_H']
    #     init_relpos[ind] = np.linalg.norm(hill_pos[1,1:4])
    #     init_relvel[ind] = np.linalg.norm(hill_vel[1,1:4])
    #     pos_errs[ind] = np.linalg.norm(hill_pos[-1,1:4])
    #     vel_errs[ind] = np.linalg.norm(hill_vel[-1,1:4])

    #     if ind%5==0:
    #         plt.plot(hill_pos[:,1],hill_pos[:,2])
    #         plt.grid()
    #         plt.xlabel('Hill X (m)')
    #         plt.ylabel('Hill Y (m)')
    #         plt.title(f'Simulation {ind}')
    #     # dens_list[ind] = result['']


    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.scatter(init_relpos,Z.ravel(),pos_errs)
    # plt.savefig('scatter.png')
    # ax.set_xlabel('Maneuver Displacement (m)')
    # ax.set_ylabel('Density Multiplier')

    # plt.figure()
    # plt.semilogy(init_relpos, pos_errs)
    # plt.xlabel('Index')
    # plt.ylabel('Position error')

    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # # plt.contour(init_relpos.reshape(X.shape)[5,:,:],Z[5,:,:],np.log10(pos_errs.reshape(Z.shape)[5,:,:]))
    # plt.savefig('scatter.png')
    # ax.set_xlabel('Maneuver Displacement (m)')
    # ax.set_ylabel('Log Density Multiplier')

    # fig = plt.figure()
    # # plt.contour(X[:,:,5], Y[:,:,5], np.log10(pos_errs.reshape(X.shape)[:,:,5]))
    # plt.savefig('scatter.png')
    # ax.set_xlabel('Altitude offset (m)')
    # ax.set_ylabel('True anomaly offset (deg)')

    # fig = plt.figure()
    # # plt.contour(X[:,:,8], Y[:,:,8], np.log10(pos_errs.reshape(X.shape)[:,:,8]))
    # plt.savefig('scatter.png')
    # ax.set_xlabel('Altitude offset (m)')
    # ax.set_ylabel('True anomaly offset (deg)')
    
    # fig = plt.figure()
    # # plt.contour(X[:,:,3], Y[:,:,3], np.log10(pos_errs.reshape(X.shape)[:,:,3]))
    # plt.savefig('scatter.png')
    # ax.set_xlabel('Altitude offset (m)')
    # ax.set_ylabel('True anomaly offset (deg)')

    
    # fig = plt.figure()
    # #plt.contour(Y[:,5,:], np.log10(Z[:,5,:]), np.log10(pos_errs.reshape(X.shape)[:,5,:]))
    # plt.savefig('scatter.png')
    # ax.set_xlabel('Altitude offset (m)')
    # ax.set_ylabel('True anomaly offset (deg)')
    # plt.show()
    # print('done!')
    pool.close()

def results_to_ranges_and_plot(results_list):
    fig = plt.figure()
    pos_errs = np.empty(len(results_list))
    vel_errs = np.empty(len(results_list))
    init_relpos = np.empty(len(results_list))
    init_relvel = np.empty(len(results_list))
    dens_list = np.empty(len(results_list))
    for ind,result in enumerate(results_list):
        hill_pos = result['dep_hill_nav.r_DC_H']
        hill_vel = result['dep_hill_nav.v_DC_H']
        init_relpos[ind] = np.linalg.norm(hill_pos[1,1:4])
        init_relvel[ind] = np.linalg.norm(hill_vel[1,1:4])
        pos_errs[ind] = np.linalg.norm(hill_pos[-1,1:4])
        vel_errs[ind] = np.linalg.norm(hill_vel[-1,1:4])
        dens_list[ind] = result['dens_mult']


        if ind%5==0:
            plt.plot(hill_pos[:,1],hill_pos[:,2] )
            plt.grid()
            plt.xlabel('Hill X (m)')
            plt.ylabel('Hill Y (m)')
            plt.title(f'Simulation {ind}')

    return init_relpos, init_relvel, pos_errs, vel_errs, dens_list, fig

def comparison_sweep():

    from matplotlib import cm

    with open("tv_lqr_sweep_results.pickle","rb") as fp:
            lqr_sim_results = pickle.load(fp)
    with open("desen_sweep_results.pickle","rb") as fp:
            desen_sim_results = pickle.load(fp)
    with open("ddesen_sweep_results.pickle","rb") as fp:
            ddesen_sim_results = pickle.load(fp)

    lqr_init_range, lqr_init_vel, lqr_err_range, lqr_err_vel, lqr_dens, lqr_fig = results_to_ranges_and_plot(lqr_sim_results)
    desen_init_range, desen_init_vel, desen_err_range, desen_err_vel, desen_dens, desen_fig = results_to_ranges_and_plot(desen_sim_results)
    ddesen_init_range, ddesen_init_vel, ddesen_err_range, ddesen_err_vel, ddesen_dens, desen_fig = results_to_ranges_and_plot(ddesen_sim_results)

    desen_diff = desen_err_range - lqr_err_range
    ddesen_diff = ddesen_err_range - lqr_err_range
    unique_ranges = np.unique(lqr_init_range.round(decimals=2))

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.scatter(lqr_init_range,lqr_dens,np.log10(lqr_err_range))
    ax.scatter(desen_init_range,desen_dens,np.log10(desen_err_range))
    ax.scatter(ddesen_init_range,ddesen_dens,np.log10(ddesen_err_range))
    plt.savefig('scatter.png')
    ax.set_xlabel('Maneuver Displacement (m)')
    ax.set_ylabel('Density Multiplier')

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    lqr_surf = ax.plot_surface(lqr_init_range.reshape([20,20]),lqr_dens.reshape([20,20]),np.log10(lqr_err_range.reshape([20,20])))
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    desen_surf = ax.plot_surface(desen_init_range.reshape([20,20]),desen_dens.reshape([20,20]),np.log10(desen_err_range.reshape([20,20])))
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    desen_surf = ax.plot_surface(ddesen_init_range.reshape([20,20]),ddesen_dens.reshape([20,20]),np.log10(ddesen_err_range.reshape([20,20])))



    fig = plt.figure()
    ax = fig.gca(projection='3d')
    desen_surf = ax.plot_surface(ddesen_init_range.reshape([20,20]),ddesen_dens.reshape([20,20]),desen_diff.reshape([20,20]))
    plt.title('DOC Difference vs. LQR')

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    desen_surf = ax.plot_surface(ddesen_init_range.reshape([20,20]),ddesen_dens.reshape([20,20]),ddesen_diff.reshape([20,20]))
    plt.title('CDOC Difference vs. LQR')

if __name__=='__main__':
    # # drag_sensitivity_analysis('lqr', rerunSims=True)
    drag_sensitivity_analysis('tv_lqr', rerunSims=False)
    drag_sensitivity_analysis('desen', rerunSims=False)
    drag_sensitivity_analysis('ddesen', rerunSims=False)
    comparison_sweep()
    
    plt.show()