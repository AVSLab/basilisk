%% LATEX INITIALIZATION:%
clear; 
close all;
clc;
% set(groot,'defaultAxesTickLabelInterpreter','latex');
% set(groot,'defaulttextinterpreter','latex');
% set(groot,'defaultLegendInterpreter','latex');
% 
% set(groot,'DefaultFigureWindowStyle','docked'); % Dock all figures!

set(groot,'defaultAxesFontSize',36); % Font size is monitor-dependent!
set(groot, 'defaultLineLineWidth', 1.5);  % Axes line width

% set(0, 'DefaultAxesXLimMode', 'auto');
set(groot, 'defaultLegendLocation', 'eastoutside');

%%

simCase = input('Choose from case 1-8: ');
savePlot = input('Do you want to save the plots? No - 0, Yes - 1: ');

switch simCase
    case 1
        dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/base_case_1_xr/base_case_1_xr_plot_data.mat';
        r0 = [2 0 0]; v0 = [0 0 0]; 
        controllerOn = 0;
    case 2
        dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/base_case_2_xyrdot/base_case_2_xyrdot_plot_data.mat';
        r0 = [0 0 0]; v0 = [2 2 0];
        controllerOn = 0;
    case 3 
        dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/base_case_3_zr_zrdot/base_case_3_zr_zrdot_plot_data.mat';
        r0 = [0 0 0]; v0 = [0 0 0.1];
        controllerOn = 0;
    case 4 
        dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/base_case_4_xyzr/base_case_4_xyzr_plot_data.mat';
        r0 = [1 2 3]; v0 = [0 0 0];
        controllerOn = 0;
    case 5 
        dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/control_case_5_yr/control_case_5_yr_plot_data.mat';
        rr = [0.0, -2.0, 0.0; 0.0, 2.0, 0.0]; % v0 = [0 0 0];
        controllerOn = 1;
    case 6 
        dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/control_case_6_xyr/control_case_6_xyr_plot_data.mat';
        rr = [3.0, -2.0, 0.0; -3.0, 2.0, 0.0]; % v0 = [0 0 0];
        controllerOn = 1;
    case 7 
        dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/control_case_7_xyzr/control_case_7_xyzr_plot_data.mat';
        rr = [3.0, -2.0, -4.0; -3.0, 2.0, 4.0]; % v0 = [0 0 0];
        controllerOn = 1;
    case 8 
        dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/control_case_8_xyzr_dot/control_case_8_xyzr_dot_plot_data.mat';
        rr = [3.0, -2.0, -4.0; -3.0, 2.0, 4.0]; % v0 = [-0.2, -0.2, -0.2]; (keep)
        controllerOn = 1;
    case 0 
        dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/init_config/init_config_plot_data.mat';
        rr = [0 0 0; 0 0 0]; % TODO
        controllerOn = 1;
end

% Extract file name:
[~, export_case_name, ~] = fileparts(dataPath); 
% export_plots_path = "/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultPlotsMATLAB/" + export_case_name + "/";
export_plots_path = "/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultPlotsMATLAB_Feb2025/" + export_case_name + "/";


%%
% Example of calling the function
% LoadCombine_mat(dataPath);
load(dataPath);
% controllerOn = 0;
% controllerOn = 1;

%% Plots:

% Gather data:
t = timeLineSetMin(1,:)';
dt = (t(2) - t(1)) * 60; % Simulation Frequencey/Rate, Min -> Second
t_zoom_timestamp = 10; % [min]

t_zoom = t(1:t_zoom_timestamp*60/dt); % Zoomed-in time vector for plotting enlargement.
idx_t_zoom = length(t_zoom);

numSC = length(simLength);
sim_dim = simLength(1);

% CW Analytical part:
if ~controllerOn
    mu = 3.986e14;
    a_orbit = 6578e3;
    omega = sqrt(mu/a_orbit^3);
    r_t = CWAnalytical(t*60, r0, v0, omega);
end 

for i=2:numSC % Skip index 1 (SC Index 0) for speed...

% 0) Title text & indexing:
title_text_base = "Spacecraft Index " + num2str(i-1); % + " - ";

% 1) Relative Position - Hill Frame x-y-z positions from Target S/C:
rel_pos = squeeze(dr_index(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!

fig(1) = figure('Name','rel_pos'); % apply_custom_style();
subplot(3,1,1); plot(t, rel_pos(:,1)); hold on;
title(title_text_base, "Relative Position from Target S/C (Hill-frame)");
xlabel('Time $t \ [min]$'); 
ylabel('$x \ [m]$');
if ~controllerOn && i~=1
    plot(t,r_t(:,1)); %legend('Basilisk Simulation','CW-Analytical');
elseif controllerOn && i~=1
    yline(rr(i-1,1), 'r--', '$x_r$', 'LineWidth',2, 'FontSize', 20, 'LabelHorizontalAlignment','left'); %legend('Basilisk Simulation','CW-Analytical');
    axLim = 1.2*max(abs(rel_pos(:,1)));
    ylim([-axLim axLim]);
    legend('Controlled $x$','Reference $x_r$');
end

subplot(3,1,2); plot(t, rel_pos(:,2)); hold on; 
xlabel('Time $t \ [min]$'); 
ylabel('$y \ [m]$');
if ~controllerOn && i~=1
    plot(t,r_t(:,2)); legend('Basilisk Simulation','CW-Analytical','Location','northeast');
elseif controllerOn && i~=1
    yline(rr(i-1,2), 'r--', '$y_r$', 'LineWidth',2, 'FontSize', 20, 'LabelHorizontalAlignment','left'); 
    axLim = 1.2*max(abs(rel_pos(:,2)));
    ylim([-axLim axLim]);
    legend('Controlled $y$','Reference $y_r$');
end

subplot(3,1,3); plot(t, rel_pos(:,3)); hold on; 
xlabel('Time $t \ [min]$'); 
ylabel('$z \ [m]$');
if ~controllerOn && i~=1
    plot(t,r_t(:,3));
elseif controllerOn && i~=1
    yline(rr(i-1,3), 'r--', '$z_r$', 'LineWidth',2, 'FontSize', 20, 'LabelHorizontalAlignment','left'); %legend('Basilisk Simulation','CW-Analytical');
    axLim = 1.2*max(abs(rel_pos(:,3)));
    ylim([-axLim axLim]);
    legend('Controlled $z$','Reference $z_r$');
end

% 2) MRP Pointing Errors + Angular Velocity (Body w.r.t. Hill frame):
sigma_BH = squeeze(dataSigmaBR(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!
omega_BH = squeeze(dataOmegaBR(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!

fig(2) = figure('Name','sigma_BH_omega_BH'); % apply_custom_style();
subplot(2,1,1); plot(t_zoom, sigma_BH(1:idx_t_zoom,:));
xlabel('Time $t \ [min]$'); 
ylabel('$\sigma_{B/H} \ [rad]$');
legend('$\sigma_{1}$', '$\sigma_{2}$', '$\sigma_{3}$');
title(title_text_base, "MRP Pointing Error - Body w.r.t. Hill-frame");

subplot(2,1,2); plot(t_zoom, omega_BH(1:idx_t_zoom,:));
xlabel('Time $t \ [min]$'); 
ylabel('$\omega_{B/H} \ [rad/s]$');
legend('$\omega_{x}$', '$\omega_{y}$', '$\omega_{z}$');
title(title_text_base, "Angular Velocity - Body w.r.t. Hill-frame");

if controllerOn
% 3) Cmd Force:
    F_cmd = squeeze(dataCmdForce(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!
    
    fig(3) = figure('Name','F_cmd'); % apply_custom_style();
    plot(t,F_cmd); 
    xlabel('Time $t \ [min]$'); 
    ylabel('Commanded Force $F_{cmd} \ [N]$');
    title(title_text_base, "Commanded Control Force");
    legend('$F_x$','$F_y$','$F_z$');
       
% 4) Thruster Actuations:
    % Total Impulse P_tot = summation(F*dt), dt = sampling time*60 (minute):
    F_thrusters = squeeze(dataThrust(i,:,:))'; % MATLAB `squeeze()` to reduce a dimension!
    % dt = (t(2) - t(1)) * 60; % Min -> Second
    F_tot = sum(sum(F_thrusters)); % Discrete sum of forces
    P_tot = F_tot * dt
    
    fig(4) = figure('Name','F_thrusters'); % apply_custom_style();
    plot(t,F_thrusters); 
    xlabel('Time $t \ [min]$'); 
    ylabel('Actuated Force $F_{thrusters} \ [N]$');
    title(title_text_base, "Thrusters Actuated Force");
    legend('$+x$','$-x$','$+y$','$-y$','$+z$','$-z$'); % Currently hard-coded for 6 thrusters.
    
    P_tot_txt = "Total Impulse: $P_{tot} = $ " + num2str(P_tot) + " $Ns$";
    text(t(end)/2,max(max(F_thrusters))/2,P_tot_txt,'HorizontalAlignment','center','FontSize',30);

% 5) Combined - Cmd v.s. Applied (summing +/- components per x,y,z axis) Forces
    % Reusing `F_cmd` & `F_thrusters`:
    F_applied = [ sum(F_thrusters(:,1:2),2),... 
                  sum(F_thrusters(:,3:4),2), ...
                  sum(F_thrusters(:,5:6),2) ];
    
    P_tot_3_axis = sum(F_applied) * dt;
    P_tot_txt_x = "$P_{tot,x} = $ " + num2str(round(P_tot_3_axis(1),2)) + " $Ns$";
    P_tot_txt_y = "$P_{tot,y} = $ " + num2str(round(P_tot_3_axis(2),2)) + " $Ns$";
    P_tot_txt_z = "$P_{tot,z} = $ " + num2str(round(P_tot_3_axis(3),2)) + " $Ns$";

    fig(5) = figure('Name','F_applied_vs_F_cmd'); % apply_custom_style();
    subplot(3,1,1); plot(t, F_applied(:,1), 'g-', t, F_cmd(:,1), 'r--');
    % For impulse display on legend:
    hold on; plot(NaN,NaN,'ko'); %,'Visible','off'
    title(title_text_base, "Thruster Applied v.s. Commanded Control Force");
    xlabel('Time $t \ [min]$'); 
    ylabel('$F_x \ [N]$');
    legend('$F_{applied, x}$','$F_{cmd, x}$',P_tot_txt_x);

    subplot(3,1,2); plot(t, F_applied(:,2), 'g-', t, F_cmd(:,2), 'r--');
    % For impulse display on legend:
    hold on; plot(NaN,NaN,'ko'); %,'Visible','off'
    xlabel('Time $t \ [min]$'); 
    ylabel('$F_y \ [N]$');
    legend('$F_{applied, y}$','$F_{cmd, y}$',P_tot_txt_y);

    subplot(3,1,3); plot(t, F_applied(:,3), 'g-', t, F_cmd(:,3), 'r--');
    % For impulse display on legend:
    hold on; plot(NaN,NaN,'ko'); %,'Visible','off'
    xlabel('Time $t \ [min]$'); 
    ylabel('$F_z \ [N]$');
    legend('$F_{applied, z}$','$F_{cmd, z}$',P_tot_txt_z);

% 6) Cmd Torque:
    L_cmd = squeeze(dataUsReq(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!
    
    fig(6) = figure('Name','L_cmd'); % apply_custom_style();
    plot(t_zoom,L_cmd(1:idx_t_zoom,:)); 
    xlabel('Time $t \ [min]$'); 
    ylabel('Commanded Torque $L_{cmd} \ [Nm]$');
    title(title_text_base, "Commanded Control Torque");
    legend('$L_x$','$L_y$','$L_z$');

% 7) RW Actuations (motorTorque): -> applied = actuated
    L_RW = squeeze(dataRW(i,:,:))'; % MATLAB `squeeze()` to reduce a dimension!
    
    fig(7) = figure('Name','L_RW'); % apply_custom_style();
    plot(t_zoom,L_RW(1:idx_t_zoom,:)); 
    title(title_text_base, "RW Applied Torque");
    xlabel('Time $t \ [min]$'); 
    ylabel('Applied Torque $L_{RW} \ [Nm]$');
    legend('RW $x$','RW $y$','RW $z$'); % Currently hard-coded for 3 RWs.

% 8) Combined - Cmd v.s. Applied (summing +/- components per x,y,z axis) Forces
    % Reusing `L_cmd` & `L_RW`:
    % L_applied = [ sum(L_RW(:,1:2),2),... 
    %               sum(L_RW(:,3:4),2), ...
    %               sum(L_RW(:,5:6),2) ];
    L_applied = L_RW; % 3 RWs, hardcoded for now.

    fig(8) = figure('Name','L_applied_vs_L_cmd'); % apply_custom_style();
    subplot(3,1,1); plot(t_zoom, L_applied(1:idx_t_zoom,1), 'g-', t_zoom, L_cmd(1:idx_t_zoom,1), 'r--');
    title(title_text_base, "Applied v.s. Commanded Control Torque");
    xlabel('Time $t \ [min]$'); 
    ylabel('$L_x \ [Nm]$');
    legend('$L_{applied, x}$','$L_{cmd, x}$');
    subplot(3,1,2); plot(t_zoom, L_applied(1:idx_t_zoom,2), 'g-', t_zoom, L_cmd(1:idx_t_zoom,2), 'r--');
    xlabel('Time $t \ [min]$'); 
    ylabel('$L_y \ [Nm]$');
    legend('$L_{applied, y}$','$L_{cmd, y}$');
    subplot(3,1,3); plot(t_zoom, L_applied(1:idx_t_zoom,3), 'g-', t_zoom, L_cmd(1:idx_t_zoom,3), 'r--');
    xlabel('Time $t \ [min]$'); 
    ylabel('$L_z \ [Nm]$');
    legend('$L_{applied, z}$','$L_{cmd, z}$');

end

% End loop:
    % savefig(fig_index_array);
    % exportFigToPath(export_plots_path, numSC-1, export_case_name, fig_index_array);
    if savePlot
        exportFigToPath(export_plots_path, i-1, fig);
        for n=1:length(fig)
            figure(fig(n)); 
            exportPlotToPath(export_plots_path, i-1, fig(n).Name);
            if (n == 1 || n == 5) && controllerOn
                figNameExport_1 = set_fig_xlim(fig(n), [0 t_zoom_timestamp]);
                exportPlotToPath(export_plots_path, i-1, figNameExport_1);
    
                figNameExport_2 = set_fig_xlim(fig(n), [t_zoom_timestamp 90]);
                exportPlotToPath(export_plots_path, i-1, figNameExport_2);
            end
        end
        close all;
    end
end

%% Functions:
function apply_custom_style()
    % Set global figure and axes properties
    set(gcf, 'FontSize', 16);  % Axes font size
    set(gcf, 'LineWidth', 1.5);  % Axes line width
    % set(gcf, 'Position', [100, 100, 800, 600]);  % Figure size
end

function exportPlotToPath(export_path_base, SCIndex, filename)
    export_path = export_path_base + "index_" + num2str(SCIndex);
    if ~exist(export_path, 'dir')
    % If the folder does not exist, create it
        mkdir(export_path);
        disp(['Created export folder: ' export_path]);
    end 
    
    % Assuming everytime `figure;` is called in every plot
    % apply_custom_style();

    % Save the plots into .eps and .fig:
    % savePlotFile = export_path + "/" + filename + ".eps";
    savePlotFile = export_path + "/" + filename + ".pdf";
    exportgraphics(gcf, savePlotFile, 'Resolution', 300);

    % Close the figure upon saving:
    % close all;
end

% Save an array of figures `fig` into ONE single .fig file:
function exportFigToPath(export_path_base, SCIndex, fig)
    export_path = export_path_base + "index_" + num2str(SCIndex);
    if ~exist(export_path, 'dir')
    % If the folder does not exist, create it
        mkdir(export_path);
        disp(['Created export folder: ' export_path]);
    end 
    
    % Assuming everytime `figure;` is called in every plot
    for i=1:length(fig)
        fig(i); % apply_custom_style();
    end

    % Save the plots into .eps and .fig:
    savePlotFile = export_path + "/" + "index_" + num2str(SCIndex) + ".fig"
    savefig(fig, savePlotFile);
    % close(fig);
    % Close the figure upon saving:
    % close all;
end

% Function to adjust xlim for all subplots and export
function figName = set_fig_xlim(fig_handle, x_limits)
    subplots = findobj(fig_handle, 'Type', 'axes'); % Find all subplot axes in the figure
    for j = 1:numel(subplots)
        xlim(subplots(j), x_limits); % Apply xlim to each subplot
        ylim(subplots(j), "auto");
    end
    figName = [fig_handle.Name mat2str(x_limits)]
    % exportgraphics(fig_handle, file_name, 'ContentType', 'vector');
end

function LoadCombine_mat(folder_path)
    % folder_path: Path to the folder containing the .mat files (files are named index_0_plot_data.mat, index_1_plot_data.mat, etc.).

    % Find all .mat files in the folder matching the pattern 'index_*_plot_data.mat'
    mat_files = dir(fullfile(folder_path, 'index_*_plot_data.mat'));
    
    % Check if there are any files
    if isempty(mat_files)
        error('No .mat files found in the specified folder with the pattern index_*_plot_data.mat');
    end
    
    % Define an empty structure to hold the combined variables
    combined_variables = struct();
    
    % Loop over each file found
    for i = 1:length(mat_files)
        % Create the full path to the file
        filename = fullfile(folder_path, mat_files(i).name);
        
        % Load the .mat file
        data = load(filename);
        
        % Loop over each variable in the loaded data
        var_names = fieldnames(data);
        for j = 1:length(var_names)
            var_name = var_names{j};
            var_value = data.(var_name);
            
            % If this variable has already been initialized, concatenate along a new dimension
            if isfield(combined_variables, var_name)
                combined_variables.(var_name) = cat(ndims(combined_variables.(var_name)) + 1, combined_variables.(var_name), var_value);
            else
                % Initialize the variable with the current value from the file
                combined_variables.(var_name) = var_value;
            end
        end
    end
    
    % Load the combined variables directly into the MATLAB workspace
    var_names_combined = fieldnames(combined_variables);
    for k = 1:length(var_names_combined)
        assignin('base', var_names_combined{k}, combined_variables.(var_names_combined{k}));
    end

    % Save the combined variables into a single .mat file for plotting or further analysis
    % save('combined_data.mat', '-struct', 'combined_variables');
    
    % % Example plot: Assuming one of the variables is 'data1'
    % if isfield(combined_variables, 'data1')
    %     figure;
    %     % Here, you may want to adjust plotting according to the dimensionality
    %     plot(squeeze(combined_variables.data1));
    %     title('Plot of Combined data1 Variable');
    % else
    %     disp('No variable named ''data1'' found.');
    % end
end

function r_t = CWAnalytical(t, r0, v0, omega)
    rx_t = 4*r0(1) + 2*v0(2)/omega + v0(1)*sin(omega * t)/omega - ( 3*r0(1) + 2*v0(2)/omega )*cos(omega * t);
    ry_t = 2*v0(1)*cos(omega * t)/omega + (6*r0(1) + 4*v0(2)/omega)*sin(omega * t) - ( 6*omega*r0(1) + 3*v0(2) )*t - 2*v0(1)/omega + r0(2);
    % -2*r0(1)*sin(omega * t) + r0(2);
    rz_t = r0(3)*cos(omega * t) + v0(3)*sin(omega * t)/omega;

    r_t = [rx_t ry_t rz_t];
end 