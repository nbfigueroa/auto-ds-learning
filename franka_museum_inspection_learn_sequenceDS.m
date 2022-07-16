%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Learn Sequence of N Segmented Demonstrations as N LPV-DS  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear all; clc

%%%% Set directories (if recordings are not at the same level as catkin_ws
%%%% then this should change!
pkg_dir        = fileparts(matlab.desktop.editor.getActiveFilename);
mat_dir        = pkg_dir + "/SegData-DS/";
mat_files      = dir(strcat(mat_dir,'*.mat'));
latest_mat     = mat_files(end);
plot_ds        = 1; % To plot the learned DS' on the workspace
plot_2d_slices = 1; 
plot_gmm       = 1; % To plot the corresponding GMM params
show_robot     = 0; % To show robot kinematic chain in visualization
is_museum      = 1; %1: MIT Museum Setup, 0: PENN Figueroa Lab Setup

% Load the latest segmented/clustered trajectories
load(strcat(mat_dir,latest_mat.name))
[~, matname, ~] = fileparts(latest_mat.name);

% Get number of DS to learn
N_ds = size(sequence_ds,2);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Step 1: Learn a DS for each trajectory cluster      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
do_seds = 0;
for s=1:N_ds
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  Learning: Learn LPV-DS with CORL 2018 Approach  %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if do_seds % lpvds with symmetric constraint shows more continuous flows than SEDS!
        seds_obj                = 'mse'; % 'mse' or 'likelihood' mse gives the smoothest DS
        K_gmm                   = 3; %0: will do BIC otherwise it will set K to this value
        [ds_gmm, ds_lpv]        = learn_seds(sequence_ds{s}.Data_sh, sequence_ds{s}.att, seds_obj, K_gmm);
        sequence_ds{s}.ds_gmm   = ds_gmm; % GMM parameters
        sequence_ds{s}.ds_lpv   = ds_lpv; % lambda function for DS exection
    else
        lyap_constr = 2;     % 0:'convex':     A' + A < 0 (Proposed in paper)
                             % 2:'non-convex': A'P + PA < -Q given P (Proposed in paper) 
        if s==1
            symm_constr = 1;% 1: Enforce all A's to be symmetric
        else
            symm_constr = 0;     
        end
        [ds_gmm, ds_lpv, A_k, b_k, P] = learn_lpvds(sequence_ds{s}.Data, sequence_ds{s}.Data_sh, sequence_ds{s}.att, gmm_type, lyap_constr, symm_constr); % learn DS
        sequence_ds{s}.ds_gmm         = ds_gmm; % GMM parameters
        sequence_ds{s}.gmm_type       = gmm_type; % GMM Estimation Type
        sequence_ds{s}.lyap_constr    = lyap_constr; % Lyapunov Constraint Type
        sequence_ds{s}.symm_constr    = symm_constr; % Lyapunov Constraint Type
        sequence_ds{s}.ds_lpv         = ds_lpv; % lambda function for DS exection
        sequence_ds{s}.A_k            = A_k; % Linear DS parametersc
        sequence_ds{s}.b_k            = b_k; % Linear DS parameters
        sequence_ds{s}.P              = P;   % Scaling matrix learned for Lyapunov function
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %   Evaluation: Compute Metrics and Visualize Velocities %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%Compute Errors
    % Compute RMSE on training data
    rmse = mean(rmse_error(ds_lpv, sequence_ds{s}.Data(1:3,:), sequence_ds{s}.Data(4:6,:)));
    fprintf('LPV-DS got prediction RMSE on training set: %d \n', rmse);

    % Compute e_dot on training data
    edot = mean(edot_error(ds_lpv, sequence_ds{s}.Data(1:3,:), sequence_ds{s}.Data(4:6,:)));
    fprintf('LPV-DS got e_dot on training set: %d \n', edot);
    
    % FOR DEBUGGING
    % Compare Velocities from Demonstration vs DS
    % h_vel = visualizeEstimatedVelocities(sequence_ds{s}.Data, sequence_ds{s}.ds_lpv);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %   Export Model: Exporting learned DS paramaters for use in ROS/C++/Python %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%   Export DS parameters to Mat/Txt/Yaml files  %%%%%%%%%%%%%%%%%%%
%     % TODO: Write 2 DS models for left and right picking locations!!
%     DS_name = strcat('franka_inspection_ds_museum_', num2str(s));
%     DS_name = strcat(matname, '_DS_', num2str(s));
%     
%     save_lpvDS_to_Mat(DS_name, pkg_dir, sequence_ds{s}.ds_gmm, ... 
%     sequence_ds{s}.A_k, sequence_ds{s}.b_k, sequence_ds{s}.att, sequence_ds{s}.x0_all, sequence_ds{s}.att_all, sequence_ds{s}.dt, ...
%     sequence_ds{s}.P, 2, [])
%  
%     % Yaml format:::: Used in ROS for the lpv-ds C++ library:
%     % https://github.com/nbfigueroa/lpvDS-lib
%     % and python-ROS extension:
%     % https://github.com/nbfigueroa/ds-opt-py
%     save_lpvDS_to_Yaml(DS_name, pkg_dir,  sequence_ds{s}.ds_gmm, sequence_ds{s}.A_k, sequence_ds{s}.att, ...
%         sequence_ds{s}.x0_all, sequence_ds{s}.att_all, sequence_ds{s}.dt, s)

%     Store a copy of the yaml files each with a different attractor for
%     "left" and "right" picking locations and with "latest" name for 
end
save_file = pkg_dir + "/models/"+matname+".mat";
save(save_file, 'sequence_ds')
%% %%%%%%%%%%%%%%%%%%%%%%%%%%
%   Plot Learning Results %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
close all;
for s=1:N_ds
    %%%%%%%%%%%%%%    Plot Streamlines of Each Resulting DS  %%%%%%%%%%%%%%%%%%%
    % Fill in plotting options
    if plot_ds
    % Visualize Workspace
    if exist('fhandle','var');clear fhandle;end
    [~,fhandle] = plotFrankaInspectionWorkspace_Trajectories([], is_museum, show_robot);
    hold on;
    
    % Position/Velocity Trajectories
    vel_samples = 10; vel_size = 0.75; 
    [h_data, h_att, h_vel] = plot_reference_trajectories_DS(sequence_ds{s}.Data, sequence_ds{s}.att, vel_samples, vel_size);
    axis_limits = axis;
    hold on;
    
    ds_plot_options = [];
    ds_plot_options.sim_traj      = 1;            % To simulate trajectories from x0_all
    ds_plot_options.x0_all        = [sequence_ds{s}.x0_all];       % Intial Points
    ds_plot_options.init_type     = 'ellipsoid';   % For 3D DS, to initialize streamlines
                                                   % 'ellipsoid' or 'cube'  
    ds_plot_options.nb_points     = 70;            % No of streamlines to plot (3D)
    ds_plot_options.sample_points = sequence_ds{s}.Data(1:end/2,1:10:end);
    ds_plot_options.plot_vol      = 0;            % Plot volume of initial points (3D)
    ds_plot_options.limits        = axis_limits + [0 0.1 0.0 0.0 0.0 0.20];
    ds_plot_options.fig_handle    = fhandle;
    [hd, hs, hr, x_sim] = visualizeEstimatedDS(sequence_ds{s}.Data(1:end/2,:), sequence_ds{s}.ds_lpv, ds_plot_options);
    hold on;
    
    % Plot Learned GMM Params
    if plot_gmm && ~do_seds
        if exist('h_gmm','var');clear h_gmm;end
        [h_gmm] = plot3DGMMParams(sequence_ds{s}.ds_gmm);
    end
    
    % Compute DTWD between train trajectories and reproductions
    if ds_plot_options.sim_traj
    nb_traj       = size(x_sim,3);
    ref_traj_leng = size(sequence_ds{s}.Data,2)/nb_traj;
    dtwd = zeros(1,nb_traj);
    for n=1:nb_traj
        start_id = round(1+(n-1)*ref_traj_leng);
        end_id   = round(n*ref_traj_leng);
        dtwd(1,n) = dtw(x_sim(:,:,n)',sequence_ds{s}.Data(1:3,start_id:end_id)',20);
    end
    fprintf('LPV-DS got DTWD of reproduced trajectories: %2.4f +/- %2.4f \n', mean(dtwd),std(dtwd));
    end
    
    title_name = strcat('Streamlines of Learned DS s=', num2str(s));
    title(title_name,  'Interpreter', 'LaTex','FontSize',20)
    
    if plot_2d_slices
        fig_2dSlice = figure('Color',[1 1 1], 'Position', [758  551 1163 411]);
        subplot(1,2,1);plot_ds_model_3D_2Dslice(fig_2dSlice, sequence_ds{s}.ds_lpv, sequence_ds{s}.att, axis_limits(3:end), [1 3], []);
        xlabel('$x_1$','Interpreter','LaTex','FontSize',15);
        ylabel('$x_3$','Interpreter','LaTex','FontSize',15);
        subplot(1,2,2);plot_ds_model_3D_2Dslice(fig_2dSlice, sequence_ds{s}.ds_lpv, sequence_ds{s}.att, axis_limits(3:end), [2 3], []);
        xlabel('$x_2$','Interpreter','LaTex','FontSize',15);
        ylabel('$x_3$','Interpreter','LaTex','FontSize',15);
    end
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Simulate Sequence of DS  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; 
close all;
load(save_file);

%%%%%% Plot Mitsubishi Workspace (need rosbag_to_mat repo)
fig_sim = figure('Color',[1 1 1]);

% Get Bowl Transforms (should get them from Apriltags or Optitrack)
[H_pickup_station, H_inspection_tunnel, H_release_station] = computeFrankaInspectionTransforms();
% Plot Franka Inspection Workspace
Objects_APregions = plotFrankaInspectionWorkspace(H_pickup_station, H_inspection_tunnel, H_release_station);
hold on;
title_name = strcat('Sequence of DS simulation');
title(title_name,  'Interpreter', 'LaTex','FontSize',20)
xlabel('$x_1$', 'Interpreter', 'LaTex', 'FontSize',20);
ylabel('$x_2$', 'Interpreter', 'LaTex','FontSize',20);
zlabel('$x_3$', 'Interpreter', 'LaTex','FontSize',20);
xlim([-0.25 1.75])
ylim([-1.1 1.1])
zlim([-1  1.5])
view([62,22])
grid on
axis equal

% Animate Sequence of DS
make_vid = 0;
if make_vid
    vid_name = pkg_dir + "/figs/franka_inspection_sequenceDS_simulation";
    obj = VideoWriter(vid_name, 'MPEG-4');
    obj.Quality = 100;
    obj.FrameRate = round(1/sequence_ds{1}.dt);
    obj.open()
end

% Actual mixing sequence, 2 should be after every scoop
mixing_sequence = [1 2];

% To simulate changes in attractor (bowl location)
bowl_shifts = [];

for s=1:length(mixing_sequence)
    
    % Construct DS with shifted attractor if needed
    s_seq = mixing_sequence(s);
    ds_att_s      = sequence_ds{s_seq}.att;
    
    % Do the shifts here (laterz)
    ds_att_s_hat  = ds_att_s;
    Rot_att = eye(3);
    
    % Store latest attractor to use
    sequence_ds{s_seq}.att_new =  ds_att_s_hat;
    
    % Construct DS with new attractor not seen in data
    ds_lpv_s = @(x)lpv_ds_att(x, ds_att_s, ds_att_s_hat, Rot_att, sequence_ds{s_seq}.ds_gmm, sequence_ds{s_seq}.A_k);    
    
    % Choose starting points for each DS 
    if s_seq> 1
        x0 = sequence_ds{s_seq-1}.att_new;
        if s == length(mixing_sequence)
            x0 = sequence_ds{s-1}.att_new;
        end
    else
        % Start from Robot's init position and then continue from previous
        % DS attractor (that was guaranteed to be reached)
        x0 =  sequence_ds{s_seq}.x0_all(:,1);
    end
    
    % Plot Initial State
    h_x0 = scatter3(x0(1),x0(2),x0(3), 50, [0 1 0]); hold on;
    
    % Get Simulated trajectory
    hold on;
    opt_sim = [];
    opt_sim.dt    = sequence_ds{s_seq}.dt*10;   
    opt_sim.i_max = 10000;
    opt_sim.tol   = 0.010;
    opt_sim.plot  = 0;
    [x_sim, ~]    = Simulation(x0 , [], ds_lpv_s, opt_sim);
 
    % Plot Attractor
    h_att = scatter3(ds_att_s_hat(1),ds_att_s_hat(2),ds_att_s_hat(3), 50, [0 0 0],'filled'); hold on;
    hold on;
    
    %Animate trajectory
    ds_trajectory = animatedline('LineWidth',2);
    hold on;
    for i=1:size(x_sim, 2)
        addpoints(ds_trajectory,x_sim(1,i),x_sim(2,i),x_sim(3,i));
        head = scatter3(x_sim(1,i),x_sim(2,i),x_sim(3,i),'filled','MarkerFaceColor','b');
        drawnow        
        
        if make_vid            
            f = getframe(gcf);
            writeVideo(obj,f);
        else
            pause(sequence_ds{s_seq}.dt*0.001);
        end
        delete(head);        
    end
    
    fprintf("Attractor %d Reached\n", s_seq);
end

if make_vid
   obj.close(); 
end
fprintf("Task Finalized\n", s);
