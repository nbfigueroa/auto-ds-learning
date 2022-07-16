%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%        Script for Segmenting Trajectories from Latest Recording         %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 1: Load Demonstrations Extracted from ROSBags  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear all; clc

%%%% Set directories (if recordings are not at the same level as catkin_ws
%%%% then this should change!
data_dir          = '../../../museum_recordings/mat/';
mat_files         = dir(strcat(data_dir,'*.mat'));
latest_mat        = mat_files(end);
show_robot        = 0; % To show robot kinematic chain in visualization
is_museum         = 1; %1: MIT Museum Setup, 0: PENN Figueroa Lab Setup
do_plot_seg_raw   = 0; % Plot segmented raw labeled trajectories on workspace
plot_proc_segs    = 1; % Plot processed segments on workspace
show_vel_profiles = 1; % Show velocity profiles of final segmented data

% This will load the latest trajectory recorded
load(strcat(data_dir,latest_mat.name));
[~, matname, ~] = fileparts(latest_mat.name);
data_raw = data_ee_pose.pose(1:3,:);
dt_raw   = data_ee_pose.dt;

viz_sample_step  = ceil(length(data_raw)/2000); % Sample step to visualize raw data
% sample_step = 10; % The data is recorded at 1kHZ this is ALOT of data
% that is not necessary, this value will downsample the data to get
% efficient learning, dt is scaled accordingly to learn the model correctly

%%%%%% Plot Franka Inspection Workspace (need rosbag_to_mat repo)
Objects_APregions = plotFrankaInspectionWorkspace_Trajectories(data_raw(:,1:viz_sample_step:end), is_museum, show_robot);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 2: Segment Demonstrations by tracking APRegion State-Change (Felix CORL'2022) %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Process Collected Data for Task Specification Inference
save_dir = fileparts(matlab.desktop.editor.getActiveFilename) + "/RawData/";
delete(save_dir + "*"); 
% nfiles = 0;

states = [];
for j=1:length(Objects_APregions)
    hull_V = Objects_APregions{j}.V;                                  
    states = [states; inhull(data_raw(1:3,:)',hull_V')'];                                                                
end
predicates.WaypointPredicates = states;
predicates.ThreatPredicates = [];
predicates.PositionPredicates = zeros(size(states));

predicates_json = jsonencode(predicates);
fid = fopen(save_dir + matname+"_traj.json",'w');
fprintf(fid, predicates_json); 

% Segment Collected Data based on APRegion State-Tracking
save_dir = fileparts(matlab.desktop.editor.getActiveFilename) + "/TrajData/";
save(save_dir + matname+"_traj.mat", 'data_raw')

% This should be done in a different way, for now it's fixed to 2 APRegions
% that denote the pick and release station
segs = {{}, {}};
prev_t = 1;
state_change{1}.inROI = false; state_change{1}.track = [];
state_change{2}.inROI = false; state_change{2}.track = [];
for t=1:size(data_raw, 2)
    for j=1:length(Objects_APregions)
        hull_V = Objects_APregions{j}.V; 
        if ~state_change{j}.inROI
            if inhull(data_raw(1:3,t)',hull_V')
                state_change{j}.inROI = true;
                state_change{j}.track = [state_change{j}.track data_raw(:, t)];
            end
        else
            if inhull(data_raw(1:3,t)',hull_V')
                state_change{j}.track = [state_change{j}.track data_raw(:, t)];
            else
                mid = round(size(state_change{j}.track, 2)/2);
                segs{j}{end+1} = data_raw(:, prev_t:t-mid);
                prev_t = t-mid+1;
                state_change{j}.inROI = false;
                state_change{j}.track = [];
            end
        end
    end

end   

for i=1:length(segs)
    save_file = fileparts(matlab.desktop.editor.getActiveFilename) + "/TrajData/"+ matname + ...
        num2str(i,'_traj_%02d') + ".mat";
    seg = segs{i};
    save(save_file, 'seg')
end

if do_plot_seg_raw
    %%%%%% Plot All Segmented Trajectories on Workspace
    % Visualize Workspace
    plotFrankaInspectionWorkspace_Trajectories([], is_museum, show_robot);
    
    % Plot Segmented Data Over Workspace
    N_segs = size(segs,2);
    for ii=1:N_segs
        segs_color = [1 rand rand];    
        segs_ii = segs{ii};
        for jj=1:size(segs_ii,2)
            % Plot Cartesian Trajectories
            scatter3(segs_ii{jj}(1,1:viz_sample_step:end), segs_ii{jj}(2,1:viz_sample_step:end), segs_ii{jj}(3,1:viz_sample_step:end), 7.5, 'MarkerEdgeColor','k','MarkerFaceColor',segs_color); 
            hold on;
        end
    end
    title('Franka Segmented End-Effector Trajectories',  'Interpreter', 'LaTex','FontSize',20)
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step 3: Smoothen and process segmented trajectories for DS learning (Nadia CORL'2018)  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save_dir = fileparts(matlab.desktop.editor.getActiveFilename) + "/SegData-DS/";
nfiles = 0;

%%%%%% Process Raw Segmented Data
sequence_demos = {};
N_segs = size(segs,2);
proc_sample_step = floor(viz_sample_step/2); %This should give at most 1000 samples for trajectory
dt_proc = dt_raw*proc_sample_step;

for ii=1:N_segs

    segs_ii     = segs{ii};
    for jj=1:size(segs_ii,2)
        segs_ii{jj} = smoothSegmentedTrajectoryDS(segs_ii{jj}', dt_proc, proc_sample_step);
    end
 
    % Construct Data Structure for DS Learning
    sequence_ds{ii}.data    = segs_ii;
    sequence_ds{ii}.dt      = dt_proc;
    
    % Process Segmented Data for DS Learning (New)    
    [Data, Data_sh, att, att_all, x0_all] = processSegmentedData(sequence_ds{ii}.data);
    sequence_ds{ii}.Data    = Data;
    sequence_ds{ii}.Data_sh = Data_sh;
    sequence_ds{ii}.att     = att;
    sequence_ds{ii}.att_all = att_all;
    sequence_ds{ii}.x0_all  = x0_all;

    if plot_proc_segs
        % Visualize Workspace
        plotFrankaInspectionWorkspace_Trajectories([], is_museum, show_robot);
        segs_color = [rand rand rand];
        for jj=1:size(segs_ii,2)
            % Plot Cartesian Trajectories
            scatter3(segs_ii{jj}(1,:), segs_ii{jj}(2,:), segs_ii{jj}(3,:), 7.5, 'MarkerEdgeColor','k','MarkerFaceColor',[rand rand rand]); 
            hold on;
        end
        title(strcat('Segmented Trajectories for DS:', num2str(ii)),  'Interpreter', 'LaTex','FontSize',20)
    end

    % For Debugging: Checking the velocity profiles!
    figure('Color',[1 1 1]); 
    if show_vel_profiles
        subplot(1,2,1); plot(segs_ii{1}(4:end,:)')
        subplot(1,2,2); plot(segs_ii{2}(4:end,:)')
    end
end

save_file = fileparts(matlab.desktop.editor.getActiveFilename) + "/SegData-DS/"+matname+"_traj.mat";
save(save_file, 'sequence_ds', 'dt_proc')
