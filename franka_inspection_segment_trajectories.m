%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 1: Load Demonstrations Extracted from ROSBags  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear all; clc

% Load Data from Mat File
mat_data_dir = '../../recordings/icra-22-demo/mat/';
matfile = strcat(mat_data_dir,'demos_icra2022_raw_data.mat');
load(matfile);

%%%%%% Plot Franka Inspection Workspace (need rosbag_to_mat repo)
figure('Color',[1 1 1])

% Get Station Transforms (should get them from Apriltags or Optitrack)
[H_pickup_station, H_inspection_tunnel, H_release_station] = computeFrankaInspectionTransforms();

% Plot Franka Inspection Workspace
Objects_APregions = plotFrankaInspectionWorkspace(H_pickup_station, H_inspection_tunnel, H_release_station);

% Extract data
N = size(data_ee_pose,2);
sample_step  = 2;
data    = {};
dt_data = [];
for ii=1:N
    data{ii} = data_ee_pose{ii}.pose(1:3,:);
    dt_data = [dt_data data_ee_pose{ii}.dt]; 
    % Extract desired variables
    hand_traj  = data{ii}(:,1:sample_step:end);   

    % Plot Cartesian Trajectories
    scatter3(hand_traj(1,:), hand_traj(2,:), hand_traj(3,:), 7.5, 'MarkerEdgeColor','k','MarkerFaceColor',[rand rand rand]); hold on;
    hold on;
end
dt = mean(dt_data);
xlabel('$x_1$', 'Interpreter', 'LaTex', 'FontSize',20);
ylabel('$x_2$', 'Interpreter', 'LaTex','FontSize',20);
zlabel('$x_3$', 'Interpreter', 'LaTex','FontSize',20);
title('Franka End-Effector Trajectories',  'Interpreter', 'LaTex','FontSize',20)
xlim([-0.25 1.75])
ylim([-1.1 1.1])
zlim([-1  1.5])
view([62,22])
grid on
axis equal

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 2: Segment Demonstrations by tracking APRegion state-change (Felix)  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Process Collected Data for Task Specification Inference
save_dir = fileparts(matlab.desktop.editor.getActiveFilename) + "/RawData/";
delete(save_dir + "*"); 
nfiles = 0;

for i=1:length(data)
    states = [];
    for j=1:length(Objects_APregions)
        hull_V = Objects_APregions{j}.V;                                  
        states = [states; inhull(data{i}(1:3,:)',hull_V')'];                                                                
    end
    predicates.WaypointPredicates = states;
    predicates.ThreatPredicates = [];
    predicates.PositionPredicates = zeros(size(states));
    
    predicates_json = jsonencode(predicates);
    fid = fopen(save_dir + "franka_Inspection_traj"+num2str(nfiles+i,'%02d')+".json",'w');
    fprintf(fid, predicates_json); 
end

% Segment Collected Data based on APRegion State-Tracking
save_dir = fileparts(matlab.desktop.editor.getActiveFilename) + "/TrajData/";
% delete(save_dir + "*");
save(save_dir + "franka_inspection_traj.mat", 'data')

% This should be done in a different way!
segs = {{}, {}};

for i=1:length(data)
    prev_t = 1;
    state_change{1}.inROI = false; state_change{1}.track = [];
    state_change{2}.inROI = false; state_change{2}.track = [];
    state_change{3}.inROI = false; state_change{3}.track = [];
    for t=1:size(data{i}, 2)
        for j=1:length(Objects_APregions)
            hull_V = Objects_APregions{j}.V; 
            if ~state_change{j}.inROI
                if inhull(data{i}(1:3,t)',hull_V')
                    state_change{j}.inROI = true;
                    state_change{j}.track = [state_change{j}.track data{i}(:, t)];
                end
            else
                if inhull(data{i}(1:3,t)',hull_V')
                    state_change{j}.track = [state_change{j}.track data{i}(:, t)];
                else
                    mid = round(size(state_change{j}.track, 2)/2);
                    segs{j}{end+1} = data{i}(:, prev_t:t-mid);
                    prev_t = t-mid+1;
                    state_change{j}.inROI = false;
                    state_change{j}.track = [];
                end
            end
        end
    end   
end

for i=1:length(segs)
    save_file = fileparts(matlab.desktop.editor.getActiveFilename) + "/TrajData/franka_inspection_traj" + ...
        num2str(i,'%02d') + ".mat";
    seg = segs{i};
    save(save_file, 'seg')
end


%% %%%% Plot All Segmented Trajectories on Mitsubishi Workspace (need rosbag_to_mat repo)
figure('Color',[1 1 1])
% Plot Franka Inspection Workspace
plotFrankaInspectionWorkspace(H_pickup_station, H_inspection_tunnel, H_release_station);

% Plot Segmented data
N_segs = size(segs,2);
% sample_step  = 1;
for ii=1:N_segs
    segs_color = [rand rand rand];    
    segs_ii = segs{ii};
    for jj=1:size(segs_ii,2)
        % Plot Cartesian Trajectories
        scatter3(segs_ii{jj}(1,:), segs_ii{jj}(2,:), segs_ii{jj}(3,:), 7.5, 'MarkerEdgeColor','k','MarkerFaceColor',segs_color); 
        hold on;
    end
end
xlabel('$x_1$', 'Interpreter', 'LaTex', 'FontSize',20);
ylabel('$x_2$', 'Interpreter', 'LaTex','FontSize',20);
zlabel('$x_3$', 'Interpreter', 'LaTex','FontSize',20);
title('Franka Segmented End-Effector Trajectories',  'Interpreter', 'LaTex','FontSize',20)
xlim([-0.25 1.75])
ylim([-1.1 1.1])
zlim([-1  1.5])
view([62,22])
grid on
axis equal

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Smoothen and process segmented trajectories for DS learning (Nadia CORL'2018)  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save_dir = fileparts(matlab.desktop.editor.getActiveFilename) + "/SegData-DS/";
% delete(save_dir + "*"); 
nfiles = 0;

%%%%%% Plot Individual Segmented Trajectories on Mitsubishi Workspace (need rosbag_to_mat repo)
% Plot Segmented data
plot_segments  = 1;
% sample_step    = 1;
sequence_demos = {};
N_segs = size(segs,2);
dt_ = mean(dt_data)*sample_step;

if plot_segments;close all;end
if N_segs > 4
    ss = 2;
else
    ss = 1;
end
for ii=1:N_segs
    if plot_segments
        figure('Color',[1 1 1])
        % Plot Franka Inspection Workspace
        plotFrankaInspectionWorkspace(H_pickup_station, H_inspection_tunnel, H_release_station);
        segs_color = [rand rand rand];    
    end
    segs_ii     = segs{ii};

    for jj=1:size(segs_ii,2)
        segs_ii{jj} = smoothSegmentedTrajectoryDS(segs_ii{jj}', dt_);
        if plot_segments
            % Plot Cartesian Trajectories
            scatter3(segs_ii{jj}(1,:), segs_ii{jj}(2,:), segs_ii{jj}(3,:), 7.5, 'MarkerEdgeColor','k','MarkerFaceColor',[rand rand rand]); 
            hold on;
        end
    end
    
    if plot_segments
        xlabel('$x_1$', 'Interpreter', 'LaTex', 'FontSize',20);
        ylabel('$x_2$', 'Interpreter', 'LaTex','FontSize',20);
        zlabel('$x_3$', 'Interpreter', 'LaTex','FontSize',20);
        title_string = strcat('Segmented and Smoothed Trajectories for DS s=', num2str(ii)); 
        title(title_string,  'Interpreter', 'LaTex','FontSize',20)
        xlim([-0.25 1.75])
        ylim([-1.1 1.1])
        zlim([-1  1.5])
        grid on
        axis equal
        view([62,22])
    end
    
    
    % Construct Data Structure for DS Learning
    sequence_ds{ii}.data = segs_ii;
    sequence_ds{ii}.dt = dt_;
    
    % Process Segmented Data for DS Learning (New)    
    [Data, Data_sh, att, att_all, x0_all] = processSegmentedData(sequence_ds{ii}.data);
    sequence_ds{ii}.Data    = Data;
    sequence_ds{ii}.Data_sh = Data_sh;
    sequence_ds{ii}.att     = att;
    sequence_ds{ii}.att_all = att_all;
    sequence_ds{ii}.x0_all  = x0_all;
end

save_file = fileparts(matlab.desktop.editor.getActiveFilename) + "/SegData-DS/franka_Inspection_traj.mat";
save(save_file, 'sequence_ds', 'dt')
