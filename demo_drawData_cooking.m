%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo Script for Drawing Data GMM-based LPV_DS Learning for paper:       %
%  'A Physically-Consistent Bayesian Non-Parametric Mixture Model for     %
%   Dynamical System Learning.'                                           %                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2018 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Nadia Figueroa                                                 % 
% email:   nadia.figueroafernandez@epfl.ch                                %
% website: http://lasa.epfl.ch                                            %
%                                                                         %
% This work was supported by the EU project Cogimon H2020-ICT-23-2014.    %
%                                                                         %
% Permission is granted to copy, distribute, and/or modify this program   %
% under the terms of the GNU General Public License, version 2 or any     %
% later version published by the Free Software Foundation.                %
%                                                                         %
% This program is distributed in the hope that it will be useful, but     %
% WITHOUT ANY WARRANTY; without even the implied warranty of              %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General%
% Public License for more details                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Draw 2D Dataset with GUI  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear all; clc

% Define Objects
objects{1}.symbol = 'M';
objects{2}.symbol = 'C';
objects{3}.symbol = 'B';
objects{4}.symbol = 'S';
objects{5}.symbol = 'P';

objects{1}.pos = [4 -2 2 2];
objects{2}.pos = [1 -4 1 1];
objects{3}.pos = [1 1 1 1];
objects{4}.pos = [8 1 1 1];
objects{5}.pos = [8 -4 1 1];

objects{1}.color = [.8 .5 .5];
objects{2}.color = [.3 .8 .3];
objects{3}.color = [1. .8 .3];
objects{4}.color = [.7 .7 .7];
objects{5}.color = [.3 .3 .3];

[fig1, limits1] = draw_background(objects, 1);

% Draw Reference Trajectories
[data, hp] = draw_mouse_data_on_DS(fig1, limits1);

%% Process Drawn Data for Task Specification Inference
save_dir = fileparts(matlab.desktop.editor.getActiveFilename) + "/RawData/";
% nfiles = length(dir) - 2;
delete(save_dir + "*"); nfiles = 0;

for i=1:length(data)
    M_state = zeros(size(data{i}(1, :)));
    M_state(end) = 1;
    states = [M_state];
    for j=1:length(objects)
        rect = objects{j}.pos;
        states = [states;
                  all([data{i}(1:2, :) >= transpose(rect(1:2)); ...
                       data{i}(1:2, :) <= transpose(rect(1:2)) ...
                                        + transpose(rect(3:4))])]
    end
    predicates.WaypointPredicates = states;
    predicates.ThreatPredicates = [];
    predicates.PositionPredicates = zeros(size(states));
    
    predicates_json = jsonencode(predicates);
    fid = fopen(save_dir + "traj"+num2str(nfiles+i,'%02d')+".json",'w');
    fprintf(fid, predicates_json); 
end

%% Segment Drawn Data
save_dir = fileparts(matlab.desktop.editor.getActiveFilename) + "/TrajData/";
delete(save_dir + "*");
save(save_dir+"traj.mat", 'data')

segs = {{}, {}, {}, {}, {}};

for i=1:length(data)
    prev_t = 1;
    state_change{1}.inROI = false; state_change{1}.track = [];
    state_change{2}.inROI = false; state_change{2}.track = [];
    state_change{3}.inROI = false; state_change{3}.track = [];
    state_change{4}.inROI = false; state_change{4}.track = [];
    state_change{5}.inROI = false; state_change{5}.track = [];
    for t=1:size(data{i}, 2)
        for j=1:length(objects)
            rect = objects{j}.pos;
            if ~state_change{j}.inROI
                if all(data{i}(1:2, t) >= transpose(rect(1:2))) && ...
                   all(data{i}(1:2, t) <= transpose(rect(1:2)) ...
                                    + transpose(rect(3:4)))
                    state_change{j}.inROI = true;
                    state_change{j}.track = [state_change{j}.track data{i}(:, t)];
                end
            else
                if all(data{i}(1:2, t) >= transpose(rect(1:2))) && ...
                   all(data{i}(1:2, t) <= transpose(rect(1:2)) ...
                                    + transpose(rect(3:4)))
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
    if prev_t < size(data{i}, 2) % append final segment to mixing
        segs{1}{end+1} = data{i}(:, prev_t:t);
    end
end

for i=1:length(segs)
    save_file = fileparts(matlab.desktop.editor.getActiveFilename) + "/TrajData/traj" + ...
        num2str(i,'%02d') + ".mat";
    seg = segs{i};
    save(save_file, 'seg')
end


%% Plot clustered segments
% for i=1:length(segs)
%     if ~isempty(segs{i})
%         [Data, Data_sh, att, x0_all, dt] = processDrawnData(segs{i});   
%         [h_data, h_att, h_vel] = plot_reference_trajectories_DS(Data, att, 0, 0);
%     end
% end


%% Plot clustered segments
close all;
draw_trajectory(objects, segs, false)
% draw_trajectory(objects, segs, true)
draw_trajectory_with_DS(objects, segs)

function draw_trajectory(objects, segs, draw_segs)
    
    if ~draw_segs
        [fig2, limits2] = draw_background(objects, .2);
    end

    for i=1:length(segs)
        if draw_segs
            [fig2, limits2] = draw_background(objects, .2);
        end
        [Data, Data_sh, att, x0_all, dt] = processDrawnData(segs{i}); % to get att
        
        N = length(segs{i});
        for j=1:N
            h_data = scatter(segs{i}{j}(1,:),segs{i}{j}(2,:), ...
                     'MarkerFaceColor', objects{i}.color, 'MarkerEdgeColor', [1 1 1]); hold on;
            h_att = scatter(att(1),att(2),150,[0 0 0],'d','Linewidth',2); hold on;
        end
    end
end

function draw_trajectory_with_DS(objects, segs)
 
    for i=1:length(segs)       
        [fig2, limits2] = draw_background(objects, .2);      
        [Data, Data_sh, att, x0_all, dt] = processDrawnData(segs{i}); % to get att
        
        % Learn LPV-DS with CORL 2018 Approach
        [ds_gmm, ds_lpv] = learn_lpvds(Data, Data_sh, att); % learn DS
        
        % Visualize Learned DS
        [hs] = plot_ds_model(fig2, ds_lpv, [0 0]', limits2,'medium'); 
         
        % Visualize GMM Cluster Parameters Trajectory Data
        [~, est_labels] =  my_gmm_cluster(Data(1:2,:), ds_gmm.Priors, ds_gmm.Mu, ds_gmm.Sigma, 'hard', []);
        [h_gmm] = plotGMMParameters(Data(1:2,:), est_labels, ds_gmm.Mu, ds_gmm.Sigma, fig2);
        axis(limits2)
        box on
        grid on
        xlabel('$x_1$','Interpreter','LaTex','FontSize',15);
        ylabel('$x_2$','Interpreter','LaTex','FontSize',15);       
    end
end


% for i=1:length(segs)
%     [fig2, limits2] = draw_background(objects, .2);
%     [Data, Data_sh, att, x0_all, dt] = processDrawnData(segs{i});
%     N = length(segs{i});
%     for j=1:N
%         alpha = 0.2 + 0.8*j/N; % alpha for that curve
%         h_data = scatter(segs{i}{j}(1,:),segs{i}{j}(2,:), ...
%                  'MarkerFaceColor', objects{i}.color, 'MarkerEdgeColor', [1 1 1]); hold on;
%         h_att = scatter(att(1),att(2),150,[0 0 0],'d','Linewidth',2); hold on;
%     end
% end

% Position/Velocity Trajectories
% vel_samples = 10; vel_size = 0.5; 
% [h_data, h_att, h_vel] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);


% Extract Position and Velocities
% M          = size(Data,1)/2;    
% Xi_ref     = Data(1:M,:);
% Xi_dot_ref = Data(M+1:end,:);  

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% To use this dataset go to the demo_loadData_*.m scripts %
%% and start with the[Step 2] block of code                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create Figure
function [fig, limits] = draw_background(objs, alpha)
    % draw kitchen background. Objects are structs with rectangle pos and
    % color
   
    fig = figure('Color',[1 1 1]);
    limits = [0 10 -6 4];
    axis(limits)
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.25, 0.55, 0.2646 0.4358]);
    grid on

    for i=1:length(objs)
        rectangle('Position', objs{i}.pos, 'FaceColor', [objs{i}.color alpha], 'EdgeColor',[1 1 1]); hold on;
    end
end