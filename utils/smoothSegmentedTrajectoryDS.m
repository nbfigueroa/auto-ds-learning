function [seg] = smoothSegmentedTrajectoryDS(seg, dt, sample_step)

vel_cutting = 0.01;
pos_cutting = 1e-3;


% Sample trajectories
seg = seg(1:sample_step:end,:);
dt = dt*sample_step;

% Filter Trajectories and Compute Derivativess
% dx_nth = sgolay_time_derivatives(seg, dt, 1, 2, 15);
dx_nth = sgolay_time_derivatives(seg, dt, 3, 5, 15);
Xi_ref_tmp     = dx_nth(:,:,1)';
Xi_dot_ref_tmp = dx_nth(:,:,2)'; 

% trimming demonstrations (Removing measurements with zero velocity)
zero_vel_idx = find(vecnorm(Xi_dot_ref_tmp) < 5e-4);
fprintf ('Segment: %d datapoints removed ... \n', length(zero_vel_idx));
Xi_ref_tmp(:,zero_vel_idx) = [];
Xi_dot_ref_tmp(:,zero_vel_idx) = [];  

% Check final measurements                        
[idx_end, dist_end] = knnsearch( Xi_ref_tmp(:,end-10:end)', Xi_ref_tmp(:,end)', 'k',10);
idx_zeros = idx_end(dist_end < pos_cutting);
Xi_ref_tmp(:,idx_zeros)=[];
Xi_dot_ref_tmp(:,idx_zeros)=[];

% Make last measurment 0 velocity and scale the previous
Xi_dot_ref_tmp(:,end)   = zeros(size(Xi_ref_tmp,1),1);
for k =1:10
    Xi_dot_ref_tmp(:,end-k) = (Xi_dot_ref_tmp(:,end-k) +  Xi_dot_ref_tmp(:,end-(k-1)))/2;
end

% Final Smoothed and Segmented Trajectories
seg  = [Xi_ref_tmp; Xi_dot_ref_tmp];

end