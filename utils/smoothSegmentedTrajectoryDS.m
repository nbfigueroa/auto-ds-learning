function [seg] = smoothSegmentedTrajectoryDS(seg, dt, sample_step)

pos_cutting = 1e-4;

% Sample trajectories
seg = seg(1:sample_step:end,:);

% Filter Trajectories and Compute Derivativess
%   nth_order :     max order of the derivatives 
%   n_polynomial :  Order of polynomial fit
%   window_size :   Window length for the filter
nth_order    = 1;
n_polynomial = 2;
window_size  = ceil(size(seg,1)*0.05); % 31 seems to work in general
if floor(window_size/2)==window_size/2
  window_size = window_size + 1;
end

dx_nth = sgolay_time_derivatives(seg, dt, nth_order, n_polynomial, window_size);
Xi_ref_tmp     = dx_nth(:,:,1)';
Xi_dot_ref_tmp = dx_nth(:,:,2)'; 

% trimming demonstrations (Removing measurements with near-zero velocity norm)
zero_vel_idx = find(vecnorm(Xi_dot_ref_tmp) < 0.075);
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
for k =1:window_size
    Xi_dot_ref_tmp(:,end-k) = (Xi_dot_ref_tmp(:,end-k) +  Xi_dot_ref_tmp(:,end-(k-1)))/2;
end

% Cut-off first part of trajectories and remove if velocity is too slow!
init_trim = ceil(size(Xi_dot_ref_tmp,2)*0.10);
zero_vel_idx = find(vecnorm(Xi_dot_ref_tmp(:,1:init_trim)) < 0.1);
fprintf ('Segment: %d datapoints removed ... \n', length(zero_vel_idx));
Xi_ref_tmp(:,zero_vel_idx) = [];
Xi_dot_ref_tmp(:,zero_vel_idx) = []; 


% Final Smoothed and Segmented Trajectories
seg  = [Xi_ref_tmp; Xi_dot_ref_tmp];

end