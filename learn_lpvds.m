function [ds_gmm, ds_lpv, A_k, b_k, P_opt] = learn_lpvds(Data, Data_sh, att, gmm_type, lyap_constr, symm_constr, metric_sens)
%LEARN_LPVDS Summary of this function goes here
%   Detailed explanation goes here

% Extract Position and Velocities
M           = size(Data,1)/2;    
Xi_ref      = Data(1:M,:);
Xi_dot_ref  = Data(M+1:end,:);   
[~, nb_data] = size(Xi_ref);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 2 (GMM FITTING): Fit GMM to Trajectory Data %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% GMM Estimation Algorithm %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% GMM Estimation Algorithm %%%%%%%%%%%%%%%%%%%%%%
% 0: Physically-Consistent Non-Parametric (Collapsed Gibbs Sampler)
% 1: GMM-EM Model Selection via BIC
% 2: CRP-GMM (Collapsed Gibbs Sampler)
est_options = [];
% est_options.type             = 0;   % GMM Estimation Algorithm Type
% PC-GMM IS 0 BUT LIGHT_SPEED SHOULD BE COMPILED
est_options.type             = gmm_type;   % GMM Estimation Algorithm Type 

% If algo 1 selected:
est_options.maxK             = 10;  % Maximum Gaussians for Type 1
est_options.fixed_K          = [];  % Fix K and estimate with EM for Type 1

% If algo 0 or 2 selected:
est_options.samplerIter      = 30;  % Maximum Sampler Iterations
                                    % For type 0: 20-50 iter is sufficient
                                    % For type 2: >100 iter are needed
                                    
est_options.do_plots         = 0;   % Plot Estimation Statistics
% Size of sub-sampling of trajectories for efficient GMM estimation
% 1/2 for 2D datasets, >2/3 for real
sub_sample = 1;
if nb_data > 1000
        sub_sample = ceil(nb_data/1000);
end
if nb_data > 2000
        sub_sample = ceil(nb_data/2000);
end
est_options.sub_sample       = sub_sample;       

% Metric Hyper-parameters
est_options.estimate_l       = 1;   % '0/1' Estimate the lengthscale, if set to 1
est_options.l_sensitivity    = metric_sens;   % lengthscale sensitivity [1-10->>100]
                                    % Default value is set to '2' as in the
                                    % paper, for very messy, close to
                                    % self-intersecting trajectories, we
                                    % recommend a higher value
est_options.length_scale     = [];  % if estimate_l=0 you can define your own
                                    % l, when setting l=0 only
                                    % directionality is taken into account

% Fit GMM to Trajectory Data
[Priors, Mu, Sigma] = fit_gmm(Xi_ref, Xi_dot_ref, est_options);
% if length(Priors) < 7 % Try again
%     [Priors, Mu, Sigma] = fit_gmm(Xi_ref, Xi_dot_ref, est_options);
% end


% Order Gaussian parameters based on closeness to attractor 
[idx] = knnsearch(Mu', att', 'k', size(Mu,2));
Priors = Priors(:,idx);
Mu     = Mu(:,idx);
Sigma  = Sigma(:,:,idx);

% Make the closet Gaussian isotropic and place it at the attractor location
Sigma(:,:,1) = min(2*max(diag(Sigma(:,:,1))),0.015)*eye(M);
Mu(:,1) = att;

if true
    %% Generate GMM data structure for DS learning
    clear ds_gmm; ds_gmm.Mu = Mu; ds_gmm.Sigma = Sigma; ds_gmm.Priors = Priors; 
    
    % (Recommended!) Step 2.1: Dilate the Covariance matrices that are too thin
    % This is recommended to get smoother streamlines/global dynamics
    adjusts_C  = 1;
    if adjusts_C  == 1 
        tot_dilation_factor = 1.5; rel_dilation_fact = 0.75;        
        Sigma_ = adjust_Covariances(ds_gmm.Priors, ds_gmm.Sigma, tot_dilation_factor, rel_dilation_fact);
        ds_gmm.Sigma = Sigma_;
    end   
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% %%%%%%%%  Step 3 (DS ESTIMATION): ESTIMATE SYSTEM DYNAMICS MATRICES  %%%%%%%%%
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%% DS OPTIMIZATION OPTIONS %%%%%%%%%%%%%%%%%%%%%% 
    % Type of constraints/optimization 
    % lyap_constr 
    % 0:'convex':     A' + A < 0 (Proposed in paper)
    % 1:'non-convex': A'P + PA < 0 (Sina's Thesis approach - not suitable for 3D)
    % 2:'non-convex': A'P + PA < -Q given P (Proposed in paper)                                 
    init_cvx    = 1;      % 0/1: initialize non-cvx problem with cvx                
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if lyap_constr == 0 || lyap_constr == 1
        P_opt = eye(M);
    else
        % P-matrix learning
    %     [Vxf] = learn_wsaqf(Data,0,att);
       
        % (Data shifted to the origin)
        % Assuming origin is the attractor (works better generally)
        [Vxf] = learn_wsaqf(Data_sh);
        P_opt = Vxf.P;
    end
    
    %%%%%%%%  LPV system sum_{k=1}^{K}\gamma_k(xi)(A_kxi + b_k) %%%%%%%%  
    if lyap_constr == 1
        [A_k, b_k, ~] = optimize_lpv_ds_from_data(Data_sh, zeros(M,1), lyap_constr, ds_gmm, P_opt, init_cvx);
        ds_lpv = @(x) lpv_ds(x-repmat(att,[1 size(x,2)]), ds_gmm, A_k, b_k);
    else
        [A_k, b_k, ~] = optimize_lpv_ds_from_data(Data, att, lyap_constr, ds_gmm, P_opt, init_cvx, symm_constr);
        ds_lpv = @(x) lpv_ds(x, ds_gmm, A_k, b_k);
    end

end

end

