function [ds_gmm, ds_seds] = learn_seds(Data_sh, att, seds_obj, K_gmm)
%LEARN_SEDS Summary of this function goes here
%   Detailed explanation goes here

% Extract Position and Velocities
M          = size(Data_sh,1)/2;    
Xi_ref     = Data_sh(1:M,:);
Xi_dot_ref = Data_sh(M+1:end,:);  


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 2 (GMM FITTING): Fit GMM to Trajectory Data %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 'seds-init': follows the initialization given in the SEDS code
% 0: Do Model Selection with BIC
if K_gmm == 0
    est_options = [];
    est_options.type        = 1;   % GMM Estimation Alorithm Type
    est_options.maxK        = 10;  % Maximum Gaussians for Type 1/2
    est_options.do_plots    = 0;   % Plot Estimation Statistics
    est_options.fixed_K     = [];   % Fix K and estimate with EM
    est_options.sub_sample  = 1;   % Size of sub-sampling of trajectories 
    
    [Priors0, Mu0, Sigma0] = fit_gmm([Xi_ref; Xi_dot_ref], [], est_options);
    nb_gaussians = length(Priors0);
else
    % Select manually the number of Gaussian components
    nb_gaussians = K_gmm;
end

% Finding an initial guess for GMM's parameter
[Priors0, Mu0, Sigma0] = initialize_SEDS([Xi_ref; Xi_dot_ref],nb_gaussians);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%  Step 3 (DS ESTIMATION): RUN SEDS SOLVER  %%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear options;
options.tol_mat_bias  = 10^-6;    % A very small positive scalar to avoid
                                  % instabilities in Gaussian kernel [default: 10^-1]                             
options.display       = 1;        % An option to control whether the algorithm
                                  % displays the output of each iterations [default: true]                            
options.tol_stopping  = 10^-6;    % A small positive scalar defining the stoppping
                                  % tolerance for the optimization solver [default: 10^-10]
options.max_iter      = 500;      % Maximum number of iteration forthe solver [default: i_max=1000]
options.objective     = seds_obj; % 'mse'/'likelihood'
sub_sample            = 1;

%running SEDS optimization solver
[Priors, Mu, Sigma]= SEDS_Solver(Priors0,Mu0,Sigma0,[Xi_ref(:,1:sub_sample:end); Xi_dot_ref(:,1:sub_sample:end)],options); 
ds_seds = @(x) GMR_SEDS(Priors,Mu,Sigma,x-repmat(att,[1 size(x,2)]),1:M,M+1:2*M);
clear ds_gmm; ds_gmm.Mu = Mu; ds_gmm.Sigma = Sigma; ds_gmm.Priors = Priors;
end

