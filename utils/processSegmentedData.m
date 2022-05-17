function [Data, Data_sh, att, att_all, x0_all] = processSegmentedData(data)

% Computing the attractor and shifting all the trajectories
N = length(data);att_all = [];
M = size(data{1},1)/2;
for n=1:N
    att_all = [att_all data{n}(1:M,end)];
end
att = mean(att_all,2);
shifts = att_all - repmat(att, [1 size(att_all,2)]);
Data = []; Data_sh = []; x0_all = [];
for l=1:N
    % Gather Data
    data_ = data{l};
    shifts_ = repmat(shifts(:,l),[1 size(data_,2)]);
    data_(1:M,:)       = data_(1:M,:) - shifts_;
    data_(M+1:end,end) = zeros(M,1);
    data_(M+1:end,end-1) = (data_(M+1:end,end) + zeros(M,1))/2;
    data_(M+1:end,end-2) = (data_(M+1:end,end-2) + data_(M+1:end,end-1))/2;
    Data = [Data data_];
    
    % All starting position for reproduction accuracy comparison
    x0_all = [x0_all data_(1:M,1)];
    
    % Shift data to origin for Sina's approach + SEDS
    data_(1:M,:)       = data_(1:M,:) - repmat(att, [1 size(data_,2)]);
    data_(M+1:end,end) = zeros(M,1);
    
    Data_sh = [Data_sh data_];
    
    % Generate new data structure for SEDS + Diff-DS
    data{l} = data_;
end
data_12 = data{1}(:,1:2);
% dt = abs((data_12(1,1) - data_12(1,2))/data_12(3,1));
end