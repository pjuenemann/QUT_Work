function u = hectorQuadrotorComputePotentialField( target , pose, pose_obs, dist)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
d = 0.1;
p0 = 1;
k_att = 1;
k_rep = -5; % -0.5;

F1 = getF_attractiv(k_att, pose, target);
F2 = [0 0];
for i = 1:numel(dist)
    F2 = F2 + getF_repulsive(k_rep, p0, pose([1 2]), pose_obs(i,[1 2]), dist(i));
end
F = F1 - [F2 0];
u = d * F;
end

