function u = hectorQuadrotorComputePotentialField( target , pose, pose_obs, minDist)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
d = 0.2;
p0 = 5;
k_att = 1;
k_rep = -5; % -0.5;

F1 = getF_attractiv(k_att, pose, target);
F2 = getF_repulsive(k_rep, p0, pose, pose_obs, minDist);
F = F1 - F2;
u = d * F;
end

