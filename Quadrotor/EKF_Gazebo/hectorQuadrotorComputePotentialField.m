function u = hectorQuadrotorComputePotentialField( target , pose, pose_obs, minDist)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
d = 0.1;
p0 = 5;
k_att = 1;
k_rep = -5; % -0.5;

F = getF_attractiv(k_att, pose, target);
if minDist <= p0
    F = double(F - k_rep*(1/minDist - 1/p0)*(1/minDist^2)*((pose - pose_obs)/minDist));
end
u = d * F;
end

