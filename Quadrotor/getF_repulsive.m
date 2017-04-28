function [ F_rep ] = getF_repulsive( k_rep, p0, q, q_obs, minDist)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% a = inf; % minimale Distanz
% for i = 1:size(obX1, 1)
%     for j = 1:size(obX1, 2)
%         obstacle = [obX1(i,j) obY1(i,j) h(1,i)];
%         b = norm(obstacle - q);
%         if b < a
%             a = b;
%             q_obs = obstacle; % Pose mit minimaler Distanz
%         end
%     end
% end
if minDist <= p0
    F_rep = k_rep*(1/minDist - 1/p0)*(1/minDist^2)*((q - q_obs)/minDist);
else
    F_rep = [0 0 0];
end

end

