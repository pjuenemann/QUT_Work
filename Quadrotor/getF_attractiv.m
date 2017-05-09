function [ F_att ] = getF_attractiv( k_att, q, target )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if sqrt((sum(q - target))^2) < 2
    k_att = 10;
end
F_att = -k_att*(q - target);

end

