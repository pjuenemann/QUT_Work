function [ F_att ] = getF_attractiv( k_att, q, target )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
F_att = -k_att*(q - target);

end

