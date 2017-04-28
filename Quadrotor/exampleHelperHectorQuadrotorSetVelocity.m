function exampleHelperQuadrotorSetVelocity(velPub, vLinX, vAngZ, vLinZ)
%exampleHelperTurtleBotSetVelocity Sets linear and angular velocity of Turtlebot

%   Copyright 2015 The MathWorks, Inc.

persistent velMsg

if isempty(velMsg)
    velMsg = rosmessage(velPub);
end

    velMsg.Linear.X = vLinX;
    velMsg.Angular.Z = vAngZ;
    velMsg.Linear.Z = vLinZ;
    send(velPub,velMsg);
end
