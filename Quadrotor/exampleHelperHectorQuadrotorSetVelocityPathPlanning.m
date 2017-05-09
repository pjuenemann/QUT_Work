function exampleHelperQuadrotorSetVelocityPathPlanning(velPub, vLinX, vLinY, vLinZ, vAngZ)
%exampleHelperTurtleBotSetVelocity Sets linear and angular velocity of Turtlebot

%   Copyright 2015 The MathWorks, Inc.

persistent velMsg

if isempty(velMsg)
    velMsg = rosmessage(velPub);
end

    velMsg.Linear.X = vLinX;
    velMsg.Linear.Y = vLinY;
    velMsg.Linear.Z = vLinZ;
    velMsg.Angular.Z = vAngZ;
    send(velPub,velMsg);
end
