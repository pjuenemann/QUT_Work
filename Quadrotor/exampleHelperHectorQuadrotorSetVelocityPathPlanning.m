function exampleHelperQuadrotorSetVelocityPathPlanning(velPub, vLinX, vLinY, vLinZ)
%exampleHelperTurtleBotSetVelocity Sets linear and angular velocity of Turtlebot

%   Copyright 2015 The MathWorks, Inc.

persistent velMsg

if isempty(velMsg)
    velMsg = rosmessage(velPub);
end

    velMsg.Linear.X = vLinX;
    %velMsg.Linear.Y = vLinY;
    velMsg.Angular.Z = vLinY;
    velMsg.Linear.Z = vLinZ;
    send(velPub,velMsg);
end
