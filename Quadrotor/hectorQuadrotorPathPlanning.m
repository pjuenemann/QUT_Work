function obsData = hectorQuadrotorPathPlanning(handles, target)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
plotobj = ExampleHelperTurtleBotVisualizer([-10,10,-10,10]);
obsData = zeros(1,2);
% keyObj = ExampleHelperTurtleBotKeyInput();

% disp('Keyboard Control: ');
% disp('q=quit');
% disp('Waiting for input: ');

exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
        0, 0, 1);
pause(0.1);
exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
        0, 0, 0);
% Use ASCII key representations for reply, initialize here
reply = 0;

pose = readPose(handles.odomSub.LatestMessage);
laserdata = readLaserData(handles.laserSub.LatestMessage);

try
    plotData(plotobj,pose([1 2 4]),laserdata);
catch
    reply = 'q'; % If figure is closed, exit
end

while sqrt(sum((pose([1:3]) - target).^2)) > 0.1 %&& reply~='q'    
    %reply = getKeystroke(keyObj);
    
    % Retrieve pose and laser data and send to plotting function
    % Wait for next laser reading
    laserMsg = receive(handles.laserSub,3);
    odomMsg = handles.odomSub.LatestMessage;
    laserdata = readLaserData(laserMsg);
    pose = readPose(odomMsg);
    
    
%     th = pose(3)-pi/2;
% 
%     dist = [];
%     dataWorld = [];
%     for i = 1:size(laserdata,1)
%         % Compute the world-frame location of laser points
%         data = laserdata(i,:);
%         dataWorld = data*[cos(th) sin(th);-sin(th) cos(th)] ...
%             + repmat(pose(1:2),[numel(data(:,1)),1]);
%         dist = [dist; norm(abs(dataWorld - pose([1 2])))];
%     end
%     minDist = min(dist)
%     data = dataWorld((find(dist==minDist)),:)
%     obs=zeros(1,2);
%     obs(1) = pose(1) + cos(pose(4))*data(1);
%     obs(2) = pose(2) + sin(pose(4))*data(2);
%     obsData = [obsData; obs];


    [minDist, data] = findObstacle(laserMsg, pose);
    data = [data(1,:) pose(3)];
    % Publish velocities to the robot
    u = 0.4 * hectorQuadrotorComputePotentialField(target,...
        pose([1:3]), data, minDist);
    forwardX = u(1);
    forwardY = u(2);
    if (pose(3) <= 0.5)
        forwardZ = 0.2;
    else
        forwardZ = u(3);
    end
    exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
        forwardX, forwardY, forwardZ);
    
    try
        plotData(plotobj,pose([1 2 4]),laserdata);
    catch
        reply = 'q'; % If figure is closed, exit
    end
    
    pause(0.1);
end
exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
        0, 0, 0);
% closeFigure(keyObj);
end

function laserData = readLaserData(laserMsg)
%readPose Extract the laser data in Cartesian coordinates
laserData = readCartesian(laserMsg) * [0 1; 1 0];
end

function [minDist, dataWorld] = findObstacle(obj, pose)
defaults.RangeLimits = [0.5, obj.RangeMax];
R = obj.Ranges;
validIdx = isfinite(R) & R >= defaults.RangeLimits(1) & R <= defaults.RangeLimits(2);
R = R(validIdx);
minDist = min(R);
validIdx = find(R==minDist);
angles = obj.readScanAngles();
cartAngles = angles(validIdx);
x = cos(cartAngles) .* R(validIdx,1);
y = sin(cartAngles) .* R(validIdx,1);
cart = double([x,y]) * [0 1; 1 0];

th = pose(4)-pi/2;
dataWorld = cart*[cos(th) sin(th);-sin(th) cos(th)] ...
                + repmat(pose(1:2),[numel(cart(:,1)),1]);
end

function pose3 = readPose(odomMsg)
%readPose Extract the robot odometry reading as [x y theta] vector

% Extract the x, y, and theta coordinates
poseMsg = odomMsg.Pose.Pose;
xpos = poseMsg.Position.X;
ypos = poseMsg.Position.Y;
zpos = poseMsg.Position.Z;
quat = poseMsg.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
theta = angles(1);
pose = [xpos, ypos, theta];
pose3 = [xpos, ypos, zpos, theta]

end

