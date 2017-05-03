function obsData = hectorQuadrotorPathPlanning(handles, target)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
plotobj = ExampleHelperTurtleBotVisualizer([-10,10,-10,10]);
obsData = zeros(1,2);
originTarget = target;
lastPositions = zeros(10,3);
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
    
    sonar_height = receive(handles.sonar,3)

    a_gain = 0.4;
    gain = 0.4;
    [minDist, data] = findObstacle(laserMsg, pose);
    data = [data(1,:) pose(3)];
    % Publish velocities to the robot
    lastPositions = circshift(lastPositions,1);
    lastPositions(1,:) = pose([1:3]);
    v = sqrt(sum((pose([1:3]) - target).^2));
    if v > 1 && sqrt(sum(std(lastPositions).^2)) < 0.075  % abs(sum(F)) < 1
        target(3) = target(3) + 0.5; % * v;
    else
        target = originTarget;
    end
    u = hectorQuadrotorComputePotentialField(target,...
        pose([1:3]), data, minDist);

    a = 0;
%     if abs(a) >= 1 
%         a_gain = 1/a;
%     end

    theta = atan2(u(2), u(1)) - pose(4);
    if abs(theta) > -0.1 && abs(theta) < 0.1
        a = norm(u(1), u(2)); % u(1);
    end
    forwardX = a_gain * a;
    forwardY = gain * theta; % u(2);
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

