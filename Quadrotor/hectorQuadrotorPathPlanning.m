function obsData = hectorQuadrotorPathPlanning(handles, target, B, A)
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
    pose = pose + filter(B,A,pose);
    
    sonar_height = receive(handles.sonar,3);

    a_gain = 0.2;
    gain = 0.2;
    [dist, data] = findObstacle(laserMsg, pose, B, A);
    if ~isempty(data)
        z = repmat(pose(3), [size(data,1), 1]);
        data = [data z];
    end
    % Publish velocities to the robot
%     lastPositions = circshift(lastPositions,1);
%     lastPositions(1,:) = pose([1:3]);
%     v = sqrt(sum((pose([1:3]) - target).^2));
%     if v > 1 && sqrt(sum(std(lastPositions).^2)) < 0.075  % abs(sum(F)) < 1
%         target(3) = target(3) + 0.5; % * v;
%     else
%         target = originTarget;
%     end
    u = hectorQuadrotorComputePotentialField(target,...
        pose([1:3]), data, dist);

    a = 0;
    theta = atan2(u(2), u(1)) - pose(4);
    if abs(theta) > -0.1 && abs(theta) < 0.1
        a = norm(u(1), u(2)); % u(1);
    end
    
    if (abs(u(1)) > 1)
        u(1) = u(1) / abs(u(1));
    end
    forwardX = 0.4*u(1) % a_gain * a;
    if (abs(u(2)) > 1)
        u(2) = u(2) / abs(u(2));
    end
    forwardY = 0.4*u(2) % gain * theta; % u(2);
    if (abs(u(3)) > 1)
        u(3) = u(3) / abs(u(3));
    end
    forwardZ = u(3)
    
    if (sonar_height.Range_ < 0.5)
            forwardX = 0;
            forwardY = 0;
            forwardZ = 0.2;
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

function [R, dataWorld] = findObstacle(obj, pose, B, A)
defaults.RangeLimits = [0.5, obj.RangeMax];
R = obj.Ranges  + filter(B,A,obj.Ranges);
validIdx = isfinite(R) & R >= defaults.RangeLimits(1) & R <= 1; %defaults.RangeLimits(2);
% R = R(validIdx);
% minDist = min(R);
% validIdx = find(R<1); % validIdx = find(R==minDist);
R = R(validIdx);
angles = obj.readScanAngles();
cartAngles = angles(validIdx);
x = cos(cartAngles) .* R(:,1); % R(validIdx,1);
y = sin(cartAngles) .* R(:,1); % R(validIdx,1);
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
pose3 = [xpos, ypos, zpos, theta];

end

