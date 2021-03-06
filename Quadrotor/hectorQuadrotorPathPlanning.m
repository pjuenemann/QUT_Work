function obsData = hectorQuadrotorPathPlanning(handles, target, B, A)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
plotobj = ExampleHelperTurtleBotVisualizer([-10,10,-10,10]);
obsData = zeros(1,2);

exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
        0, 0, 1, 0);
pause(0.1);
exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
        0, 0, 0, 0);

pose = readPose(handles.odomSub.LatestMessage);
laserdata = readLaserData(handles.laserSub.LatestMessage);
plotData(plotobj,pose([1 2 6]),laserdata);

while sqrt(sum((pose([1:3]) - target).^2)) > 0.1 %&& reply~='q'        
    % Retrieve pose and laser data and send to plotting function
    % Wait for next laser reading
    laserMsg = receive(handles.laserSub,3);
    odomMsg = handles.odomSub.LatestMessage;
    laserdata = readLaserData(laserMsg);
    pose = readPose(odomMsg);
    pose = pose + filter(B,A,pose);
    
    sonar_height = receive(handles.sonar,3);
%     exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
%         0, 0, 0, 0);
    [dist, data] = findObstacle(laserMsg, pose, B, A);
%     if ~isempty(data)
%         z = repmat(pose(3), [size(data,1), 1]);
%         data = [data z]
%     end

%     if size(data,1) >= 3
%         d = dist;
%         d = sort(d);
%         e = d(1);
%         for i = 2:numel(d)
%             if d(i) > e(end) + 0.25
%                 e = [e d(i)];
%             end
%         end
%         idx = [];
%         for i = e
%             idx = [idx find(dist==i)];
%         end
%         if numel(idx) >= 3
%             [vx, vy] = voronoi(data(idx,1),data(idx,2));
%             figure(2);
%             %         hold on;
%             plot(vx,vy,'b-');
%         end
%     end
    
%     [x y] = meshgrid(-5:0.5:5,-5:0.5:5);
%     v = zeros(size(x));
%     w = zeros(size(x));
%     for i = 1:size(x,1)
%         for j = 1:size(y,1)
%             distN = zeros(size(data,1),1);
%             for k = 1:size(data,1)
%                 distN(k) = norm(data(k,:) - [x(i,j) y(i,j) data(k,3)]);
%             end
%             u = hectorQuadrotorComputePotentialField(target, [x(i,j), y(i,j), pose(3)], data, distN);
%             v(i,j) = u(1)/0.1;
%             w(i,j) = u(2)/0.1;
%         end
%     end
%     figure(2);
%     quiver(x,y,v,w)
%     tmpData = data;
%     for i = 1:size(data,1)
%         if tmpData(i,3) <= 0.5
%             tmpData(i,:) = [];
%         end
%     end
%     data = tmpData;

    u = hectorQuadrotorComputePotentialField(target,...
        pose([1:3]), data, dist);
    
    if (abs(u(1)) > 1)
        u(1) = u(1) / abs(u(1));
    end
    vLinX = 0.2*u(1);
    if (abs(u(2)) > 1)
        u(2) = u(2) / abs(u(2));
    end
    vLinY = 0.2*u(2);
    if (abs(u(3)) > 1)
        u(3) = u(3) / abs(u(3));
    end
    vLinZ = u(3);
    
    vAngZ = 0;
    psiDrone = atan2(target(2) - pose(2), target(1) - pose(1)) - pose(6);
    if abs(psiDrone) > 0.25 % && abs(psiDrone) < 0.25
        vAngZ = psiDrone;
    end
    
    if (sonar_height.Range_ < 0.5)
        vLinX = 0;
        vLinY = 0;
        vLinZ = 0.2;
    end
    
    % Publish velocities to the robot
    exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
        vLinX, vLinY, vLinZ, vAngZ);
    plotData(plotobj,pose([1 2 6]),laserdata);  
    pause(0.1);
end
exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
        0, 0, 0, 0);
end

function laserData = readLaserData(laserMsg)
%readPose Extract the laser data in Cartesian coordinates
laserData = readCartesian(laserMsg) * [0 1; 1 0];
end

function [R, dataWorldXYZ] = findObstacle(obj, pose, B, A)
defaults.RangeLimits = [0.5, obj.RangeMax];
R = obj.Ranges  + filter(B,A,obj.Ranges);
validIdx = isfinite(R) & R >= defaults.RangeLimits(1) & R <= 1.5; %defaults.RangeLimits(2);
R = R(validIdx);
angles = obj.readScanAngles();
cartAngles = angles(validIdx);
x = cos(cartAngles) .* R(:,1);
y = sin(cartAngles) .* R(:,1);
% cart = double([x,y]) * [0 1;1 0];
if ~isempty(x) && ~isempty(y)
    cart = double([x,y,repmat(pose(3),[numel(R(:,1)),1])]) * [0 0 1; 0 1 0; 0 0 1]; % cart = double([x,y]) * [0 1;1 0]; 
else
    cart = [];
end

% Without rotation in X and Y. Here Theta is rotation on Z-axis
% th = pose(6)-pi/2;
% dataWorldXY = cart*[cos(th) sin(th);-sin(th) cos(th)] ...
%                 + repmat(pose(1:2),[numel(cart(:,1)),1]);

% Rotation of objects in X, Y and Z
dataWorldXYZ = [];
if ~isempty(cart)
    phi = pose(4);
    theta = pose(5);
    psi = pose(6)-pi/2;
    dataWorldXYZ = cart*[1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)] ...
        *[cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)] ...
        *[cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1] ...
        + repmat(pose(1:3),[numel(cart(:,1)),1]);
end
end

function pose = readPose(odomMsg)
%readPose Extract the robot odometry reading as [x y z phi theta psi] vector

% Extract the x, y, z, phi, theta and psi coordinates
poseMsg = odomMsg.Pose.Pose;
xpos = poseMsg.Position.X;
ypos = poseMsg.Position.Y;
zpos = poseMsg.Position.Z;
quat = poseMsg.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]); % default order for Euler angle rotations is ZYX
pose = [xpos, ypos, zpos, angles(3), angles(2), angles(1)];
end

