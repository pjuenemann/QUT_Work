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
plotData(plotobj,pose([1 2 4]),laserdata);

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
    if ~isempty(data)
        z = repmat(pose(3), [size(data,1), 1]);
        data = [data z]
    end

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

    u = hectorQuadrotorComputePotentialField(target,...
        pose([1:3]), data, dist);
    
    if (abs(u(1)) > 1)
        u(1) = u(1) / abs(u(1));
    end
    vLinX = 0.4*u(1); % a_gain * a;
    if (abs(u(2)) > 1)
        u(2) = u(2) / abs(u(2));
    end
    vLinY = 0.4*u(2); % gain * theta; % u(2);
    if (abs(u(3)) > 1)
        u(3) = u(3) / abs(u(3));
    end
    vLinZ = u(3);
    
    vAngZ = 0;
    theta = atan2(target(2) - pose(2), target(1) - pose(1)) - pose(4);
    if abs(theta) > -0.25 && abs(theta) < 0.25
        vAngZ = theta;
    end
    
    if (sonar_height.Range_ < 0.5)
        vLinX = 0;
        vLinY = 0;
        vLinZ = 0.2;
    end
    
    % Publish velocities to the robot
    exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
        vLinX, vLinY, vLinZ, vAngZ);
    plotData(plotobj,pose([1 2 4]),laserdata);  
    pause(0.1);
end
exampleHelperHectorQuadrotorSetVelocityPathPlanning(handles.velPub,...
        0, 0, 0, 0);
end

function laserData = readLaserData(laserMsg)
%readPose Extract the laser data in Cartesian coordinates
laserData = readCartesian(laserMsg) * [0 1; 1 0];
end

function [R, dataWorld] = findObstacle(obj, pose, B, A)
defaults.RangeLimits = [0.5, obj.RangeMax];
R = obj.Ranges  + filter(B,A,obj.Ranges);
validIdx = isfinite(R) & R >= defaults.RangeLimits(1) & R <= 2; %defaults.RangeLimits(2);
R = R(validIdx);
angles = obj.readScanAngles();
cartAngles = angles(validIdx);
x = cos(cartAngles) .* R(:,1);
y = sin(cartAngles) .* R(:,1);
cart = double([x,y]) * [0 1; 1 0];

th = pose(4)-pi/2;
dataWorld = cart*[cos(th) sin(th);-sin(th) cos(th)] ...
                + repmat(pose(1:2),[numel(cart(:,1)),1]);
end

function pose = readPose(odomMsg)
%readPose Extract the robot odometry reading as [x y z theta] vector

% Extract the x, y, z and theta coordinates
poseMsg = odomMsg.Pose.Pose;
xpos = poseMsg.Position.X;
ypos = poseMsg.Position.Y;
zpos = poseMsg.Position.Z;
quat = poseMsg.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
theta = angles(1);
pose = [xpos, ypos, zpos, theta];

end

