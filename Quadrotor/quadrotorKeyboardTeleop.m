clear all;
close all;
clc;

ipaddress_vm = '192.168.255.128';
ipaddress_host = '192.168.56.1';
rosinit(ipaddress_vm, 'NodeHost', ipaddress_host)

% handles.odomSub = rossubscriber('/odom', 'BufferSize', 25);
% receive(handles.odomSub,3);
handles.laserSub = rossubscriber('/scan', 'BufferSize', 5);
receive(handles.laserSub,3);

handles.odomSub = rossubscriber('/ground_truth/state', 'BufferSize', 25);
receive(handles.odomSub,3);
handles.sonar = rossubscriber('/sonar_height', 'BufferSize', 5);
receive(handles.sonar,3);

handles.velPub = rospublisher('/cmd_vel');
% exampleHelperHectorQuadrotorKeyboardControl(handles);
target = [6, 0, 0.7];
obsData = hectorQuadrotorPathPlanning(handles, target);


rosshutdown