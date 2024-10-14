clc;clear;close all

%% Robot Model

% Flange to TCP
toolpos = [0.05, 0, 0.07];
tcp = [eye(3),toolpos';0,0,0,1];

% Base
base = eye(4);

% Model
[KinePara,DispPara,PreSetting] = model_UR5(tcp,base);

%% Setup Simulation

% Construct the Robot
myRob = SimpleRobotRJ(KinePara,DispPara);

% Joint Limit (Optional)
% If not specify, will be +-pi
myRob.jrange = PreSetting.jrange;

% Set Home Position (Optional)
myRob.home = PreSetting.Home;

% Base Frame and Tool Frame Size (Optional)
% If not specify, will not show frames
myRob.frame0Size = 0.12;
myRob.frametSize = 0.05;

myRob.RobName = PreSetting.Name;

%% Show Robot with Initial Pose
% myRob.showRobot(ag);
% ag is optional input, if not specify, will be all 0

myRob.showellip = true;
myRob.elipType = 'force';
myRob.ShowRobot(myRob.home);

xlim([-1.1,1.3]); 
ylim([-1,1]); 
zlim([-0.1,1.3])

%zlim([-0.2,inf])
%% Joint Motion

% Disable tcp trace
myRob.tcptrace = false;

% myRob.GoZero;
% pause(0.4)
myRob.GoHome;

load('jdata.mat');
ags = ags(1:5:end,[1,2,3,5,4,6]);
ags(:,5) = -ags(:,5);
ags = ags+myRob.home;

% MoveAbsJ, Joint Motion
myRob.MoveAbsJ(ags(1,:));
pause(0.4);

% Enable tcp trace
myRob.tcptrace = true;
myRob.MoveAbsJ(ags);

%% Jog

myRob.Jog;
