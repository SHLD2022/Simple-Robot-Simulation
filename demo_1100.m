clc;clear;close all

%% Robot Model

% Flange to TCP
toolpos = [0.03, 0, 0.04];
tcp = [eye(3),toolpos';0,0,0,1];

% Base
base = eye(4);

% Model
[KinePara,DispPara,PreSetting] = model_1100(tcp,base);

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
myRob.frame0Size = 0.05;
myRob.frametSize = 0.025;

myRob.RobName = PreSetting.Name;

myRob.ShowRobot;
myRob.Jog;
myRob.keeptrace = true;