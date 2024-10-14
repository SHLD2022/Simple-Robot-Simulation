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

%myRob.showellip = true;
myRob.ShowRobot(myRob.home);

xlim([-0.4,0.6]); 
ylim([-0.4,0.4]); 
zlim([-0.01,0.7])


% Disable tcp trace
myRob.tcptrace = false;

myRob.GoHome;
T_final = myRob.cpose;
pause(0.4)

myRob.tcptrace = true;
myRob.keeptrace = true;

%% Offs

% Move -0.3 in z dirction, ref to world frame (default)
myRob.Offs([0,0,-0.3])

% Move -0.3 in z dirction, ref to tool frame
myRob.Offs([0,0,-0.3],'tool','showframe',true)

% Show initial and goal frame
myRob.Offs([0,-0.3,0],'showframe',true)

%% Rots

% rotate about z axis, ref to tool frame (default)
myRob.Rots(pi/2,'z')

% rotate about y axis, ref to world frame
myRob.Rots(pi/2,'y','world','showframe',true)

%% RRMove

% linear and rotation
myRob.RRMove(T_final,'showframe',true)

%% Jog

myRob.tcptrace = false;
myRob.keeptrace = false;
myRob.Jog;
