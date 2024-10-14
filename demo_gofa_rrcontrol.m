clc;clear;close all
[filepath,~,~] = fileparts(mfilename("fullpath"));
addpath(fullfile(filepath,'myfunc'))

%% Robot Model

% Flange to TCP
toolpos = [0.06, 0, 0.08];
tcp = [eye(3),toolpos';0,0,0,1];

% Base
base = eye(4);

% Model
[KinePara,DispPara,PreSetting] = model_GoFa(tcp,base);

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

myRob.ShowRobot([0,0,0,0,30,0]/180*pi);
xlim([-0.3,0.7]); 
ylim([-0.7,0.3]); 
zlim([-0.02,1])
view(130,15)

%% Initial and Goal Pose

ag0 = myRob.cjoint;
T_goal = [   -0.5238   -0.7070    0.4751    0.5952;
             -0.5897   -0.1015   -0.8012   -0.6006;
              0.6147   -0.6999   -0.3637    0.4781;
              0         0         0    1.0000];
T_goal(1:3,1:3) = ForceRot(T_goal(1:3,1:3),'m');

PlotFrame(myRob.cpose,'scale',0.08,'linewidth',2,'style','-',...
        'colorind',1,'text','off');hold on
PlotFrame(T_goal,'scale',0.08,'linewidth',2,'style','-',...
        'colorind',2,'text','off');hold on

%% Loop

axisnum = 6;
maxloop = 200;
k = 0.2; % Learning Rate
max_step = ones(axisnum,1)*1/180*pi.*[-1,1]; 
% Max Rotation Angle for Each Step

ag = ag0;
myRob.keeptrace = true;

for i = 1:maxloop

    % Jacobian
    J0 = myRob.get_jacob0;

    % matrix difference
    [dp,dw] = diffSE3(T_goal,myRob.cpose,'world');    
    e = [dp;dw];

    % Stop Iteration
    if norm(dp)<=1/1000/10 && norm(dw)*180/pi<=0.1
        break
    end

    dag = k * (pinv(J0) * e);
    dag = bound2range(dag,max_step,true);
    ag = ag + dag';
    myRob.MoveAbsJ(ag);
end
