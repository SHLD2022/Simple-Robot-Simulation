function [KinePara,DispPara,PreSetting] = model_1100(tcp,base)
%% Parameters Kinematics Model

% Vector to Next Frame, ref to Frame0
KinePara.kinv = [0  0  0.1346;
0  -0.0551  0.0554;
0  0.0551  0.185;
0.0479  0  0;
0.1371  0  0;
0.045  0  0;
0  0  0]';



% Rotation Axis ref to Frame0
KinePara.rotdirc = [3,2,2,1,2,1];

% Last Joint Frame to Tool Flange
KinePara.flange = [0,0,-1;0,1,0;1,0,0]';

% Base Frame
% KinePara.base = R2T(RotXYZ(pi/2,'RotAxis',2),[0;0;1]);
KinePara.base = base;

% Flange to TCP
KinePara.tcp = tcp;

%% Parameters Draw the Robot

% Joint Size (radius of the circle)
DispPara.jsize = [[1,1,0.9]*0.03, [1,0.75,0.65]*0.025];

% Number of Points on the Circle
DispPara.cirpts = 9;

% How to Draw the Links
% 2,3,1 means draw y dirction first, then z then x
DispPara.offorder = [2,3,1;
                     1,3,2;
                     1,2,3;
                     2,3,1;
                     1,2,3;
                     2,3,1;
                     2,3,1];

%%
% Joint Limit 
PreSetting.jrange = [-230,230;
                     -115,113;
                     -205,55;
                     -230,230;
                     -125,120;
                     -400,400]'/180*pi;

% 
PreSetting.Home = [0,0,0,0,30,0]/180*pi;

PreSetting.Name = 'CRB1300';
end