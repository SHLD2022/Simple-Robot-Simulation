function [KinePara,DispPara,PreSetting] = model_GoFa(tcp,base)
%% Parameters Kinematics Model

% Vector to Next Frame, ref to Frame0
KinePara.kinv = [0	0	0.1679;
                 0	-0.0609	0.0971;
                 0	0	0.444;
                 0.113	0.0609	0.11;
                 0.357	0.0565	0;
                 0.101	-0.0565	0.08
                 0   0   0]';

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
DispPara.jsize = [[1,1,0.9]*0.06, [1,0.8,0.8]*0.055];

% Number of Points on the Circle
DispPara.cirpts = 9;

% How to Draw the Links
% 2,3,1 means draw y dirction first, then z then x
DispPara.offorder = [2,3,1;
                     2,3,1;
                     1,2,3;
                     2,3,1;
                     1,2,3;
                     2,3,1;
                     2,3,1];

%%
% Joint Limit 
PreSetting.jrange = [-180,180;
                     -180,180;
                     -225,85;
                     -180,180;
                     -180,180;
                     -270,270]'/180*pi;

% 
PreSetting.Home = [0,0,0,0,30,0]/180*pi;

PreSetting.Name = 'CRB15000';
end