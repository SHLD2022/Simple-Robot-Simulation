clc;clear;close all

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

%%
n = 1000;
ag5 = linspace(-15,-5,n)/180*pi;
ags = zeros(n,6);
ags(:,5) = ag5;
mu = NaN(n,3);
v = NaN(6,n);
for i = 1:n
%     if i==520
%         1+1
%     end
    [mu(i,:),v(:,i)] = myRob.manipulability(ags(i,:));
end
[mincal,minind] = min(mu);
singag = ag5(minind)*180/pi;

figure
plot(ag5*180/pi,mu,'.-'); 
title(['singularity at A5 = ',num2str(mean(singag),'%.2f')])
grid on
xlabel('Joint 5 Angle [deg]','FontWeight','bold')
ylabel('\mu','FontWeight','bold')