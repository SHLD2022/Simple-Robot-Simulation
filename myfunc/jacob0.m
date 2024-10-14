function J = jacob0(T_cum,Rot_Dir,varargin)
% T_cum: 4*4*n Matrix
% {T1},{T1*T2},{T1*T2*T3},...,{T1*T2*...*Tn}
% Rot_Dir: Rotation Dirction of Local Frame. 

axisnum = size(T_cum,3)-1;
J = NaN(6,axisnum);
if nargin == 3
    P = varargin{1};
else
    P = T_cum(1:3,4,end);
end

for i=1:axisnum
    J(4:6,i) = T_cum(1:3,Rot_Dir(i),i);
    J(1:3,i) = cross(J(4:6,i),P-T_cum(1:3,4,i));
end

end