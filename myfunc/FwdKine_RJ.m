function T = FwdKine_RJ(ag,rotdirc,kinv,tcp,base,varargin)

if nargin<6
    flange = [0,0,-1;0,1,0;1,0,0]';
else
    flange = varargin{1};
end

rotdirc = reshape(rotdirc,size(ag));
ag = ag.*sign(rotdirc);
rotdirc = abs(rotdirc);

axisnum = length(ag);
T = NaN(4,4,axisnum+3);

% Frame 1~6
for i = 1:axisnum
    T(:,:,i+1) = R2T(RotXYZ(ag(i),'RotAxis',rotdirc(i)),kinv(:,i));
end

% Frame flange
T(:,:,i+2) = R2T(flange,kinv(:,end));

% Frame tcp
T(:,:,end) = tcp;

% Frame base
T(:,:,1) = base;

end