function J = jacobb(T_cum,Rot_Dir)

J = jacob0(T_cum,Rot_Dir);
J(1:3,:) = T_cum(1:3,1:3,end)' * J(1:3,:);
J(4:6,:) = T_cum(1:3,1:3,end)' * J(4:6,:);

end