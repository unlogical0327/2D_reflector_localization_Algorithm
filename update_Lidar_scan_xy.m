% Convert the calculated rotation and transition of Lidar to system and
% update Lidar x-y map
% report RMSE of whole Lidar map (could be large due to new coming data)
function [Lidar_update_Table,Lidar_update_xy]=update_Lidar_scan_xy(ret_R,ret_T,Lidar_Table1,Lidar_Table,Lidar_x,Lidar_y)
% ret_R: rotation matrix
% ret_T: transition matrix
% Lidar_Table1: new Lidar scan data
% LIdar_Table: reference lidar data
A1=Lidar_Table1;
B1=Lidar_Table;
n_t=length(A1);
A2 = ret_R^-1*(A1 - repmat(ret_T, 1, n_t));   % !!!! why should I use -
%A2 = A2';
Lidar_update_Table=A2;
% Find the error
err = A2 - B1;
err = err .* err;
err = sum(err(:));
rmse = sqrt(err/n_t);

disp(sprintf("RMSE of whole map: %f", rmse));
%disp("If RMSE is approaching zero, the matching is getting very close!");

Lidar_xy=[Lidar_x;Lidar_y]';
Lidar_update_xy=ret_R^-1*(Lidar_xy-ret_T)';