% Report RMS errors of reflector array
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% find Rotation and transition of matrix A and B
% to find the location of Lidar itself, we calculate the centroid of many
% points 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ret_R, ret_T, reflector_rmse, Lidar_update_xy]=locate_reflector_xy(Reflector_Table,Reflector_ID,detected_reflector,detected_ID,Lidar_x,Lidar_y)
A1=[Reflector_Table(1,1:length(Reflector_ID));Reflector_Table(2,1:length(Reflector_ID))]';
B1=[detected_reflector(1,1:length(detected_ID));detected_reflector(2,1:length(detected_ID))]';
% R = orth(rand(3,3)); % random rotation matrix
% 
% if det(R) < 0
%     V(:,3)= -1*V(:,3);
%     R = V*U';
% end
% 
% t = rand(3,1); % random translation
% 
% n = 10; % number of points
% A = rand(n,3);
% B = R*A' + repmat(t, 1, n);
% B = B';
n_t=length(A1);
[ret_R,ret_T]=rigid_transform_3D(A1, B1);
A2 = (ret_R*A1') + repmat(ret_T, 1, n_t);
A2 = A2';

% Find the error
err = A2 - B1;
err = err .* err;
err = sum(err(:));
reflector_rmse = sqrt(err/n_t);

disp(sprintf("RMSE: %f", reflector_rmse));
disp("If RMSE is approaching zero, the matching is getting very close!");

Lidar_xy=[Lidar_x;Lidar_y]';
Lidar_update_xy=(ret_R*Lidar_xy'+ret_T)';