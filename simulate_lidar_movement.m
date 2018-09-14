% Load Lidar data and add random displacement to x and y
% Data format in Lidar data example
% Angle\Distance\Amplitude
function [Lidar_Table_disp,Lidar_data]=simulate_lidar_movement(theta,dist)
    fname = ['Lidar_data_example'];
    Lidar_data = dlmread( fname, ' ', 3, 0)';
    %theta=randi(180)/180*pi;
    %dist=randi(100);
    x_dist=cos(theta)*dist;
    y_dist=sin(theta)*dist;
    %Lidar_data(1,:)=Lidar_data(1,:)+theta/pi*180;
    for ii=1:length(Lidar_data)
    Lidar_Table(1,ii)=cos(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii)/10;
    Lidar_Table(2,ii)=sin(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii)/10;
    end
    %R = orth(rand(2,2)) % random rotation matrix
    R=[cos(theta) -sin(theta);sin(theta) cos(theta)];
%     if det(R) < 0
%       V(:,2) = -1*V(:,2);
%       R = V*U';
%     end
    %t = rand(2,1) % random translation
    t=dist*rand(2,1);
    n = length(Lidar_data); % number of points
    Lidar_Table_disp = R*Lidar_Table + repmat(t, 1, n);
  