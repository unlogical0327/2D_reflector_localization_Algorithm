% Load Lidar data 
% 
%
function [Lidar_Table,Lidar_data]=load_Lidar_data()
    fname = ['Lidar_data_example'];
    Lidar_data = dlmread( fname, ' ', 3, 0)';
    for ii=1:length(Lidar_data)
    Lidar_Table(1,ii)=cos(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii)/10;
    Lidar_Table(2,ii)=sin(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii)/10;
    end