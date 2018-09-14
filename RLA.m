%Reflector localization algorithm
% This program is developed and copyright owned by Soleilware LLC
% The code is writen to verify the correctness of the localization
% algorithm process and efficiency.
% --------------------------------
% Created by Qi Song on 9/3/2018
% The process if designed as followed 
%% 1. Create/load the reflector position true table from reading CVS file/generating random list.
%% 2. Read distance data from Lidar, create the display frame in memory. 
%% 3. Screen the data point based on the reflection and the adjacent point
% continuity. Identify the reflector from background and check if the reflector is identical.
%% 4. Find the current position of Robot.
%% 5. Initalize the reflector table and find at least 2/3 reflectors: pick up at least 2/3 reflectors from the list(nearest distance or most distingushed).
% calculate the relative distance and match with the list.
%% 6. Calculate the expectation reflector list dynamically: from the distance and angle measurement from last moment. predict the current
% distance and angle, and save to the expectation list.
%% 7. Match the expectation list with the reflector list dynamically.
% Calculate the error between the expectation list and true list and match with the predefined threshold.
%% 8. Calculate the pose with the matched reflector pairs.
%----------------debugging question list-----------------------------------
%% Q: Do we really need to predict the expectation locations of reflector?
% This may impact the accurary of localization and depends on the update
% rate of Lidar distance data. Need to check with experiment
%% A: To be update.
%% Q: To predict the distance change and angle, what algorithm we need to apply? SLAM or any other data asistance like IMU, Odometer?
%% A: To be update
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define the variables and tables here
clear all
Table_size=120; %the szie of reflector number 
Reflector_Table=zeros(3,Table_size);   % Define the table as distance/angle/intensity
Expectation_Table=zeros(3,Table_size);   % Expectation table to predict the current location
map_size_x = 1000;   % map x dimension in meter
map_size_y = 1000;   % map y dimension in meter
colunm_x = 12;
row_y = Table_size/colunm_x;
Lidar_x=10;    % x coordinate of Lidar
Lidar_y=0;    % y coordinate of Lidar
%% Define simulation configuration here
list_source_flag=1;  % set the list flag to 0--read from file, 1--manually set the reflector location 2--generate 120x110 reflector array 2--generate from random location
Prediction_flag=0;   % enable feature to calculate the location prediction after step 5
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation starts from here !!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Create/load the reflector position true table from reading CVS file/generating random list.
if list_source_flag == 0 % read from file
    fname = ['Reflector_Table_example'];
    raw_data = dlmread( fname, ' ', 3, 0)';
    for ii=1:length(raw_data)
        Reflector_ID(ii) = ii;
    Reflector_Table(1,Reflector_ID(ii))=cos(raw_data(1,ii)/180*pi)*raw_data(2,ii);   % generate reflector array x 
    Reflector_Table(2,Reflector_ID(ii))=sin(raw_data(1,ii)/180*pi)*raw_data(2,ii);   % generate reflector array y
    end
elseif list_source_flag == 1 % manually set reflector location
    % input at least 3 points to calculate the robot location 
    % 45.0000 565.7  100
    % 45.0000 707.1  100
    % 45.0000 848.5  100
    Reflector_ID(1)=1;   
    Reflector_Table(1,Reflector_ID(1))=40;    % x 
    Reflector_Table(2,Reflector_ID(1))=40;    % y
    Reflector_ID(2)=2;
    Reflector_Table(1,Reflector_ID(2))=50;    % x
    Reflector_Table(2,Reflector_ID(2))=0;     % y
    Reflector_ID(3)=3;
    Reflector_Table(1,Reflector_ID(3))=0;
    Reflector_Table(2,Reflector_ID(3))=60;
    Reflector_ID(4)=4;
    Reflector_Table(1,Reflector_ID(4))=-50;
    Reflector_Table(2,Reflector_ID(4))=50;
    Reflector_ID(5)=5;
    Reflector_Table(1,Reflector_ID(5))=-60;
    Reflector_Table(2,Reflector_ID(5))=-60;
elseif list_source_flag == 2 % generate the 120 reflector array
    x_increm=map_size_x/(colunm_x-1);
    y_increm=map_size_y/(row_y-1);
    ii=1;
    for i_x=1:colunm_x
        for i_y=1:row_y
           x_pos = x_increm*(i_x-1);   % x location
           y_pos = y_increm*(i_y-1);   % y location
          Reflector_ID(ii)=(i_x-1)*row_y+i_y;
          Reflector_Table(1,Reflector_ID(ii))= x_pos;
          Reflector_Table(2,Reflector_ID(ii))= y_pos;
          ii=ii+1;
        end
    end
elseif list_source_flag == 3 % generate the reflector table randomly
    Reflector_Table = rand(120,100)*1000;
end
%%%--------- Plot the reflector map
figure(101)
plot(Reflector_Table(1,:),Reflector_Table(2,:),'or');
title('Map with reflector array');
xlabel('X dimension (M)');
ylabel('Y dimension (M)');
%xlim([0 100])
%ylim([0 100])
a = Reflector_ID'; b = num2str(a); c = cellstr(b);
dx = 0.5; dy = 0.5;
hold on;
text(Reflector_Table(1,Reflector_ID)+dx,Reflector_Table(2,Reflector_ID)+dy, c);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Read distance data from Lidar, create the display frame in memory. 
    fname = ['Lidar_data_example'];
    Lidar_data = dlmread( fname, ' ', 3, 0)';
    for ii=1:length(Lidar_data)
    Lidar_Table(1,ii)=cos(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii)/10;
    Lidar_Table(2,ii)=sin(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii)/10;
    end
%%%------------ plot Lidar data
figure(102)
plot(Lidar_Table(1,:),Lidar_Table(2,:),'+b');
title('Lidar reflection');
xlabel('X dimension (M)');
ylabel('Y dimension (M)');
%xlim([-1000 1000])
%ylim([-1000 1000])
%% 3. Screen the data point based on the reflection and the adjacent point
% continuity. Identify the reflector from background and check if the reflector is identical.
%%% identify the reflectors from Lidar data, return detected reflector array
amp_thres=80;
angle_delta=0.5;
distance_delta=10;
[detected_ID,detected_reflector]=identify_reflector(amp_thres,angle_delta,distance_delta,Lidar_data,Lidar_Table);
%%%%%%%%%%%%
% hold on;plot(detected_reflector(1,:),detected_reflector(2,:),'or')    
% a = detected_ID'; b = num2str(a); c = cellstr(b);
% dx = 5; dy = 5;
% hold on;
% text(detected_reflector(1,:)+dx,detected_reflector(2,:)+dy, c);
color='b'
plot_reflector(detected_reflector,detected_ID,color)
%% 4. match lidar data with reflector table and find the current position of Robot.
thres_dist_match=1;
[matched_reflector_ID]=calc_match_distance(Reflector_Table,Reflector_ID,detected_reflector,detected_ID,thres_dist_match);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% find Rotation and transition of matrix A and B and Lidar location
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[ret_R, ret_T, reflector_rmse, Lidar_update_xy]=locate_reflector_xy(Reflector_Table,Reflector_ID,detected_reflector,detected_ID,Lidar_x,Lidar_y);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot x and y coordinate 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on;plot(Lidar_update_xy(1,1),Lidar_update_xy(1,2),'ok')
Lidar_trace=Lidar_update_xy;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% above could be the calibration process at the initialization of
% software localization
%
% Below could be taken as the iterative process to obtain new Lidar scan
% and locate Lidar itself, convert to world coordinate and plot the Lidar
% moving trace.

%% 5. Initialize the reflector table and find at least 2/3 reflectors: pick up at least 2/3 reflectors from the list(nearest distance or most distingushed).
% --Select at least 3 reflectors to locate the robot
% --Manually choose ix reflectors(ix nearest points)
num_loc_ref=4;   % --define how many reflectors to be used.
nearest_ID_en=0;   % --enable nearest ID selection?0-not enabled; 1-enabled

% Option1: Load updated Lidar data
%[Lidar_Table1,Lidar_data1]=load_Lidar_data();
% Option2: Generate random displacement and load to system
%% ---- Loop starts from here ----
%  set the loop number of simulation and generate random movement at each
%  moment
Loop_num=10;
for ll=1:Loop_num     % simulation loop start from here!!!
theta=randi(360)/180*pi
dist=randi(500)
[Lidar_Table1,Lidar_data1]=simulate_lidar_movement(theta,dist);    %simulate random displacement
% identify the reflectors
[detected_ID1,detected_reflector1]=identify_reflector(amp_thres,angle_delta,distance_delta,Lidar_data1,Lidar_Table1);
for i=1:num_loc_ref
    enabled_ID(i)=i;
    enabled_reflector(:,enabled_ID)=detected_reflector1(:,enabled_ID);  % read enabled ID from Lidar data
end
%% Match the enabled reflectors in the reference reflectors
[matched_en_reflector_ID1]=calc_match_distance(Reflector_Table,Reflector_ID,enabled_reflector,enabled_ID,thres_dist_match);
%[matched_en_reflector_ID2]=calc_match_distance(Reflector_Table,Reflector_ID,detected_reflector1,detected_ID1,thres_dist_match);

%% plot map new Lidar scan
figure(102)
hold on;plot(Lidar_Table1(1,:),Lidar_Table1(2,:),'+g');
color='g';
%% plot map with random displacement
%plot_reflector(detected_reflector1,detected_ID1,color)
%% calculate rotation and transition
[ret_R1, ret_T1, reflector_rmse, Lidar_update_xy]=locate_reflector_xy(Reflector_Table,Reflector_ID,detected_reflector1,detected_ID1,Lidar_x,Lidar_y);
%% calculate updated map in the world map 
[Lidar_update_Table,Lidar_update_xy]=update_Lidar_scan_xy(ret_R1,ret_T1,Lidar_Table1,Lidar_Table,Lidar_x,Lidar_y);
%% Plot reference map in the world coordinate
figure(103)
plot(Lidar_Table(1,:),Lidar_Table(2,:),'+b');
hold on;
color='b';
%% Plot the reflectors in the world map
plot_reflector(detected_reflector,detected_ID,color) % plot reference reflector 
hold on;
%% Plot update map in the world map
plot(Lidar_update_Table(1,:),Lidar_update_Table(2,:),'+g');
% identify the reflector in the updated map
[detected_ID2,detected_reflector2]=identify_reflector(amp_thres,angle_delta,distance_delta,Lidar_data,Lidar_update_Table)
hold on;
color='g';
plot_reflector(detected_reflector2,detected_ID2,color)
hold on;
%% Save new Lidar location to the trace and plot
Lidar_trace=[Lidar_trace; Lidar_update_xy]
plot(Lidar_trace(:,1),Lidar_trace(:,2),'o-k')
disp(sprintf("RMSE: %f for %i th step", reflector_rmse,ll));
disp("RMSE errors for each reflector matching calculation ");
pause(2)
end  % Simulation loop end up here!!!