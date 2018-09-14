
function [matched_reflect_ID] = calc_match_distance(Reflect_Table,Reflect_ID,detect_reflector,detect_ID,thres_dist_match)
% Define matching threshold value here
threshold_distance_match=thres_dist_match;
%-- calculate distance matrix between two reflectors
Reflect_vector=[Reflect_Table(1,1:length(Reflect_ID));Reflect_Table(2,1:length(Reflect_ID))]';
Ref_dist_vector=pdist(Reflect_vector);   % calculate the reference distance vector 
detect_vector=[detect_reflector(1,1:length(detect_ID));detect_reflector(2,1:length(detect_ID))]';
detect_dist_vector=pdist(detect_vector);   % calculate the detected distance vector from Lidar data

%-- match the distance matrix with reflector tables
m=0;n=0;
for j=1:length(Ref_dist_vector)
for i=1:length(detect_dist_vector)
    if abs(Ref_dist_vector(1,j)-detect_dist_vector(1,i))<threshold_distance_match
        m=m+1;
        n=n+1;
        matched_reflect_ID(m)=j;
        matched_detect_ID(n)=i;
        disp(sprintf('Matched reflector ID: %i', i));
    end
end
end
if matched_reflect_ID>0
disp('matched reflectors: ');
disp(sprintf('Reflector ID:-%i ', matched_reflect_ID));
else
    disp('No matched reflectors found');
end
