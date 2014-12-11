function [orientation center height]=localize(best_const1, best_const2, best_const3, best_const4)
p1_gnd=[0   14.5];
p2_gnd=[-11.655 8.741];
p3_gnd=[10.563 2.483];
p4_gnd=[0 -14.5];

% scale factor cm/pixels
unit_scale = 500./[48.0104;53.6004;98.8433;82.0975;90.1388;69.2892];
availability_matrix = ...
    [1 1 1 0 0 0;...
    1 0 0 1 1 0;...
    0 1 0 1 0 1;...
    0 0 1 0 1 1];

% matrix of our constellation and ground truth
p = [best_const1;best_const2;best_const3;best_const4];
p_gnd = [p1_gnd;p2_gnd;p3_gnd;p4_gnd];

% indices of valid points
valid = [sum(best_const1 ~= [1023 1023]) == 2; ...
    sum(best_const2 ~= [1023 1023]) == 2; ...
    sum(best_const3 ~= [1023 1023]) == 2; ...
    sum(best_const4 ~= [1023 1023]) == 2];
good_points = find(valid == 1);

if length(good_points) >2
    % vectors between each point
    vector = zeros(sum(1:length(good_points)-1),2);
    vector_gnd = zeros(sum(1:length(good_points)-1),2);
    iterator = 1;
    for i = 1:length(good_points)-1
        for j = i+1:length(good_points)
            vector(iterator,:) = p(good_points(i),:)-p(good_points(j),:);
            vector_gnd(iterator,:) = p_gnd(good_points(i),:)-p_gnd(good_points(j),:);
            iterator = iterator+1;
        end
    end
    % calculate angle with respect to vertical of our measured values
    theta = atan2(vector(:,2),vector(:,1))-atan2(vector_gnd(:,2),vector_gnd(:,1));
    theta(theta<-pi) = theta(theta<-pi)+2*pi;
    theta(sign(theta) ~= sign(theta(1))) = theta(sign(theta) ~= sign(theta(1)))*(-1);
    orientation = -mean(theta);
    
    % calculate center of constellation
    vector_norms = sqrt(sum(vector.^2,2));
    vector_gnd_norms = sqrt(sum(vector_gnd.^2,2));
    scale = sum(vector_norms)/sum(vector_gnd_norms);
    centers = zeros(length(good_points),2);
    for i = 1:length(good_points)
        centers(i,:) = p(good_points(i),:)-scale*p_gnd(good_points(i),:).*[-sin(orientation) cos(orientation)];
    end
    center = mean(centers)-[1023 768]/2;
    
    % get a height estimate for scaling distance measure
    conversion_indices = find(sum(availability_matrix(good_points,:),1)==2);
    height = unit_scale(conversion_indices)'*vector_norms/length(good_points);
else
    orientation = nan;
    center = [nan nan];
    height = nan;
end