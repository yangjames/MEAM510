function [orientation center]=localize(best_const1, best_const2, best_const3, best_const4)
p1_gnd=[0   14.5];
p2_gnd=[-11.655 8.741];
p3_gnd=[10.563 2.483];
p4_gnd=[0 -14.5];

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
    % calculate angle with respect to horizontal of our measured values
    theta = atan2(vector(:,2),vector(:,1))-atan2(vector_gnd(:,2),vector_gnd(:,1));
    theta(theta<-pi) = theta(theta<-pi)+2*pi;
    theta(sign(theta) ~= sign(theta(1))) = theta(sign(theta) ~= sign(theta(1)))*(-1);
    orientation = mean(theta);
    
    % calculate center of constellation
    vector_norms = sqrt(sum(vector.^2,2));
    vector_gnd_norms = sqrt(sum(vector_gnd.^2,2));
    scale = sum(vector_norms)/sum(vector_gnd_norms);
    centers = zeros(length(good_points),2);
    for i = 1:length(good_points)
        centers(i,:) = p(good_points(i),:)-scale*p_gnd(good_points(i),:).*[-sin(orientation) cos(orientation)];
    end
    center = mean(centers);
else
    orientation = nan;
    center = [nan nan];
end