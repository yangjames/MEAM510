%% ground truth values of each marker in world frame
p1_gnd = [10.563 2.483];
p2_gnd = [0 14.5];
p3_gnd = [-11.655 8.741];
p4_gnd = [0 -14.5];
p_gnd = [p1_gnd; p2_gnd; p3_gnd; p4_gnd];

%% ground truth point-to-point vectors
p12_gnd = p2_gnd-p1_gnd; % 1 -> 2
p21_gnd = p1_gnd-p2_gnd; % 2 -> 1
p23_gnd = p3_gnd-p2_gnd; % 2 -> 3
p32_gnd = p2_gnd-p3_gnd; % 3 -> 2
p34_gnd = p4_gnd-p3_gnd; % 3 -> 4
p43_gnd = p3_gnd-p4_gnd; % 4 -> 3
p14_gnd = p4_gnd-p1_gnd; % 1 -> 4
p41_gnd = p1_gnd-p4_gnd; % 4 -> 1
p13_gnd = p3_gnd-p1_gnd; % 1 -> 3
p24_gnd = p4_gnd-p2_gnd; % 2 -> 4

p_ptp_gnd = zeros(4,4,2);
for i = 1:4
    for j = 1:4
        p_ptp_gnd(i,j,:)=p_gnd(j,:)-p_gnd(i,:);
    end
end

% point to point normalized vectors
p12_gnd_norm = p12_gnd/norm(p12_gnd);
p21_gnd_norm = p21_gnd/norm(p21_gnd);
p23_gnd_norm = p23_gnd/norm(p23_gnd);
p32_gnd_norm = p32_gnd/norm(p32_gnd);
p34_gnd_norm = p34_gnd/norm(p34_gnd);
p43_gnd_norm = p43_gnd/norm(p43_gnd);
p14_gnd_norm = p14_gnd/norm(p14_gnd);
p41_gnd_norm = p41_gnd/norm(p41_gnd);
% additional vectors
p13_gnd_norm = p13_gnd/norm(p13_gnd);
p31_gnd_norm = p31_gnd/norm(p31_gnd);
p24_gnd_norm = p24_gnd/norm(p24_gnd);
p42_gnd_norm = p42_gnd/norm(p42_gnd);

%% ground truth angles between each point
theta1_gnd = acos(p12_gnd_norm*p14_gnd_norm');
theta2_gnd = acos(p23_gnd_norm*p21_gnd_norm');
theta3_gnd = acos(p32_gnd_norm*p34_gnd_norm');
theta4_gnd = acos(p41_gnd_norm*p43_gnd_norm');

%% angles between p-to-p norm vectors and horizontal x axis
beta12_gnd = atan2(p12_gnd_norm(2),p12_gnd_norm(1));
beta21_gnd = atan2(p21_gnd_norm(2),p21_gnd_norm(1));
beta23_gnd = atan2(p23_gnd_norm(2),p23_gnd_norm(1));
beta32_gnd = atan2(p32_gnd_norm(2),p32_gnd_norm(1));
beta34_gnd = atan2(p34_gnd_norm(2),p34_gnd_norm(1));
beta43_gnd = atan2(p43_gnd_norm(2),p43_gnd_norm(1));
beta14_gnd = atan2(p14_gnd_norm(2),p14_gnd_norm(1));
beta41_gnd = atan2(p41_gnd_norm(2),p41_gnd_norm(1));
beta13_gnd = atan2(p13_gnd_norm(2),p13_gnd_norm(1));
beta31_gnd = atan2(p31_gnd_norm(2),p31_gnd_norm(1));
beta24_gnd = atan2(p24_gnd_norm(2),p24_gnd_norm(1));
beta42_gnd = atan2(p42_gnd_norm(2),p42_gnd_norm(1));

beta = [0 beta12_gnd beta13_gnd beta14_gnd;...
    beta21_gnd 0 beta23_gnd beta24_gnd;...
    beta31_gnd beta32_gnd 0 beta34_gnd;...
    beta41_gnd beta42_gnd beta43_gnd 0];

%% load data
load('A.mat');
p1 = [1 5];
p2 = [2 6];
p3 = [3 7];
p4 = [4 8];
p = [p1; p2; p3; p4];

figure(1)
clf
grid on
hold on
axis equal
set(gca,'xlim',[0 1023], 'ylim',[0 1023]);

scale = 5;
offset = 0.5*[1023 1023];

p1_gnd_plot = plot(scale*p1_gnd(1)+offset(1),scale*p1_gnd(2)+offset(2),'r-*');
p2_gnd_plot = plot(scale*p2_gnd(1)+offset(1),scale*p2_gnd(2)+offset(2),'g-*');
p3_gnd_plot = plot(scale*p3_gnd(1)+offset(1),scale*p3_gnd(2)+offset(2),'b-*');
p4_gnd_plot = plot(scale*p4_gnd(1)+offset(1),scale*p4_gnd(2)+offset(2),'k-*');

p1_plot = plot(rawStarData(1,p1(1)),rawStarData(1,p1(2)),'r-*');
p2_plot = plot(rawStarData(1,p2(1)),rawStarData(1,p2(2)),'g-*');
p3_plot = plot(rawStarData(1,p3(1)),rawStarData(1,p3(2)),'b-*');
p4_plot = plot(rawStarData(1,p4(1)),rawStarData(1,p4(2)),'k-*');

invalid = [1023 1023];
for i = 1:size(rawStarData,1)
    % check which points are valid
    valid = [sum(rawStarData(i,p1) ~= invalid) == 2;...
        sum(rawStarData(i,p2) ~= invalid) == 2;...
        sum(rawStarData(i,p3) ~= invalid) == 2;...
        sum(rawStarData(i,p4) ~= invalid) == 2];
    
    % get the indices of the valid points
    idx = find(valid == 1);
    
    % check if we have more than one point
    if length(idx) > 1
        % get the difference vectors between the valid points and get the
        % corresponding groun truth difference vectors
        vec = zeros(sum(1:length(idx)-1),2);
        idx_gnd = zeros(sum(1:length(idx)-1),2);
        idx2 = 1;
        for j = 1:length(idx)-1
            for k = j+1:length(idx)
                vec(idx2,:) = rawStarData(i,p(idx(k),:))-rawStarData(i,p(idx(j),:));
                idx_gnd(idx2,:) = [j k];
                idx2 = idx2+1;
            end
        end
        
        % normalize the difference vectors
        vec_norm = vec./repmat(sqrt(sum(vec.^2,2)),1,2);
        
        % get the corresponding ground truth difference vectors
        vec_gnd = zeros(sum(1:length(idx)-1),2);
        for j = 1:size(idx_gnd,1)
            vec_gnd(j,:) = p_ptp_gnd(idx_gnd(j,1),idx_gnd(j,2),:);
        end
        
        % normalize the ground truth difference vectors
        vec_gnd_norm = vec_gnd./repmat(sqrt(sum(vec_gnd.^2,2)),1,2);
        
        % get angles between each of the difference vectors
        angles = zeros(sum(1:length(idx)-1),1);
        for j = 1:size(idx_gnd,1)
            angles(j) = acos(vec_gnd_norm(j,:)*vec_norm(j,:)');
        end
        angle = mean(angles);
        % get angles of ground truth difference vectors wrt horizontal
        beta_gnd = beta(idx_gnd(1,1),idx_gnd(1,2));
        rot = [cos(beta_gnd) sin(beta_gnd); -sin(beta_gnd) cos(beta_gnd)];
        projected_vec = rot*vec_norm(1,:)';
        if projected_vec(2) < 0
            angle = -angle;
        end
        angle*180/pi
    end
    % redraw
    set(p1_plot,'xdata',rawStarData(i,p1(1)),'ydata',rawStarData(i,p1(2)));
    set(p2_plot,'xdata',rawStarData(i,p2(1)),'ydata',rawStarData(i,p2(2)));
    set(p3_plot,'xdata',rawStarData(i,p3(1)),'ydata',rawStarData(i,p3(2)));
    set(p4_plot,'xdata',rawStarData(i,p4(1)),'ydata',rawStarData(i,p4(2)));
    drawnow
    pause(0.01)
end