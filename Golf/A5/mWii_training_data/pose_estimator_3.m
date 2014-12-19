clear all
%% ground truth values of each marker in world frame
p1_gnd = [10.563 2.483];
p2_gnd = [0 14.5];
p3_gnd = [-11.655 8.741];
p4_gnd = [0 -14.5];
p_gnd = [p1_gnd; p2_gnd; p3_gnd; p4_gnd];

threshold = 0.05;

p_ptp_gnd = zeros(4,4,2);
for i = 1:4
    for j = 1:4
        p_ptp_gnd(i,j,:)=p_gnd(j,:)-p_gnd(i,:);
    end
end
iterator = 1;
for i = 1:2
    for j = i+1:3
        for k=j+1:4
            triangles_gnd(iterator,:) = [acos([p_ptp_gnd(j,i,1) p_ptp_gnd(j,i,2)]*[p_ptp_gnd(j,k,1) p_ptp_gnd(j,k,2)]'/(norm([p_ptp_gnd(j,i,1) p_ptp_gnd(j,i,2)])*norm([p_ptp_gnd(j,k,1) p_ptp_gnd(j,k,2)]))) acos([p_ptp_gnd(i,j,1) p_ptp_gnd(i,j,2)]*[p_ptp_gnd(i,k,1) p_ptp_gnd(i,k,2)]'/(norm([p_ptp_gnd(i,j,1) p_ptp_gnd(i,j,2)])*norm([p_ptp_gnd(i,k,1) p_ptp_gnd(i,k,2)]))) acos([p_ptp_gnd(k,j,1) p_ptp_gnd(k,j,2)]*[p_ptp_gnd(k,i,1) p_ptp_gnd(k,i,2)]'/(norm([p_ptp_gnd(k,j,1) p_ptp_gnd(k,j,2)])*norm([p_ptp_gnd(k,i,1) p_ptp_gnd(k,i,2)])))];
            iterator = iterator + 1;
        end
    end
end

[biggest_gnd_angles indices_gnd_big]= max(triangles_gnd,[],2);
[biggest_gnd_angle index_gnd_big]= max(biggest_gnd_angles);
[smallest_gnd_angles indices_gnd_small] = min(triangles_gnd,[],2);
[smallest_gnd_angle index_gnd_small] = min(smallest_gnd_angles);
indicator = [2 1 3; 2 1 4; 3 1 4; 3 2 4];

%% import data
load('B.mat')
idx = [1 5; 2 6; 3 7; 4 8];

figure(1)
clf
grid on
hold on
axis equal
set(gca,'xlim',[0 1023], 'ylim',[0 1023]);

p1_plot = plot(rawStarData(1,idx(1,1)),rawStarData(1,idx(1,2)),'r-*');
p2_plot = plot(rawStarData(1,idx(2,1)),rawStarData(1,idx(2,2)),'g-*');
p3_plot = plot(rawStarData(1,idx(3,1)),rawStarData(1,idx(3,2)),'b-*');
p4_plot = plot(rawStarData(1,idx(4,1)),rawStarData(1,idx(4,2)),'k-*');
for d = 1:size(rawStarData,1)
   % if (
    % create the vectors between each measured star
    p_ptp = zeros(4,4,2);
    for i = 1:4
        for j = 1:4
            p_ptp(i,j,:)=rawStarData(d,idx(j,:))-rawStarData(d,idx(i,:));
        end
    end
    
    % make triangles from the vectors
    iterator = 1;
    for i = 1:2
        for j = i+1:3
            for k=j+1:4
                triangles(iterator,:) = [acos([p_ptp(j,i,1) p_ptp(j,i,2)]*[p_ptp(j,k,1) p_ptp(j,k,2)]'/(norm([p_ptp(j,i,1) p_ptp(j,i,2)])*norm([p_ptp(j,k,1) p_ptp(j,k,2)])))...
                    acos([p_ptp(i,j,1) p_ptp(i,j,2)]*[p_ptp(i,k,1) p_ptp(i,k,2)]'/(norm([p_ptp(i,j,1) p_ptp(i,j,2)])*norm([p_ptp(i,k,1) p_ptp(i,k,2)])))...
                    acos([p_ptp(k,j,1) p_ptp(k,j,2)]*[p_ptp(k,i,1) p_ptp(k,i,2)]'/(norm([p_ptp(k,j,1) p_ptp(k,j,2)])*norm([p_ptp(k,i,1) p_ptp(k,i,2)])))];
                iterator = iterator + 1;
            end
        end
    end      
    
    % find the biggest angle and the point it's associated with
    [biggest_angles, indices] = max(triangles,[],2); % biggest angle of each triangle
    [biggest_angle, index] = max(biggest_angles); % biggest of the biggest
    
    % check for an angle that's obviously too big
    bad_points = [0 0 0 0];
    if biggest_angle > biggest_gnd_angle*(1+threshold)
        bad_points(indicator(index,indices(index))) = 1;
    end
    % get index of the unaffected triangle and check if it matches one
    % of the ground truth triangles
    if sum(bad_points)>0
        triangle_index = find(sum(~ismember(indicator,find(bad_points==1)),2)==3);
    else
        triangle_index = [];
    end
    % check for an angle that's obviously too small, masked with the bad
    % point we may have found for finding an angle too big
    if ~isempty(triangle_index)
        [smallest_angle, index_small] = min(triangles(triangle_index,:));
        if smallest_angle < smallest_gnd_angle*(1-threshold)
            bad_points = [1 1 1 1];
        end
    else
        [smallest_angles, indices_small] = min(triangles,[],2);
        [smallest_angle, index_small] = min(smallest_angles);
        bad_points(indicator(index_small,indices_small(index_small)))=1;
    end
    
    bad_points
    % if we have more than 1 bad point, we can't do anything
    if sum(bad_points < 2)
        
        sorted_triangle = sort(triangles(triangle_index,:))
        triangles
        fprintf('found a bad point: %d\n', d)
    end
    % redraw
    set(p1_plot,'xdata',rawStarData(d,idx(1,1)),'ydata',rawStarData(i,idx(1,2)));
    set(p2_plot,'xdata',rawStarData(d,idx(2,1)),'ydata',rawStarData(i,idx(2,2)));
    set(p3_plot,'xdata',rawStarData(d,idx(3,1)),'ydata',rawStarData(i,idx(3,2)));
    set(p4_plot,'xdata',rawStarData(d,idx(4,1)),'ydata',rawStarData(i,idx(4,2)));
    drawnow
    pause(0.01)
end