clear all
%% ground truth values of each marker in world frame
p1_gnd = [10.563 2.483];
p2_gnd = [0 14.5];
p3_gnd = [-11.655 8.741];
p4_gnd = [0 -14.5];
p_gnd = [p1_gnd; p2_gnd; p3_gnd; p4_gnd];


gnd_difference_vecs = zeros(12,2);
iterator = 1;
for i = 1:3
    for j = i+1:4
        gnd_difference_vecs(iterator,:) = p_gnd(i,:)-p_gnd(j,:);
        gnd_difference_vecs(iterator+6,:) = -gnd_difference_vecs(iterator,:);
        iterator = iterator + 1;
    end
end

load('A.mat')
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
    % get difference vectors
    iterator = 1;
    difference_vecs = zeros(6,2);
    q = [rawStarData(d,idx(1,:)); ...
        rawStarData(d,idx(2,:)); ...
        rawStarData(d,idx(3,:)); ...
        rawStarData(d,idx(4,:))];
    for i = 1:3
        for j = i+1:4
            difference_vecs(iterator,:) = q(i,:)-q(j,:);
            iterator = iterator + 1;
        end
    end
    
    theta = zeros(66,1);
    iterator = 1;
    for i = 1:6
        for j = 1:12
            if i == j
                continue;
            end
            theta(iterator) = acos(difference_vecs(i,:)*gnd_difference_vecs(j,:)'/(norm(difference_vecs(i,:))*norm(gnd_difference_vecs(j,:))));
            iterator = iterator + 1;
        end
    end
    
    threshold = 0.05;
    bins = [theta(1)];
    counter = [1];
    found = 0;
    for i = 2:66
        iterator = 1;
        while (~found && iterator < length(bins))
            if (1-threshold<theta(i)/bins(iterator)) && (theta(i)/bins(iterator) < 1+threshold)
                counter(iterator) = counter(iterator)+1;
                found = 1;
            end
            iterator = iterator+1;
        end
        if ~found
            bins = [bins theta(i)];
            counter = [counter 1];
        end
        found = 0;
    end
    % redraw
    set(p1_plot,'xdata',rawStarData(d,idx(1,1)),'ydata',rawStarData(d,idx(1,2)));
    set(p2_plot,'xdata',rawStarData(d,idx(2,1)),'ydata',rawStarData(d,idx(2,2)));
    set(p3_plot,'xdata',rawStarData(d,idx(3,1)),'ydata',rawStarData(d,idx(3,2)));
    set(p4_plot,'xdata',rawStarData(d,idx(4,1)),'ydata',rawStarData(d,idx(4,2)));
    drawnow;
    pause(0.01);
    [count, index]=max(counter);
    bins(index)*180/pi
end