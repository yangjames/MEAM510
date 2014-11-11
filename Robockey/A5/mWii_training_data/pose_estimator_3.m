%% ground truth values of each marker in world frame
p1_gnd = [10.563 2.483];
p2_gnd = [0 14.5];
p3_gnd = [-11.655 8.741];
p4_gnd = [0 -14.5];
p_gnd = [p1_gnd; p2_gnd; p3_gnd; p4_gnd];

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

%% import data
load('B.mat')
idx = [1 5; 2 6; 3 7; 4 8];
for d = 1:10%size(rawStarData,1)
    p_ptp = zeros(4,4,2);
    for i = 1:4
        for j = 1:4
            p_ptp(i,j,:)=rawStarData(d,idx(j,:))-rawStarData(d,idx(i,:));
        end
    end
    for t = 1:4
        iterator = 1;
        for i = 1:2
            for j = i+1:3
                for k=j+1:4
                    triangles(iterator,:) = [acos([p_ptp(j,i,1) p_ptp(j,i,2)]*[p_ptp(j,k,1) p_ptp(j,k,2)]'/(norm([p_ptp(j,i,1) p_ptp(j,i,2)])*norm([p_ptp(j,k,1) p_ptp(j,k,2)]))) acos([p_ptp(i,j,1) p_ptp(i,j,2)]*[p_ptp(i,k,1) p_ptp(i,k,2)]'/(norm([p_ptp(i,j,1) p_ptp(i,j,2)])*norm([p_ptp(i,k,1) p_ptp(i,k,2)]))) acos([p_ptp(k,j,1) p_ptp(k,j,2)]*[p_ptp(k,i,1) p_ptp(k,i,2)]'/(norm([p_ptp(k,j,1) p_ptp(k,j,2)])*norm([p_ptp(k,i,1) p_ptp(k,i,2)])))];
                    iterator = iterator + 1;
                end
            end
        end      
    sort(triangles_gnd,2)*180/pi
    sort(triangles,2)*180/pi
    end
end