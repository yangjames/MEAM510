%% ground truth values
p1_gnd = [10.563 2.483];
p2_gnd = [0 14.5];
p3_gnd = [-11.655 8.741];
p4_gnd = [0 -14.5];

% ground truth unit vectors
p1_gnd_norm = p1_gnd/norm(p1_gnd);
p2_gnd_norm = p2_gnd/norm(p2_gnd);
p3_gnd_norm = p3_gnd/norm(p3_gnd);
p4_gnd_norm = p4_gnd/norm(p4_gnd);

%% ground truth point-to-point vectors
p12_gnd = p2_gnd-p1_gnd; % 1 -> 2
p21_gnd = p1_gnd-p2_gnd; % 2 -> 1
p23_gnd = p3_gnd-p2_gnd; % 2 -> 3
p32_gnd = p2_gnd-p3_gnd; % 3 -> 2
p34_gnd = p4_gnd-p3_gnd; % 3 -> 4
p43_gnd = p3_gnd-p4_gnd; % 4 -> 3
p14_gnd = p4_gnd-p1_gnd; % 1 -> 4
p41_gnd = p1_gnd-p4_gnd; % 4 -> 1
% additional vectors
p13_gnd = p3_gnd-p1_gnd; % 1 -> 3
p31_gnd = p1_gnd-p3_gnd; % 3 -> 1
p24_gnd = p4_gnd-p2_gnd; % 2 -> 4
p42_gnd = p2_gnd-p4_gnd; % 4 -> 2
% associated normalized vectors
p12_gnd_norm = p12_gnd/norm(p12_gnd);
p21_gnd_norm = p21_gnd/norm(p21_gnd);
p23_gnd_norm = p23_gnd/norm(p23_gnd);
p32_gnd_norm = p32_gnd/norm(p32_gnd);
p34_gnd_norm = p34_gnd/norm(p34_gnd);
p43_gnd_norm = p43_gnd/norm(p43_gnd);
p14_gnd_norm = p14_gnd/norm(p14_gnd);
p41_gnd_norm = p41_gnd/norm(p41_gnd);

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

%% rotation matrices for getting p-to-p frames
R12 = [cos(beta12_gnd) sin(beta12_gnd); -sin(beta12_gnd) cos(beta12_gnd)];
R21 = [cos(beta21_gnd) sin(beta21_gnd); -sin(beta21_gnd) cos(beta21_gnd)];
R23 = [cos(beta23_gnd) sin(beta23_gnd); -sin(beta23_gnd) cos(beta23_gnd)];
R32 = [cos(beta32_gnd) sin(beta32_gnd); -sin(beta32_gnd) cos(beta32_gnd)];
R34 = [cos(beta34_gnd) sin(beta34_gnd); -sin(beta34_gnd) cos(beta34_gnd)];
R43 = [cos(beta43_gnd) sin(beta43_gnd); -sin(beta43_gnd) cos(beta43_gnd)];
R14 = [cos(beta14_gnd) sin(beta14_gnd); -sin(beta14_gnd) cos(beta14_gnd)];
R41 = [cos(beta41_gnd) sin(beta41_gnd); -sin(beta41_gnd) cos(beta41_gnd)];

%% import data
load('A.mat');
p1 = rawStarData(:,[1 5]);
p2 = rawStarData(:,[2 6]);
p3 = rawStarData(:,[3 7]);
p4 = rawStarData(:,[4 8]);

scale = 5;
offset = 0.5*[1023 1023];

figure(1)
clf
grid on
hold on
axis equal
set(gca,'xlim',[0 1023], 'ylim',[0 1023]);

p1_gnd_plot = plot(scale*p1_gnd(1)+offset(1),scale*p1_gnd(2)+offset(2),'r-*');
p2_gnd_plot = plot(scale*p2_gnd(1)+offset(1),scale*p2_gnd(2)+offset(2),'g-*');
p3_gnd_plot = plot(scale*p3_gnd(1)+offset(1),scale*p3_gnd(2)+offset(2),'b-*');
p4_gnd_plot = plot(scale*p4_gnd(1)+offset(1),scale*p4_gnd(2)+offset(2),'k-*');

p1_plot = plot(p1(1,1),p1(1,2),'r-*');
p2_plot = plot(p2(1,1),p2(1,2),'g-*');
p3_plot = plot(p3(1,1),p3(1,2),'b-*');
p4_plot = plot(p4(1,1),p4(1,2),'k-*');

for i = 1:size(rawStarData,1)
    % calculate point-to-point vectors
    p12 = p2(i,:)-p1(i,:);
    p23 = p3(i,:)-p2(i,:);
    p34 = p4(i,:)-p3(i,:);
    p41 = p1(i,:)-p4(i,:);
    p21 = -p12;
    p32 = -p23;
    p43 = -p34;
    p14 = -p41;

    % normalize all the vectors
    p12_norm = p12/norm(p12);
    p23_norm = p23/norm(p23);
    p34_norm = p34/norm(p34);
    p41_norm = p41/norm(p41);
    p21_norm = p21/norm(p21);
    p32_norm = p32/norm(p32);
    p43_norm = p43/norm(p43);
    p14_norm = p14/norm(p14);
    
    % check validity of the points
    %{
    valid = [0 0 0 0];
    valid(1) = p1(i,:) ~= [1023 1023];
    valid(2) = p2(i,:) ~= [1023 1023];
    valid(3) = p3(i,:) ~= [1023 1023];
    valid(4) = p4(i,:) ~= [1023 1023];
    %}
    % get angles of valid points
    angles = 0;
    %if (valid(1) && valid(2))
        
    %end
    
    % redraw
    set(p1_plot,'xdata',p1(i,1),'ydata',p1(i,2));
    set(p2_plot,'xdata',p2(i,1),'ydata',p2(i,2));
    set(p3_plot,'xdata',p3(i,1),'ydata',p3(i,2));
    set(p4_plot,'xdata',p4(i,1),'ydata',p4(i,2));
    drawnow
    pause(0.01)
end