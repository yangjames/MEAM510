clear all
load('A.mat')

offset = 1/2*[1024 1024];
scale = 5;

%% ground truth values of each marker in world frame
p1_gnd = [10.563 2.483];
p2_gnd = [0 14.5];
p3_gnd = [-11.655 8.741];
p4_gnd = [0 -14.5];

%% direction vectors of each point
p1_dir = p1_gnd/norm(p1_gnd);
p2_dir = p2_gnd/norm(p2_gnd);
p3_dir = p3_gnd/norm(p3_gnd);
p4_dir = p4_gnd/norm(p4_gnd);

%% absolute angles from horizontal x of each ground truth marker
% rad_conv = 180/pi;
% theta1 = atan2(p1_dir(2),p1_dir(1));
% theta2 = atan2(p2_dir(2),p2_dir(1));
% theta3 = atan2(p3_dir(2),p3_dir(1));
% theta4 = atan2(p4_dir(2),p4_dir(1));

%% vector differences
p12 = p1_gnd - p2_gnd;
p14 = p1_gnd - p4_gnd;
p23 = p2_gnd - p3_gnd;
p34 = p3_gnd - p4_gnd;


%% normalized difference vectors
p12_dir = p12/norm(p12);
p14_dir = p14/norm(p14);
p23_dir = p23/norm(p23);
p34_dir = p34/norm(p34);

%% angles between points
theta1 = acos(p12_dir*p13_dir');
theta2 = acos(

%% absolute angles from horizontal x of the ground truth vector differences
theta12 = atan2(p12_dir(2),p12_dir(1));
theta13 = atan2(p13_dir(2),p13_dir(1));
theta14 = atan2(p14_dir(2),p14_dir(1));
theta23 = atan2(p23_dir(2),p23_dir(1));
theta24 = atan2(p24_dir(2),p24_dir(1));
theta34 = atan2(p34_dir(2),p34_dir(1));


%% rotation matrices of difference vector from ground truth
R12 = getrot(theta12);
R13 = getrot(theta13);
R14 = getrot(theta14);
R23 = getrot(theta23);
R24 = getrot(theta24);
R34 = getrot(theta34);

figure(1)
clf
axis equal
grid on
hold on
set(gca,'xlim',[0 1023],'ylim',[0 1023]);

p1_gnd_plot = plot(p1_gnd(1,1),p1_gnd(1,2),'r*');
p2_gnd_plot = plot(p2_gnd(1,1),p2_gnd(1,2),'b*');
p3_gnd_plot = plot(p3_gnd(1,1),p3_gnd(1,2),'g*');
p4_gnd_plot = plot(p4_gnd(1,1),p4_gnd(1,2),'k*');
p1_plot = plot(p1(1,1),p1(1,2),'r*');
p2_plot = plot(p2(1,1),p2(1,2),'b*');
p3_plot = plot(p3(1,1),p2(1,2),'g*');
p4_plot = plot(p4(1,1),p2(1,2),'k*');


for i=1:size(rawStarData,1)
    % calculate point-to-point vectors

    % check validity of points

    % estimate angle
    
    % redraw
    set(p1_plot,'xdata',p1(i,1),'ydata',p1(i,2));
    set(p2_plot,'xdata',p2(i,1),'ydata',p2(i,2));
    set(p3_plot,'xdata',p3(i,1),'ydata',p3(i,2));
    set(p4_plot,'xdata',p4(i,1),'ydata',p4(i,2));
    drawnow
    pause(0.01)
end