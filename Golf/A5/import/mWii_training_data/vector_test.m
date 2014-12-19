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
rad_conv = 180/pi;
theta1 = atan2(p1_dir(2),p1_dir(1));
theta2 = atan2(p2_dir(2),p2_dir(1));
theta3 = atan2(p3_dir(2),p3_dir(1));
theta4 = atan2(p4_dir(2),p4_dir(1));

%% vector differences
p12 = p1_gnd - p2_gnd;
p13 = p1_gnd - p3_gnd;
p14 = p1_gnd - p4_gnd;
p23 = p2_gnd - p3_gnd;
p24 = p2_gnd - p4_gnd;
p34 = p3_gnd - p4_gnd;

%% normalized difference vectors
p12_dir = p12/norm(p12);
p13_dir = p13/norm(p13);
p14_dir = p14/norm(p14);
p23_dir = p23/norm(p23);
p24_dir = p24/norm(p24);
p34_dir = p34/norm(p34);

%% absolute angles from horizontal x of the ground truth vector differences
theta12 = atan2(p12_dir(2),p12_dir(1));
theta13 = atan2(p13_dir(2),p13_dir(1));
theta14 = atan2(p14_dir(2),p14_dir(1));
theta23 = atan2(p23_dir(2),p23_dir(1));
theta24 = atan2(p24_dir(2),p24_dir(1));
theta34 = atan2(p34_dir(2),p34_dir(1));

R12 = [cos(theta12) sin(theta12); -sin(theta12) cos(theta12)];

a=[0 1];
a_dir = a/norm(a);

angle_est = acos(a_dir*p12_dir')*180/pi;
a_norm = R12*a_dir';
if a_norm(2)<0
    angle_est = -angle_est
else
    angle_est
end

figure(3)
clf
hold on
grid on
axis equal
set(gca,'xlim',[-1 1],'ylim',[-1 1]);
plot([0 a_dir(1)],[0 a_dir(2)],'r-*')
plot([0 p12_dir(1)],[0 p12_dir(2)],'b-*')