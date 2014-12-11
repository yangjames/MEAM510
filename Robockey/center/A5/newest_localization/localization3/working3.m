
clear all
load('B.mat')


%%
s1=[rawStarData(:,1),rawStarData(:,5)];
s2=[rawStarData(:,2),rawStarData(:,6)];
s3=[rawStarData(:,3),rawStarData(:,7)];
s4=[rawStarData(:,4),rawStarData(:,8)];


%%
i=1;


%[const1_123 const2_123 const3_123 const4_123, error(1)]=point_match(star1,star2,star3);

% [best_const1, best_const2, best_const3, best_const4, error]=best_triangle(star1,star2,star3,star4);

% [best_const1, best_const2, best_const3, best_const4, error]=likely_solution(star1,star2,star3,star4);


xPos_history=[];
yPos_history=[];
angleOfRotGlobal=[];
b=1;

for i=1:size(rawStarData,1)

star1=s1(i,:);
star2=s2(i,:);
star3=s3(i,:);
star4=s4(i,:);

[best_const1, best_const2, best_const3, best_const4, error]=stupid_solution(star1,star2,star3,star4);
[orientation center height]=localize(best_const1, best_const2, best_const3, best_const4);


% test code
% center=((best_const1+best_const4)/2)-(1023/2);
% vect=(best_const1-best_const4);
%orientation=-((atan2(vect(2),vect(1)))-(pi/2));
%%%%% orientation=-((atan2(vect(2),vect(1)))+(pi/2));


%height;
r = 750*center*32/1024*pi/180; % briefly replaced height with 750,

%r=center;

angleOfRot=-orientation;
xPos=r(1);
yPos=r(2);


angleOfRot2 = angleOfRot;
RotCtoG = [cos(angleOfRot2) sin(angleOfRot2);
          -sin(angleOfRot2) cos(angleOfRot2)]; % rotate from frame C to G


% RotCtoG = [cos(angleOfRot2) -sin(angleOfRot2);
%           sin(angleOfRot2) cos(angleOfRot2)]; % rotate from frame C to G

posRobot = RotCtoG*[-xPos; -yPos];


xPos_history(i) = posRobot(1); % negative because looking down instead of up
yPos_history(i) = posRobot(2);
angleOfRotGlobal(i) = angleOfRot2;


% [xPosGlobal, yPosGlobal, angleOfRotGlobal,plot1,plot2,plot3] = location_transformation(r(1), r(2), orientation);
%[xPos_history(b),yPos_history(b)]=update_plot(plot1,plot2,plot3,plot4,plot5,plot6, r(1), r(2), -orientation); % note this negative orientation
b=b+1;



end

figure(7)
plot(xPos_history,yPos_history)
figure(8)
c=1
quiver(xPos_history,yPos_history,c*sin(angleOfRotGlobal),c*cos(angleOfRotGlobal))
figure(9)
plot(angleOfRotGlobal*180/pi)