%%
orientation=pi/2;
xPos=20;
yPos=0;

%%


angleOfRot=-orientation;
% xPos=r(1);
% yPos=r(2);


angleOfRot2 = angleOfRot;
RotCtoG = [cos(angleOfRot2) sin(angleOfRot2);
          -sin(angleOfRot2) cos(angleOfRot2)]; % rotate from frame C to G


% RotCtoG = [-cos(angleOfRot2) sin(angleOfRot2);
%           sin(angleOfRot2) cos(angleOfRot2)]; % rotate from frame C to G

posRobot = RotCtoG*[-xPos; -yPos]

xPos_history = posRobot(1); % negative because looking down instead of up
yPos_history = posRobot(2);
angleOfRotGlobal = angleOfRot2