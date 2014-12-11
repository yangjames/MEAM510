% Location Transformation
% MEAM 510 Localization

% Elizabeth Fedalei
% Due: 11/14/14


function [xPosGlobal, yPosGlobal, angleOfRotGlobal] = location_transformation(xPos, yPos, angleOfRot)
    % INPUT
    % xPos = X-position of the constellation as seen by the camera [cm]
    % yPos = Y-position of the constellation as seen by the camera [cm]
    % angOfRot = angle of rotation of the constellation from verital as
    % seen by the camera [rad]
  
   
% xPos = 40;
% yPos = -40;
% angleOfRot = pi/3;

% PLOT ALL THE THINGS
figure(3001)
clf
hold on

% Plot what the camera sees
subplot(1,2,1)
hold on

title('Camera View Looking Up','fontweight','bold','fontsize',20);
xlabel('X-Position [cm]','fontweight','bold','fontsize',16)
ylabel('Y-Position [cm]','fontweight','bold','fontsize',16)

% Plot Robot
robotRadius = 10;
robotColor1 = '-b';
robotLineWidth = 2;

robotPlotX = -robotRadius:.01:robotRadius;
robotPlotYtop = sqrt(robotRadius^2-robotPlotX.^2);
robotPlotYbottom = -sqrt(robotRadius^2-robotPlotX.^2);

plot(robotPlotX,robotPlotYtop,robotColor1,'linewidth',robotLineWidth)
plot(robotPlotX,robotPlotYbottom,robotColor1,'linewidth',robotLineWidth)
plot([0 0],[0 robotRadius],robotColor1,'linewidth',robotLineWidth)
% plot([0 robotRadius],[0 0],robotColor,'linewidth',robotLineWidth)

% Plot Axes
AxesLineWidth = 1;
AxesLineColor = ':k';
plot([-150 150],[0 0],AxesLineColor,'linewidth',AxesLineWidth)
plot([0 0],[-150 150],AxesLineColor,'linewidth',AxesLineWidth)

% Plot Constellation
consRadius = 15;
consColor = ':r';
consLineWidth = 2;

consPlotX = (xPos-consRadius):.01:(xPos+consRadius);
consPlotYtop = yPos + sqrt(consRadius^2-(consPlotX-xPos).^2);
consPlotYbottom = yPos - sqrt(consRadius^2-(consPlotX-xPos).^2);

plot(consPlotX,consPlotYtop,consColor,'linewidth',consLineWidth)
plot(consPlotX,consPlotYbottom,consColor,'linewidth',consLineWidth)
plot([xPos xPos-consRadius*sin(angleOfRot)],[yPos yPos+consRadius*cos(angleOfRot)],consColor,'linewidth',consLineWidth)
% plot([xPos xPos+consRadius*cos(angleOfRot)],[yPos yPos+consRadius*sin(angleOfRot)],consColor,'linewidth',consLineWidth)

% Format Camera View Axes
axis equal
axis([-100 100 -100 100]);


% Begin plotting in global frame
subplot(1,2,2)
hold on

title('View of Rink Looking Down','fontweight','bold','fontsize',20);
xlabel('X-Position [cm]','fontweight','bold','fontsize',16)
ylabel('Y-Position [cm]','fontweight','bold','fontsize',16)

% Plot Rink Outline
xLength = 230;
yLength = 120;
filletRadius = 40;
RinkXRight = (xLength/2-filletRadius):.01:xLength/2;
RinkXLeft = -xLength/2:.01:-(xLength/2-filletRadius);
filletRightTop = (yLength/2-filletRadius)+sqrt(filletRadius^2-(RinkXRight-(xLength/2-filletRadius)).^2);
filletRightBottom = -(yLength/2-filletRadius)-sqrt(filletRadius^2-(RinkXRight-(xLength/2-filletRadius)).^2);
filletLeftTop = (yLength/2-filletRadius)+sqrt(filletRadius^2-(RinkXLeft+(xLength/2-filletRadius)).^2);
filletLeftBottom = -(yLength/2-filletRadius)-sqrt(filletRadius^2-(RinkXLeft+(xLength/2-filletRadius)).^2);

RinkOuterLineWidth = 2;
RinkOuterLineColor = '-k';

plot([-(xLength/2-filletRadius), (xLength/2-filletRadius)],[yLength/2 yLength/2],RinkOuterLineColor,'linewidth',RinkOuterLineWidth)
plot([xLength/2 xLength/2],[(yLength/2-filletRadius) -(yLength/2-filletRadius)],RinkOuterLineColor,'linewidth',RinkOuterLineWidth)
plot([(xLength/2-filletRadius) -(xLength/2-filletRadius)],[-yLength/2 -yLength/2],RinkOuterLineColor,'linewidth',RinkOuterLineWidth)
plot([-xLength/2 -xLength/2],[-(yLength/2-filletRadius) (yLength/2-filletRadius)],RinkOuterLineColor,'linewidth',RinkOuterLineWidth)
plot(RinkXRight,filletRightTop,RinkOuterLineColor,'linewidth',RinkOuterLineWidth)
plot(RinkXRight,filletRightBottom,RinkOuterLineColor,'linewidth',RinkOuterLineWidth)
plot(RinkXLeft,filletLeftTop,RinkOuterLineColor,'linewidth',RinkOuterLineWidth)
plot(RinkXLeft,filletLeftBottom,RinkOuterLineColor,'linewidth',RinkOuterLineWidth)

RinkInnerLineWidth = 1;
RinkInnerLineColor = '-k';
plot([40 40],[yLength/2 -yLength/2],RinkInnerLineColor,'linewidth',RinkInnerLineWidth)
plot([-40 -40],[yLength/2 -yLength/2],RinkInnerLineColor,'linewidth',RinkInnerLineWidth)

% Plot Axes
AxesLineWidth = 1;
AxesLineColor = ':k';
plot([xLength/2 -xLength/2],[0 0],AxesLineColor,'linewidth',AxesLineWidth)
plot([0 0],[yLength/2 -yLength/2],AxesLineColor,'linewidth',AxesLineWidth)


% ROBOT THINGS
% Calculate Robot Position and Rotation
RotCtoG = [cos(angleOfRot) sin(angleOfRot);
          -sin(angleOfRot) cos(angleOfRot)]; % rotate from frame C to G
posRobot = RotCtoG*[-xPos; -yPos];
xPosRobot = -posRobot(1); % negative because looking down instead of up
yPosRobot = posRobot(2);

% Plot Robot Position and Rotation
robotPlotXGlobal = (xPosRobot-robotRadius):.01:(xPosRobot+robotRadius);
robotPlotYtopGlobal = yPosRobot+sqrt(robotRadius^2-(robotPlotXGlobal-xPosRobot).^2);
robotPlotYbottomGlobal = yPosRobot-sqrt(robotRadius^2-(robotPlotXGlobal-xPosRobot).^2);

robotColor2 = ':b';
plot(robotPlotXGlobal,robotPlotYtopGlobal,robotColor2,'linewidth',robotLineWidth)
plot(robotPlotXGlobal,robotPlotYbottomGlobal,robotColor2,'linewidth',robotLineWidth)
plot([xPosRobot xPosRobot-robotRadius*sin(angleOfRot)],[yPosRobot yPosRobot+robotRadius*cos(angleOfRot)],robotColor2,'linewidth',robotLineWidth)
% plot([XposRobot XposRobot+robotRadius*cos(angleOfRot)],[YposRobot YposRobot+robotRadius*sin(angleOfRot)],robotColor,'linewidth',robotLineWidth)


% Plot Constellation
ConstellationX = [-10.563 0 0 11.655];
ConstellationY = [2.483 -14.5 14.5 8.741];

ConstellationLineColor = 'pr';
ConstellationFaceColor = 'r';
ConstellationSize = 16;

plot(ConstellationX, ConstellationY, ConstellationLineColor,'markersize',ConstellationSize,'markerfacecolor',ConstellationFaceColor)
plot([0 0],[0 consRadius],ConstellationFaceColor,'linewidth',consLineWidth)
% plot([0 consRadius],[0 0],ConstellationFaceColor,'linewidth',consLineWidth)

% Format Rink Axes
axis equal
axis ([-125 125 -70 70])

% Add Legend 
annotation('textbox',[.325 .15 .1 .1],'string','Red = Constellation, Blue = Robot');


% OUTPUT
    % xPosGlobal = X-position of the constellation as seen by the camera [m]
    % yPosGlobal = Y-position of the constellation as seen by the camera [m]
    % angOfRotGlobal = angle of rotation of the constellation from verital as
    % seen by the camera [rad]
    angleOfRotGlobal = angleOfRot;
    
end