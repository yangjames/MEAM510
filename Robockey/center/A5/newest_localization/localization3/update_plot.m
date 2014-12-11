function [xPosRobot,yPosRobot]=update_plot(plot1,plot2,plot3,plot4,plot5,plot6, xPos, yPos, angleOfRot)


robotRadius = 20;


% Scale to fit on rink for now
xPos = xPos/2;
yPos = yPos/2;

% ROBOT THINGS
% Calculate Robot Position and Rotation
angleOfRot2 = -angleOfRot;
RotCtoG = [-cos(angleOfRot2) sin(angleOfRot2);
          sin(angleOfRot2) cos(angleOfRot2)]; % rotate from frame C to G
posRobot = RotCtoG*[-xPos; -yPos];

xPosRobot = posRobot(1); % negative because looking down instead of up
yPosRobot = posRobot(2);
angleOfRotGlobal = angleOfRot2;

% Plot Robot Position and Rotation
robotPlotXGlobal = (xPosRobot-robotRadius):.01:(xPosRobot+robotRadius);
robotPlotYtopGlobal = yPosRobot+sqrt(robotRadius^2-(robotPlotXGlobal-xPosRobot).^2);
robotPlotYbottomGlobal = yPosRobot-sqrt(robotRadius^2-(robotPlotXGlobal-xPosRobot).^2);


set(plot1,'xdata',robotPlotXGlobal,'ydata',real(robotPlotYtopGlobal));
set(plot2,'xdata',robotPlotXGlobal,'ydata',real(robotPlotYbottomGlobal));
set(plot3,'xdata',[xPosRobot xPosRobot-robotRadius*sin(angleOfRot2)],'ydata',[yPosRobot yPosRobot+robotRadius*cos(angleOfRot2)]);



% robotColor2 = ':b';
% plot(robotPlotXGlobal,robotPlotYtopGlobal,robotColor2,'linewidth',robotLineWidth)
% plot(robotPlotXGlobal,robotPlotYbottomGlobal,robotColor2,'linewidth',robotLineWidth)
% plot([xPosRobot xPosRobot-robotRadius*sin(angleOfRot)],[yPosRobot yPosRobot+robotRadius*cos(angleOfRot)],robotColor2,'linewidth',robotLineWidth)
% % plot([XposRobot XposRobot+robotRadius*cos(angleOfRot)],[YposRobot YposRobot+robotRadius*sin(angleOfRot)],robotColor,'linewidth',robotLineWidth)
% 


% plotting constellation
consRadius = 30;
consColor = ':r';
consLineWidth = 2;

consPlotX = (xPos-consRadius):.01:(xPos+consRadius);
consPlotYtop = yPos + sqrt(consRadius^2-(consPlotX-xPos).^2);
consPlotYbottom = yPos - sqrt(consRadius^2-(consPlotX-xPos).^2);

set(plot4,'xdata',consPlotX,'ydata',real(consPlotYtop));
set(plot5,'xdata',consPlotX,'ydata',real(consPlotYbottom));
set(plot6,'xdata',[xPos xPos-consRadius*sin(angleOfRot)],'ydata',[yPos yPos+consRadius*cos(angleOfRot)]);



% plot4=plot(consPlotX,consPlotYtop,consColor,'linewidth',consLineWidth)
% plot5=plot(consPlotX,consPlotYbottom,consColor,'linewidth',consLineWidth)
% plot6=plot([xPos xPos-consRadius*sin(angleOfRot)],[yPos yPos+consRadius*cos(angleOfRot)],consColor,'linewidth',consLineWidth)







drawnow

end