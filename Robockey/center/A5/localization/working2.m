%%
clear all
load('A.mat')


%%
s1=[rawStarData(:,1),rawStarData(:,5)];
s2=[rawStarData(:,2),rawStarData(:,6)];
s3=[rawStarData(:,3),rawStarData(:,7)];
s4=[rawStarData(:,4),rawStarData(:,8)];


%%
i=1;

figure(1)
clf
axis equal
grid on
hold on
set(gca,'xlim',[0 1023],'ylim',[0 1023]);

p1_plot = plot(s1(i,1),s1(i,2),'r*');
p2_plot = plot(s2(i,1),s2(i,2),'b*');
p3_plot = plot(s3(i,1),s3(i,2),'g*');
p4_plot = plot(s4(i,1),s4(i,2),'k*');

star1=s1(i,:);
star2=s2(i,:);
star3=s3(i,:);
star4=s4(i,:);


figure(2)
clf
axis equal
grid on
hold on
set(gca,'xlim',[0 1023],'ylim',[0 1023]);

%[const1_123 const2_123 const3_123 const4_123, error(1)]=point_match(star1,star2,star3);

% [best_const1, best_const2, best_const3, best_const4, error]=best_triangle(star1,star2,star3,star4);

[best_const1, best_const2, best_const3, best_const4, error]=likely_solution(star1,star2,star3,star4);

t1_p1_plot = plot(best_const1(1),best_const1(2),'r*');
t1_p2_plot = plot(best_const2(1),best_const2(2),'g*');
t1_p3_plot = plot(best_const3(1),best_const3(2),'b*');
t1_p4_plot = plot(best_const4(1),best_const4(2),'k*');
t1_c_plot = plot(0,0,'m*');

legend('Const1','Cosnt2','Const3','Const4','Center');


for i=1:size(rawStarData,1)

star1=s1(i,:);
star2=s2(i,:);
star3=s3(i,:);
star4=s4(i,:);

[best_const1, best_const2, best_const3, best_const4, error]=stupid_solution(star1,star2,star3,star4);
[orientation center height]=localize(best_const1, best_const2, best_const3, best_const4);

r = height*center*32/1024*pi/180;

set(p1_plot,'xdata',s1(i,1),'ydata',s1(i,2));
set(p2_plot,'xdata',s2(i,1),'ydata',s2(i,2));
set(p3_plot,'xdata',s3(i,1),'ydata',s3(i,2));
set(p4_plot,'xdata',s4(i,1),'ydata',s4(i,2));

%title(num2str(orientation))
set(t1_p1_plot,'xdata',best_const1(1,1),'ydata',best_const1(1,2));
set(t1_p2_plot,'xdata',best_const2(1,1),'ydata',best_const2(1,2));
set(t1_p3_plot,'xdata',best_const3(1,1),'ydata',best_const3(1,2));
set(t1_p4_plot,'xdata',best_const4(1,1),'ydata',best_const4(1,2));
set(t1_c_plot,'xdata',center(1)+1023/2,'ydata',center(2)+1023/2);
drawnow
%pause
end

%%









