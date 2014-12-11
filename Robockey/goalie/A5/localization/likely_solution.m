function [best_const1, best_const2, best_const3, best_const4, error]=likely_solution(star1,star2,star3,star4)

const1=[];
const2=[];
const3=[];
const4=[];
error=[];

[const1(1,:), const2(1,:), const3(1,:), const4(1,:), error(1)]=point_match(star1,star2,star3);
[const1(2,:), const2(2,:), const3(2,:), const4(2,:), error(2)]=point_match(star1,star2,star4);
[const1(3,:), const2(3,:), const3(3,:), const4(3,:), error(3)]=point_match(star1,star3,star4);
[const1(4,:), const2(4,:), const3(4,:), const4(4,:), error(4)]=point_match(star2,star3,star4);

error_threshold=0.1;

% if mean(error)<error_threshold
%     best_const1=mode(const1);
%     best_const2=mode(const2);
%     best_const3=mode(const3);
%     best_const4=mode(const4);
%     
%     disp(error)
%     error=mean(error);
% 
%     
% else
   
[min_error, constellation_set]=min(error);

best_const1=const1(constellation_set,:);
best_const2=const2(constellation_set,:);
best_const3=const3(constellation_set,:);
best_const4=const4(constellation_set,:);
error=error(constellation_set);   
    
    
% end


end
