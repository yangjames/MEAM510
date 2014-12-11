function [best_const1, best_const2, best_const3, best_const4, error]=stupid_solution(star1,star2,star3,star4)

average=(star1+star2+star3+star4)/4;

differences=[star1-average;star2-average;star3-average;star4-average];

differences=differences.^2;
abs_differences=differences(:,1)+differences(:,2);

if range(abs_differences)<3000
    
    abs_differences(:,2)=1:4;
    abs_differences(:,3)=[star1(1);star2(1); star3(1); star4(1)];
    abs_differences(:,4)=[star1(2);star2(2); star3(2); star4(2)];
    abs_sorted=sortrows(abs_differences,1);
    
    
    
    
    best_const3=[abs_sorted(1,3),abs_sorted(1,4)];
    best_const1=[abs_sorted(2,3),abs_sorted(2,4)];
    best_const2=[abs_sorted(3,3),abs_sorted(3,4)];
    best_const4=[abs_sorted(4,3),abs_sorted(4,4)];
    
    error=1;
else
    [best_const1, best_const2, best_const3, best_const4, error]=likely_solution(star1,star2,star3,star4);
    
end



end