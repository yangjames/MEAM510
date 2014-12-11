function [const1_xy, const2_xy, const3_xy, const4_xy, error]=point_match(star1,star2,star3)

stars=[star1;star2;star3];



p1_gnd = [0 14.5];
p2_gnd = [-11.655 8.741];
p3_gnd = [10.563 2.483];
p4_gnd = [0 -14.5];

D12=sum((p1_gnd-p2_gnd).^2);
D13=sum((p1_gnd-p3_gnd).^2);
D14=sum((p1_gnd-p4_gnd).^2);
D23=sum((p2_gnd-p3_gnd).^2);
D24=sum((p2_gnd-p4_gnd).^2);
D34=sum((p3_gnd-p4_gnd).^2);


T123=[D12/D23,D13/D23,D12/D13];
T134=[D13/D14,D34/D14,D13/D34];
T124=[D12/D14,D24/D14,D12/D24];
T234=[D34/D24,D23/D24,D34/D23];


S12=sum((star1-star2).^2);
S13=sum((star1-star3).^2);
S23=sum((star2-star3).^2);

m=[1,2,S12;1,3,S13;2,3,S23];
m=sortrows(m,3);

mT=[m(1,3)/m(3,3),m(2,3)/m(3,3),m(1,3)/m(2,3)];

% find compare mT with pre defined triangles

mT_T123=sum(abs(mT-T123));
mT_T134=sum(abs(mT-T134));
mT_T124=sum(abs(mT-T124));
mT_T234=sum(abs(mT-T234));

likelihood_table=[123,mT_T123;134,mT_T134;124,mT_T124;234,mT_T234];
likelihood_table=sortrows(likelihood_table,2);

error=likelihood_table(1,2);

const1=[];
const2=[];
const3=[];
const4=[];


if likelihood_table(1,1)==123
    const2=mode([m(1,1:2),m(3,1:2)]);
    const1=mode([m(1,1:2),m(2,1:2)]);
    const3=mode([m(2,1:2),m(3,1:2)]);
    
elseif likelihood_table(1,1)==134
    const1=mode([m(1,1:2),m(3,1:2)]);
    const4=mode([m(2,1:2),m(3,1:2)]);
    const3=mode([m(1,1:2),m(2,1:2)]);
    
elseif likelihood_table(1,1)==124
    const1=mode([m(1,1:2),m(3,1:2)]);
    const4=mode([m(2,1:2),m(3,1:2)]);
    const2=mode([m(1,1:2),m(2,1:2)]);
    
elseif likelihood_table(1,1)==234
    const2=mode([m(2,1:2),m(3,1:2)]);
    const3=mode([m(1,1:2),m(2,1:2)]);
    const4=mode([m(1,1:2),m(3,1:2)]);
   
end

const1_xy=[];
const2_xy=[];
const3_xy=[];
const4_xy=[];

if ~isempty(const1)
const1_xy=stars(const1,:);
else
const1_xy=[1023,1023];
end

if ~isempty(const2)
const2_xy=stars(const2,:);
else
const2_xy=[1023,1023];
end

if ~isempty(const3)
const3_xy=stars(const3,:);
else
const3_xy=[1023,1023];
end

if ~isempty(const4)
const4_xy=stars(const4,:);
else
const4_xy=[1023,1023];
end




end