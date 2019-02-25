%as Camille
w1=10^-7; %jerk -7
w2=10^0; %com vel ref
w3=10^-1; %zmp wth zeta mean close to step
w4=10^-2; %com height

w5=10^-1*0; %zmp acceleration aka COM acceleration
w6=10^-2*0; %zmp vel between segment

OptimCostWeight=[w1 w2 w3 w4 w5 w6];
clear w1 w2 w3 w4 w5 w6