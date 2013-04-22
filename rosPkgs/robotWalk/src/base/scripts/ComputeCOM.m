function [ x, yexp ] = ComputeCOM(T, Zc, N, yref, G1, G2, G3) 

% COMPUTECOM(T, ZC, N, YEXP, G1, G2, G3)
% Using ZMP Preview Controller compute center of mass position, velocity and acceleration
% (TODO) Based on the papers:

g = 9.8;

A = [ 1 T T^2/2; 0 1 T; 0 0 1];
B = [ T^3/6; T^2/2; T];
C = [ 1 0 -Zc/g];

e_total = 0;
x{1} = [ 0; 0; 0];
yexp(1) = 0;

% (TODO) preallocating for speed
for t = 1 : (length(yref) - N)
    e_total = e_total + (yexp(t) - yref(t));  
    u(t) = -G1*e_total -G2*x{t} - G3*( (yref(t+1:t+N))' );
    x{t+1} = A*x{t} + B*u(t);
    yexp(t+1) = C*x{t+1};
end


