clear
clc

% Compare desired ZMP trajectory and expected ZMP trajectory


AHEAD = 6000;
%Height = 0.921;
Height = 0.8438;
T = 0.00033;

[zmpx, zmpy] = ZMP5Steps();

[G1, G2, G3] = getgains(T, Height, 10^7, 10, AHEAD);

[ x, zmpexpx] = ComputeCOM(T, Height, AHEAD, zmpx, G1, G2, G3);
[ y, zmpexpy] = ComputeCOM(T, Height, AHEAD, zmpy, G1, G2, G3);


%for i = 1: length(x)
for i = 1:length(x)
    xpos(i) = x{i}(1);
end

for i = 1: length(y)
    ypos(i) = y{i}(1);
end

fpx = fopen('comx', 'w');
fpy = fopen('comy', 'w');

for i = 1: length(x) 
    fprintf(fpx, '%d\n', xpos(i));
    fprintf(fpy, '%d\n', ypos(i));
end  

fclose(fpx);
fclose(fpy);

subplot(2,1,1), plot(1:length(zmpx), zmpx,'*g', 1:length(xpos), xpos, '*b', 1:length(zmpexpx), zmpexpx, '*r');
subplot(2,1,2), plot(1:length(zmpy), zmpy,'*g', 1:length(ypos), ypos, '*b', 1:length(zmpexpy), zmpexpy, '*r');


ylim([-0.15, 0.15]);
