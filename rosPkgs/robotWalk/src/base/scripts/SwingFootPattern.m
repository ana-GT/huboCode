function [ x y z] = SwingFootPattern(xstart, xend, ystart, yend, N)

% SWINGFOOTPATTERN(XSTART, XEND, YSTART, YEND. N)
% Create trajectory for swing foot
xChange = xend - xstart;
yChange = yend - ystart;

t0 = 0;
t1 = pi;
dt = (t1 - t0) / (N - 1);
t = t0;

alpha = atan2(yChange, xChange);
c = cos(alpha);
s = sin(alpha);
l = 0;

a = sqrt(xChange * xChange + yChange * yChange) / 2;
b = a / pi;

x = zeros(1,N);
y = zeros(1,N);
z = zeros(1,N);

for i = 1: N
    x(i) = xstart + l*c;
    y(i) = ystart + l*s;
    z(i) = b * sin( (cos(t) + 1) * pi/2);
    
    t = t + dt;
    l = a * cos( (cos(t) + 1) * pi/2) + a;
    
end

end
