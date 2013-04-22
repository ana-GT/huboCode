function [ q ] = ZMPPatternLat(startval, endval, DPointNum, SPointNum)

% ZMPPATTERNLAT(STARTVAL, ENDVAL, DPOINTNUM, SPOINTNUM)
% Return a vector for one footstep ZMP pattern, first double support, then
% single support
% double support ZMP pattern is cubic spline, implemented according to
% spline interpolation page on wikipedia

y1 = startval;
y2 = endval;

x1 = 1;
x2 = DPointNum;

a = startval - endval;
b = endval - startval;

x = 1:DPointNum;
t = (x-x1)/(x2-x1);

q = (1-t)*y1 + t*y2 + t.*(1-t).*(a*(1-t)+b*t);

for i = DPointNum+1:DPointNum + SPointNum
   q(i) = endval;
end


end




