function [ ZMPX, ZMPY ] = ZMP5Steps()

% ZMP5Steps()
% Create ZMP trajectory for steps

    X = 0.1;
%    Y = 0.089;
    Y = 0.141;

    SLOPEN = 300;
    LEVELN = 2100;
    
    WAITN = 6000;
    
    Xwait1 = zeros(1, WAITN);
    Xstep1 = ZMPPatternLat(0, 0, SLOPEN, LEVELN);
    Xstep2 = ZMPPatternLat(0, X, SLOPEN, LEVELN);
    Xstep3 = ZMPPatternLat(X, 2*X, SLOPEN, LEVELN);
    Xstep4 = ZMPPatternLat(2*X, 3*X, SLOPEN, LEVELN);
    Xstep5 = ZMPPatternLat(3*X, 4*X, SLOPEN, LEVELN);
    XstepEnd = ZMPPatternLat(4*X, 4*X, SLOPEN, LEVELN);
    
    Xwait2(1:WAITN) = 4*X;
    
    ZMPX = [Xwait1 Xstep1 Xstep2 Xstep3 Xstep4 Xstep5 XstepEnd Xwait2];
    
    
    Ywait1 = zeros(1, WAITN);
    Ystep1 = ZMPPatternLat(0, Y, SLOPEN, LEVELN);
    Ystep2 = ZMPPatternLat(Y, -Y, SLOPEN, LEVELN);
    Ystep3 = ZMPPatternLat(-Y, Y, SLOPEN, LEVELN);
    Ystep4 = ZMPPatternLat(Y, -Y, SLOPEN, LEVELN);
    Ystep5 = ZMPPatternLat(-Y, Y, SLOPEN, LEVELN);
    YstepEnd = ZMPPatternLat(Y, 0, SLOPEN, LEVELN);
    
    Ywait2 = zeros(1, WAITN);
    ZMPY = [Ywait1 Ystep1 Ystep2 Ystep3 Ystep4 Ystep5 YstepEnd Ywait2];
    
end
