function [ Leftx Lefty Leftz Rightx Righty Rightz ] = Swing5Steps()

% Swing5Step()
% Create trajectory for both foot


NSLOPE = 300;
NLEVEL = 2100;
NPoints = 2400;

Nwait1 = 6000;
Nwait2 = 6000;

X = 0.1;
Y =  0.141;



% wait 1
wait1Lx(1:Nwait1) = 0;
wait1Ly(1:Nwait1) = Y;
wait1Lz(1:Nwait1) = 0;

wait1Rx(1:Nwait1) = 0;
wait1Ry(1:Nwait1) = -Y;
wait1Rz(1:Nwait1) = 0;


% still 1
% Left
still1Lx(1:NSLOPE) = 0;
still1Ly(1:NSLOPE) = Y;
still1Lz(1:NSLOPE) = 0;

% Right
still1Rx(1:NSLOPE) = 0;
still1Ry(1:NSLOPE) = -Y;
still1Rz(1:NSLOPE) = 0;

% step 1
% Left
step1Lx(1:NLEVEL) = 0;
step1Ly(1:NLEVEL) = Y;
step1Lz(1:NLEVEL) = 0;
% Right
[ step1Rx step1Ry step1Rz ] = SwingFootPattern(0, X, -Y, -Y, NLEVEL);


% still 2
% Left
still2Lx(1:NSLOPE) = 0;
still2Ly(1:NSLOPE) = Y;
still2Lz(1:NSLOPE) = 0;

% Right
still2Rx(1:NSLOPE) = X;
still2Ry(1:NSLOPE) = -Y;
still2Rz(1:NSLOPE) = 0;

% step 2
% Left
[ step2Lx step2Ly step2Lz ] = SwingFootPattern(0, 2*X, Y, Y, NLEVEL);
% Right
step2Rx(1:NLEVEL) = X;
step2Ry(1:NLEVEL) = -Y;
step2Rz(1:NLEVEL) = 0;


% still 3
% Left
still3Lx(1:NSLOPE) = 2*X;
still3Ly(1:NSLOPE) = Y;
still3Lz(1:NSLOPE) = 0;

% Right
still3Rx(1:NSLOPE) = X;
still3Ry(1:NSLOPE) = -Y;
still3Rz(1:NSLOPE) = 0;

% step 3
% Left
step3Lx(1:NLEVEL) = 2*X;
step3Ly(1:NLEVEL) = Y;
step3Lz(1:NLEVEL) = 0;
% Right
[ step3Rx step3Ry step3Rz ] = SwingFootPattern(X, 3*X, -Y, -Y, NLEVEL);



% still 4
% Left
still4Lx(1:NSLOPE) = 2*X;
still4Ly(1:NSLOPE) = Y;
still4Lz(1:NSLOPE) = 0;

% Right
still4Rx(1:NSLOPE) = 3*X;
still4Ry(1:NSLOPE) = -Y;
still4Rz(1:NSLOPE) = 0;

% step 4
% Left
[ step4Lx step4Ly step4Lz ] = SwingFootPattern(2*X, 4*X, Y, Y, NLEVEL);
% Right
step4Rx(1:NLEVEL) = 3*X;
step4Ry(1:NLEVEL) = -Y;
step4Rz(1:NLEVEL) = 0;





% still 5
% Left
still5Lx(1:NSLOPE) = 4*X;
still5Ly(1:NSLOPE) = Y;
still5Lz(1:NSLOPE) = 0;

% Right
still5Rx(1:NSLOPE) = 3*X;
still5Ry(1:NSLOPE) = -Y;
still5Rz(1:NSLOPE) = 0;


% step 5
% Left
step5Lx(1:NLEVEL) = 4*X;
step5Ly(1:NLEVEL) = Y;
step5Lz(1:NLEVEL) = 0;
% Right
[ step5Rx step5Ry step5Rz ] = SwingFootPattern(3*X, 4*X, -Y, -Y, NLEVEL);




% stepeEnd + wait 2
wait2Lx(1:NPoints+Nwait2 ) = 4*X;
wait2Ly(1:NPoints+Nwait2 ) = Y;
wait2Lz(1:NPoints+Nwait2 ) = 0;

wait2Rx(1:NPoints+Nwait2 ) = 4*X;
wait2Ry(1:NPoints+Nwait2 ) = -Y;
wait2Rz(1:NPoints+Nwait2 ) = 0;


% Trajectory
Leftx = [wait1Lx still1Lx step1Lx still2Lx step2Lx still3Lx step3Lx still4Lx step4Lx still5Lx step5Lx wait2Lx];
Lefty = [wait1Ly still1Ly step1Ly still2Ly step2Ly still3Ly step3Ly still4Ly step4Ly still5Ly step5Ly wait2Ly];
Leftz = [wait1Lz still1Lz step1Lz still2Lz step2Lz still3Lz step3Lz still4Lz step4Lz still5Lz step5Lz wait2Lz];

Rightx = [wait1Rx still1Rx step1Rx still2Rx step2Rx still3Rx step3Rx still4Rx step4Rx still5Rx step5Rx wait2Rx];
Righty = [wait1Ry still1Ry step1Ry still2Ry step2Ry still3Ry step3Ry still4Ry step4Ry still5Ry step5Ry wait2Ry];
Rightz = [wait1Rz still1Rz step1Rz still2Rz step2Rz still3Rz step3Rz still4Rz step4Rz still5Rz step5Rz wait2Rz];

% write to file
fplz = fopen('leftz', 'w');
fprz = fopen('rightz', 'w');

fplx = fopen('leftx', 'w');
fprx = fopen('rightx', 'w');

for i = 1:length(Leftx)
    fprintf(fplz, '%d\n', Leftz(i));
    fprintf(fprz, '%d\n', Rightz(i));
    fprintf(fplx, '%d\n', Leftx(i));
    fprintf(fprx, '%d\n', Rightx(i));

end
fclose(fplx);
fclose(fprx);
fclose(fplz);
fclose(fprz);
end
