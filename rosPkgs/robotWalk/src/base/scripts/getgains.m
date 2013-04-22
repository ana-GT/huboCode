function [G1, G2, G3] = getgains(T, Zc, Qe, Qu, N)

% GETGAINS(T, ZC, QE, QU, N)
% Compute gains for ZMP preview controller
% (TODO) Based on the papers:

g = 9.8;


A = [ 1 T T^2/2; 0 1 T; 0 0 1];
B = [ T^3/6; T^2/2; T];
C = [ 1 0 -Zc/g];

BB = [C*B; B];
II = [1; 0; 0; 0];
FF= [C*A; A];
QQ = [Qe 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
AA = [II FF];

[X, L, G] = dare2(AA, BB, QQ, Qu);

G1 = G(1);
G2 = G(2:4);

W = (Qu + BB'*X*BB)^(-1) * BB';
TMP = (AA - BB*W*X*AA)';

for i = 1:N 
    G3(i) = -W*( (TMP)^(i-1) )*X*II;
end

end
