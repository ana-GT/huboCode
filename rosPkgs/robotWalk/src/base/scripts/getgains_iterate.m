function [G1, G2, G3] = getgains(T, Zc, Qe, Qu, N)

% GETGAINS(T, ZC, QE, QU, N)
% Compute gains for ZMP preview controller iteratively, 
% instead of just calling dare() in Matlab
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

X = eye(4,4);
Xnew = eye(4,4);

error = 10^(-10);

for i = 1:100000
    Xnew = AA'*X*AA - AA'*X*BB*(1.0/(Qu+BB'*X*BB))*BB'*X'*AA + QQ;
    curr_error = norm(Xnew-X) / norm(Xnew)
    X = Xnew;
    if curr_error < error
        'Converges after ' 
        i 
        'trials'
        curr_error
        break;
    end
end

W = (Qu + BB'*X*BB)^(-1) * BB';
G = W*X*AA;
G1 = G(1);
G2 = G(2:4);


TMP = (AA - BB*W*X*AA)';

for i = 1:N 
    G3(i) = -W*( (TMP)^(i-1) )*X*II;
end

end
