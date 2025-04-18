
global X_pk;
global P_pk;

[A,B,C,D,G] = MatrixGenerator();
X_pk = zeros(4,1);
P_pk = 0;

% R = 1.15;
% L = 0.157854;
% J = 0.006554324152089;
% b = 0.115425638432433;
% ke = 0.575360445895341;
% kt = ke * 0.814122903634464;

R = 1.00;
L = 0.149095;
J = 0.000763452454589;
b = 0.017519651130878;
ke = 0.200510144729990;
kt = ke * 0.867088244587842;

function [A,B,C,D,G] = MatrixGenerator()
R = 1.00;
L = 0.149095;
J = 0.000763452454589;
b = 0.017519651130878;
ke = 0.200510144729990;
kt = ke * 0.867088244587842;

    dt = 1.0/1000.0;

    %State Transition Matrix
    Ac = [0   1    0    0;
         0 -b/J -1/J kt/J;
         0   0    0    0;
         0 -ke/L  0  -R/L];
    %Input Matrix
    Bc = [0;
         0;
         0;
         1/L]; 
    %Process noise
    Gc = [0;
         1;
         0;
         0];
    %Output matrix
    Cc = [1 0 0 0];
    %Feed through matrix
    Dc = 0;
    
    % Create state space model
    sys = ss(Ac,Bc,Cc,Dc);
    % Convert to discrete, where dt is your discrete time-step (in seconds)
    d_sys = c2d(sys,dt);
    
    %Discrete Matrix
    A = d_sys.A;
    B = d_sys.B;
    C = d_sys.C;
    D = d_sys.D;
    G = Gc;
end


function [Es_Velocity ,Es_Current,Es_Position] = SteadyStateKalmanFilter(Vin,Position)
    global X_pk
    global P_pk
    Q = 1;
    R = 1;
    
    [A,B,C,D,G] = MatrixGenerator();
    %Prediction of the state Covariences error
    X_k = (A * X_pk) + (B * Vin);
    P_k = (A * P_pk * A.') + G*Q*G';
    
    %Computation of the Kalman Gain
    K = (P_k*C.')/(C*P_k*C.' + R);

    %Computation of estimative
    X_k = X_k + K*(Position - C*X_k);
    Es_Position = X_k(1);
    Es_Velocity = X_k(2);
    Es_Current = X_k(4);
    %Computation Covarrience error
    P_k = (eye(4)-K*C)*P_k; 

    X_pk = X_k;
    P_pk = P_k;
end
