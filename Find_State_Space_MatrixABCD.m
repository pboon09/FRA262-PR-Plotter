
global X_pk;
global P_pk;

[A,B,C,D,G] = MatrixGenerator();
X_pk = zeros(4,1);
P_pk = 0;

R = 0.206024725847665;
L = 0.012556185642912;
J = 0.004349433114583;
b = 5.856479716542130e-05;
ke = 0.062806525000000;
kt = ke;

function [A,B,C,D,G] = MatrixGenerator()
R = 0.206024725847665;
L = 0.012556185642912;
J = 0.004349433114583;
b = 5.856479716542130e-05;
ke = 0.062806525000000;
kt = ke;

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
