clc;
clear;
close all;

%% ================= DC MOTOR PARAMETERS =================
R  = 1.17;          % Ohms
L  = 0.58;          % Henry
kt = 0.011;         % Torque constant
kb = 0.011;         % Back EMF constant
J  = 1.34e-6;       % kg.m^2
b  = 1.62e-6;       % N.m.s

%% ================= MOTOR TRANSFER FUNCTION =================
num = kt;
den = [J*L  (J*R + b*L)  (b*R + kt*kb)];

G = tf(num, den);

disp('DC Motor Transfer Function:')
G

%% ================= STATE SPACE MODEL =================
[A, B, C, D] = tf2ss(num, den);

%% ================= THEORETICAL DESIGN VALUES =================
zeta = 0.3;        % Desired damping ratio
wn   = 20;         % Desired natural frequency

desired_poles = [-zeta*wn + 1i*wn*sqrt(1-zeta^2), ...
                 -zeta*wn - 1i*wn*sqrt(1-zeta^2)];

%% ================= STATE FEEDBACK GAIN =================
K = place(A, B, desired_poles);

disp('State Feedback Gain K:')
disp(K)

%% ================= CLOSED LOOP SYSTEM =================
Acl = A - B*K;

disp('Closed-loop poles:')
p = eig(Acl);
disp(p)

%% ================= NATURAL FREQUENCY & DAMPING RATIO =================
wn_calc   = abs(p(1));
zeta_calc = -real(p(1)) / wn_calc;

disp('Calculated Natural Frequency (rad/s):')
disp(wn_calc)

disp('Calculated Damping Ratio:')
disp(zeta_calc)

%% ================= STEP RESPONSE (TEXT OUTPUT ONLY) =================
sys_cl = ss(Acl, B, C, D);

t = 0:0.0005:0.3;
u = ones(size(t));

y = lsim(sys_cl, u, t);

disp('Final value of step response:')
disp(y(end))
