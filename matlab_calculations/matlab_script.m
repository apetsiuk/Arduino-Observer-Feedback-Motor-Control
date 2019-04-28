%% *Observer Feedback Control System with Arduino*
%% Develop the Controller
% The implementation of observer feedback control with the Arduino. The analog 
% implementation of an observer feedback is fairly complex. Therefore, these systems 
% are typically implemented digitally. 
% 
% 
% Considering the following system, we will design an observer feedback system 
% for controlling the position of the system. Assume the closed loop state feedback 
% poles at:
% 
% -10+/-j20 and the observer poles at -40+/-j40.
% 
% 
% Our goal is to find the state-space representation of the controllerwhich 
% we will implement with Arduino microcontroller.
% 

s = tf('s');

Plant = 6*12.6/(s^2+29*s)
state_poles = [-10+1i*20 -10-1i*20]
obs_poles = [-40+1i*40 -40-1i*40]

% SS Model of the Plant
[A,B,C,D] = tf2ss([0 0 75.6],[1 29 0])

% Control Gains
K = place(A,B,state_poles)
eig(A-B*K)
% Feedforward Gain
g = -1/(C*inv(A-B*K)*B)


%% Develop the Observer

syms s g1 g2;
G = [g1; g2];

eqn = collect(det(s*eye(2)-A+G*C))

desired_char = simplify((s-obs_poles(1))*(s-obs_poles(2)))
g2 = double(solve(378*g2/5+29==80,g2))
g1 = double(solve(378*g1/5+10962*g2/5==3200,g1))

G = [g1; g2]
F = A-G*C
H = B
% sanity check
eig(A-G*C)

G = place(A',C',obs_poles)'

damp(-10+20j)

% dampig ratio for the poles = 0.447
percent_overshoot = 100*exp(-pi*.447/sqrt(1-.447^2))

% step response
step(ss(A-B*K,B,C,D))
stepinfo(ss(A-B*K,B,C,D))


%% For the Arduino controller part

Ac = A-G*C

Bc = [B G]