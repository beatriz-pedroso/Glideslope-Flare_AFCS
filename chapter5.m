%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     extract the longitudal direction matrices.
%     These system matrices will be used to
%     compute the tranfer functions from the 
%     accelerometer to a negative elevator command.
%     Several positions of the accelerometer are tested.
%     
% Authors: Richard S. Russell, Ewoud Smeur (2021)
% 
% Edit: 
%       Pedroso, Beatriz 
%       Silva, Pedro
%       Yikilmaz, Cansu
%================================================
clear;

global fi_flag_Simulink

newline = sprintf('\n');

%% Trim aircraft to desired altitude and velocity
%%

%flight conditions to be used:
%15000ft, 500ft/s and 20000ft, 600ft/s

altitude = input('Enter the altitude for the simulation (ft)  :  ');
velocity = input('Enter the velocity for the simulation (ft/s):  ');

%Steady Wings-Level flight condition is used to trim the F-16 
FC_flag = 1;

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;           % AOA, degrees
rudder = -0.01;         % rudder angle, degrees
aileron = 0.01;         % aileron, degrees


%% Accelerometer Position from the cg, xa
%%
% different positions of the accelerometer
xa_vector = [0, 5, 5.9, 6, 7, 15]
xa = 0;
figure()
hold all
xlabel('Time [seconds]') 
ylabel('a_{n} [ft/seconds^2]')
for vector_position = 1 : 6
   xa = xa_vector(vector_position)


%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming High Fidelity Model:');
fi_flag_Simulink = 1;
[trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

%% Find the state space model for the hifi model at the desired alt and vel.
trim_state_lin = trim_state_hi; trim_thrust_lin = trim_thrust_hi; trim_control_lin = trim_control_hi;
operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

SS_hi = linearize('LIN_F16Block');

disp(' ');
%% Find trim for lofi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

%% Find the state space model for the lofi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

SS_lo = linearize('LIN_F16Block');


%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Direction
%%%%%%%%%%%%%%%%%%%%%%%

long_states = [3 5 7 8 11 13 14]; %[h theta Vt alpha q thrust elevator]
long_inputs = [2]; %elevator 
long_outputs = [19]; %an

SS_long_lo = ss(SS_lo.A(long_states,long_states), SS_lo.B(long_states,long_inputs), SS_lo.C(long_outputs,long_states), SS_lo.D(long_outputs,long_inputs));
SS_long_hi = ss(SS_hi.A(long_states,long_states), SS_hi.B(long_states,long_inputs), SS_hi.C(long_outputs,long_states), SS_hi.D(long_outputs,long_inputs));

SS_long_lo.StateName = SS_lo.StateName(long_states);
SS_long_hi.StateName = SS_hi.StateName(long_states);

SS_long_lo.InputName= SS_lo.InputName(long_inputs);
SS_long_hi.InputName= SS_hi.InputName(long_inputs);


%% STATE SPACE TO TRANSFER FUNCTION %%
%Matrix definitions
A = SS_long_lo.A
B = SS_long_lo.B
C = SS_long_lo.C
D = SS_long_lo.D
% transfer function computation
[b,a] = ss2tf(A,B,C,D)
sys = tf(SS_long_lo)
opt = stepDataOptions('StepAmplitude',-1);
[y,t] = step(sys,opt, 2)
%determination of the zeros of the transfer function
Z = zero(sys)

plot(t,y)

end

leg = legend('0','5', '5.9', '6', '7', '15');


