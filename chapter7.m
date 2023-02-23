%================================================
%     Matlab Script File used to linearize the non-linear F-16 model
%     to, then, construct the short period reduced model. The time
%     responsed of the pitch rate on a step input are plotted. Moreover,
%     the frequency domain requirements imposed by CAP and Gibson criteria
%     are calculated. Furthermore, a pole placement procedure and lead-lag
%     prefilter are implemented
%
% Authors: 
%       Pedroso, Beatriz 
%       Silva, Pedro
%       Yikilmaz, Cansu
%================================================
clear;

global fi_flag_Simulink

newline = sprintf('\n');

%% Trim aircraft to desired altitude and velocity
%%
%flight condition to be used: 20000ft, 600ft/s
altitude = 20000; %ft
velocity = 600; %ft/s

%Steady Wings-Level flight condition is used to trim the F-16 
FC_flag = 1;

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for lofi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

%% Find the state space model for the lofi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
operating_point = operpoint('LIN_F16Block_og'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

SS_lo = linearize('LIN_F16Block_og');

%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Direction
%%%%%%%%%%%%%%%%%%%%%%%

long_states = [7 8 5 11 13 14]; %[Vt alpha theta q thrust elevator]
long_inputs = [1 2]; %[thrust elevator]
long_outputs = [7 8 5 11];

SS_long_lo = ss(SS_lo.A(long_states,long_states), SS_lo.B(long_states,long_inputs), SS_lo.C(long_outputs,long_states), SS_lo.D(long_outputs,long_inputs));

SS_long_lo.StateName = SS_lo.StateName(long_states);

SS_long_lo.InputName= SS_lo.InputName(long_inputs);


%% Model without the actuator dynamics, taken from chapter 6

A_long = SS_long_lo.A;
B_long = SS_long_lo.B;

for l=1:4 
    for c=1:4 
        A_long_ac(l,c) = A_long(l,c);
    end
end

for l=1:4  
    B_long_ac(l,1) = A_long(l,6);
end

C_long_ac = eye(4);
D_long_ac = 0;

SS_long_ac=ss(A_long_ac, B_long_ac, C_long_ac, D_long_ac);
sys_long = tf(SS_long_ac);
damp(sys_long)


%% Task 1
% contruction of the short period reduced model
% the two states are the angle of attack (alpha) and the pitch rate (q)

for l=1:2 
    for c=1:2 
        A_long_ac_red(l,c) = A_long_ac(2*l,2*c);
    end
end

for l=1:2  
    B_long_ac_red(l,1) = B_long_ac(2*l,1);
end

C_long_ac_red = eye(2);
D_long_ac_red = 0;

SS_long_ac_red=ss(A_long_ac_red, B_long_ac_red, C_long_ac_red, D_long_ac_red);
sys_long_red = tf(SS_long_ac_red);



%% Task 2
% Plot of the time responses of the pitch rate on a step input for the 
%4 state and 2 state models, both without the actuator dynamics
% The pitch angle is in degrees/second (º/s)

% Short term response - 10 seconds of simulation
figure(1)
black = [0 0 0];
opt = stepDataOptions('StepAmplitude',-1);
step(sys_long(4)*180/pi, opt, 10) % 4 state model
hold on
step(sys_long_red(2)*180/pi, opt, 10) % 2 state model
xlabel('Time', 'FontSize', 15, 'Color', black);
ylabel('q (deg/second)', 'FontSize', 15,'Color', black);
title('');
ax = gca; 
ax.FontSize = 11.5; 
ax.XColor = black;
ax.YColor = black;
legend('4 state model','2 state model')
hold off


% Long term response - 1000 seconds of simulation
figure(2)
opt = stepDataOptions('StepAmplitude',-1);
step(sys_long(4)*180/pi, opt, 1000) % 4 state model
hold on
step(sys_long_red(2)*180/pi, opt, 1000) % 2 state model
xlabel('Time', 'FontSize', 15, 'Color', black);
ylabel('q (deg/second)', 'FontSize', 15,'Color', black);
title('');
ax = gca; 
ax.FontSize = 11.5; 
ax.XColor = black;
ax.YColor = black;
legend('4 state model','2 state model')
hold off


%% Task 3
% Computation of the frquency domain requirements imposed by 
% CAP and Gibson Criteria

% velocity units transformed from ft/s to m/s
V = velocity * 0.3048 %m/s

% equations given in the instruction assignment
wn_sp = 0.03*V % natural frequency
T_tt2 = 1/(0.75*wn_sp) % time constant
xi_sp = 0.5 % damping ratio


%% Task 4

s=tf('s');
% Find the poles of the system with the desired requirements (Task 3)
den_req = s^2 + 2*xi_sp*wn_sp*s + wn_sp*wn_sp;
den_req_zero = zero(den_req)

% Pole Placement
%K=[K_alpha (deg/rad); K_q(deg/(rad*sec))]
K = place(A_long_ac_red, B_long_ac_red, den_req_zero)

v_gust = 4.572; %vertical gust [m/s] 

% Formulas taken from Lecture 11, slide 16
alpha_induced = atan(v_gust/V); %rad
delta_c = K(1)*alpha_induced; %deg


%% Task 5
% Time constante is modified by means of a lead-lag prefilter

% Feedback closed loop system (A-BK)
sys_closed_loop = feedback(sys_long_red, K)
% Transfer function of the feedback closed loop system
[num, den]= tfdata(sys_closed_loop(2))

% Pre-filter system: (kq*(1 + T_tt2*s)/(s^2 + 2*xi*wn_sp*s + wn_sp^2))
prefilter_den = cell2mat(num); % denominator of the prefilter system
kq = prefilter_den(3);
prefilter_num = kq.*[0 T_tt2 1]; % numerator of the prefilter system
% transfer function of the prefilter system
filtered_sys = tf(prefilter_num, cell2mat(den)) 

%% Task 6

% Feed forward gain
K_ff = 1/(dcgain(filtered_sys))
% Final system, in which the steady state pitch rate tracks the input
final_sys = series(filtered_sys, K_ff);
% Numerator and Denominator in matrix form
% These are used in the simulink model "chapter7_pitch_rate.slx" - task 7
numerator = cell2mat(final_sys.Numerator);
denominator = cell2mat(final_sys.Denominator);


%% Task 7
% CAP and Gibson requirements verification
% Formulas from the assignment instructions

%CAP
gD = 9.80665; %[m/s^2]
CAP = (wn_sp^2)/((V/gD)*(1/T_tt2));

%Gibson
S = stepinfo(final_sys);
ratio_qm_qs = 1 + (S.Overshoot / 100)
DB_qs = T_tt2 - (2*xi_sp/(wn_sp))

%Simulink model used to plot the pitch rate and pitch angle time responses
%to a step and a ramp inputs, respectively

%constant used to switch between two conditions in the simulink model
pitch_r_switch = 1;
sim("chapter7_pitch_rate.slx")

%pitch rate step response
figure(4)
hold on
plot(ans.input.time, ans.input.data,'LineWidth',1.3)
plot(ans.tf.time, ans.tf.data,'LineWidth',1.3)
xlabel('Time (seconds)', 'FontSize', 15);
ylabel('Pitch rate (deg/seconds)', 'FontSize', 15);
title('');
ax = gca;
ax.FontSize = 11.5;
legend('step input','pitch rate response')
grid on
%set(gcf,[0 0 0])
hold off

% %simulink switch for ramp input
% pitch_r_switch = 2;
% sim("chapter7_pitch_rate.slx")

%pitch angle ramp response
figure(5)
hold on
plot(ans.input2.time, ans.input2.data,'LineWidth',1.3)
plot(ans.tf2.time, ans.tf2.data,'LineWidth',1.3)
xlabel('Time (seconds)', 'FontSize', 15);
ylabel('Pitch angle (deg)', 'FontSize', 15);
title('');
ax = gca;
ax.FontSize = 11.5;
legend('step input','pitch angle response')
grid on
hold off

% Longitudinal Poles pzmap
figure(10); 
pzmap(SS_long_ac, 'b');
% title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLongitudal Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
% title(title_string);
sgrid;

%% Task 8
%Gain scheduling for velocities in the range [600,900](ft/s)
%The contents of each K are K = [K_alpha   k_q]

% First condition, for V=600ft/s
% K_1 = -190.9868  -26.5239
% Second condition, for V=900ft/s 
% K_2 = -189.7413  -17.6758

vel = 800; %chosen velocity, to compute the respective gains
K_alpha = [-190.9868  -189.7413] %K_alpha = [K_1_alpha  K_2_alpha]
K_q = [-26.5239 -17.6758 ] %K_q = [K_1_q  K_2_q]
V_int = [600 900] %interpolation between v=600ft/s and v=900ft/s
v_alpha = interp1(V_int,K_alpha,vel)
v_q = interp1(V_int,K_q,vel)

%% gains for V = 700 and for V = 800
K_700 = [-190.5716  -23.5745]
K_800 = [-190.1565  -20.6252]


%% The whole process in the previous tasks (4-7) is repeated for other values
%pole placement
K = K_800 % gain for v=800ft/s
v_gust = 4.572; %m/s 
alpha_induced = atan(v_gust/vel); %rad
delta_c = K(1)*alpha_induced; %deg

%feedback closed loop system (A-BK)
sys_closed_loop = feedback(sys_long_red, K)
% Transfer function of the feedback closed loop system
[num, den]= tfdata(sys_closed_loop(2))

% Pre-filter system: (kq*(1 + T_tt2*s)/(s^2 + 2*xi*wn_sp*s + wn_sp^2))
prefilter_den = cell2mat(num);  % denominator of the prefilter system
kq = prefilter_den(3);
prefilter_num = kq.*[0 T_tt2 1];  % numerator of the prefilter system
filtered_sys = tf(prefilter_num, cell2mat(den))

K_ff = 1/(dcgain(filtered_sys)) %feed-forward gain
% system in which the steady state pitch rate tracks the input
final_sys = series(filtered_sys, K_ff);
numerator=cell2mat(final_sys.Numerator);
denominator=cell2mat(final_sys.Denominator);

%CAP and Gibson requirements verification
%CAP
gD = 9.80665; %[m/s^2]
CAP = (wn_sp^2)/((V/gD)*(1/T_tt2));

%Gibson
S = stepinfo(final_sys);
ratio_qm_qs = 1 + (S.Overshoot / 100)
DB_qs = T_tt2 - (2*xi_sp/(wn_sp))

%Simulink model used to plot the pitch rate and pitch angle time responses
%to a step and a ramp inputs, respectively

%constant used to switch between two conditions in the simulink model
pitch_r_switch = 1;
sim("chapter7_pitch_rate.slx")

% %pitch rate step response
% figure(6)
% hold on
% plot(ans.input.time, ans.input.signals.values)
% plot(ans.pitch_r_response.time, ans.pitch_r_response.signals.values)
% xlabel('Time (seconds)', 'FontSize', 15);
% ylabel('Pitch rate (deg/seconds)', 'FontSize', 15);
% title('');
% ax = gca;
% ax.FontSize = 11.5;
% legend('step input','pitch rate response')
% hold off
% 
% %simulink switch
% pitch_r_switch = 2;
% sim("chapter7_pitch_rate.slx")
% 
% %pitch angle ramp response
% figure(7)
% hold on
% plot(ans.input.time, ans.input.signals.values)
% plot(ans.pitch_r_response.time, ans.pitch_r_response.signals.values)
% xlabel('Time (seconds)', 'FontSize', 15);
% ylabel('Pitch angle (deg)', 'FontSize', 15);
% title('');
% ax = gca;
% ax.FontSize = 11.5;
% legend('ramp input','pitch angle response')
% hold off

%Longitudinal Poles
figure(11); 
pzmap(SS_long_ac, 'b');
title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLongitudal Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
title(title_string);
sgrid;

%% Results of the Gibson Criteria for v=700ft/s and v=800ft/s

%%%%%%% K_700 %%%%%%%
% ratio_qm_qs = 1.4580
% DB_qs =0.0608

%%%%%%% K_800 %%%%%%%
% ratio_qm_qs = 1.5084
% DB_qs =0.0608

