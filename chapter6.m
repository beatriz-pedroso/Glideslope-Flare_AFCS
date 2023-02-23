%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     extract the longitudal and lateral 
%     direction matrices for the lofi model.
%     These matrices are reduced into a fourth
%     order dynamics. The characteristics of the
%     eigenmotions are computed and its time responses
%     are plotted.
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

long_states = [5 7 8 11 13 14]; %[theta Vt alpha q thrust elevator]
long_inputs = [1 2]; %[thrust elevator]
long_outputs = [5 7 8 11];

SS_long_lo = ss(SS_lo.A(long_states,long_states), SS_lo.B(long_states,long_inputs), SS_lo.C(long_outputs,long_states), SS_lo.D(long_outputs,long_inputs));

SS_long_lo.StateName = SS_lo.StateName(long_states);

SS_long_lo.InputName= SS_lo.InputName(long_inputs);

%%%%%%%%%%%%%%%%%%%%
%% Lateral Direction
%%%%%%%%%%%%%%%%%%%%

lat_states = [4 9 10 12 15 16]; %[phi beta p r aileron rudder]
lat_inputs = [3 4]; %[aileron rudder]
lat_outputs = [4 9 10 12];

SS_lat_lo = ss(SS_lo.A(lat_states,lat_states), SS_lo.B(lat_states,lat_inputs), SS_lo.C(lat_outputs,lat_states), SS_lo.D(lat_outputs,lat_inputs));

SS_lat_lo.StateName = SS_lo.StateName(lat_states);

SS_lat_lo.InputName= SS_lo.InputName(lat_inputs);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Definition of the reduced model

%% Longitudinal
A_long = SS_long_lo.A;
B_long = SS_long_lo.B;

for l=1:4 
    for c=1:4 
        A_long_ac(l,c) = A_long(l,c);
    end
end

for l=1:4  
    for c=1:2
        B_long_ac(l,c) = A_long(l,c+4);
    end
end

C_long_ac = eye(4);
D_long_ac = 0;

%state space of the reduced model
SS_long_ac=ss(A_long_ac, B_long_ac, C_long_ac, D_long_ac);
%transfer function of the system
sys_long = tf(SS_long_ac);
%damp(sys_long)

%natural frequency, damping ratio, and poles
[wn_long,zeta_long,p_long] = damp(sys_long)
%time constant
tau_long = 1./(wn_long.*zeta_long)
%period
T_long = 2*pi./imag(p_long)
%T1/2
T_half_long = log(1/2)./real(p_long)

s=tf('s');

%Phugoid (ph)
G_ph = 1/((s-p_long(1))*(s-p_long(2)));
figure(1)
hold on
step(G_ph)
black = [0 0 0];
xlabel('Time', 'FontSize', 15, 'Color', black);
ylabel('Amplitude', 'FontSize', 15,'Color', black);
title('');
ax = gca; 
ax.FontSize = 11.5; 
ax.XColor = black;
ax.YColor = black;
hold off

%Short Period (sp)
G_sp = 1/((s-p_long(5))*(s-p_long(6)));
figure(2)
hold on
step(G_sp)
xlabel('Time', 'FontSize', 15, 'Color', black);
ylabel('Amplitude', 'FontSize', 15, 'Color', black);
title('');
ax = gca; 
ax.FontSize = 11.5; 
ax.XColor = black;
ax.YColor = black;
hold off


%% Lateral

A_lat = SS_lat_lo.A;
B_lat = SS_lat_lo.B;

for l=1:4 
    for c=1:4 
        A_lat_ac(l,c) = A_lat(l,c);
    end
end

for l=1:4  
    for c=1:2
        B_lat_ac(l,c) = A_lat(l,c+4);
    end
end

C_lat_ac = eye(4);
D_lat_ac = 0;

%state space of the reduced model
SS_lat_ac=ss(A_lat_ac, B_lat_ac, C_lat_ac, D_lat_ac);
%transfer function of the system
sys_lat = tf(SS_lat_ac);
%damp(sys_lat)
%natural frequency, damping ratio, and poles
[wn_lat,zeta_lat,p_lat] = damp(sys_lat)
%time constant
tau_lat = 1./(wn_lat.*zeta_lat)
%period
T_lat = 2*pi./imag(p_lat)
%T1/2
T_half_lat = log(2)./real(p_lat)

%Spiral
G_spiral = 1/(s-p_lat(1));
figure(3)
hold on
step(G_spiral)
xlabel('Time', 'FontSize', 15, 'Color', black);
ylabel('Amplitude', 'FontSize', 15, 'Color', black);
title('');
ax = gca; 
ax.FontSize = 11.5; 
ax.XColor = black;
ax.YColor = black;
hold off

%Roll
G_roll = 1/(s-p_lat(3));
hold on
figure(4)
step(G_roll)
xlabel('Time', 'FontSize', 15, 'Color', black);
ylabel('Amplitude', 'FontSize', 15, 'Color', black);
title('');
ax = gca; 
ax.FontSize = 11.5; 
ax.XColor = black;
ax.YColor = black;
hold off

%Dutch Roll
G_dutch_r = 1/((s-p_lat(5))*(s-p_lat(6)));
figure(5)
hold on
opt = stepDataOptions;
opt.InputOffset = 0;
opt.StepAmplitude = 1;
step(G_dutch_r,opt)
xlabel('Time', 'FontSize', 15, 'Color', black);
ylabel('Amplitude', 'FontSize', 15, 'Color', black);
title('');
ax = gca; 
ax.FontSize = 11.5; 
ax.XColor = black;
ax.YColor = black;
hold off

%% Poles of the eigenmotions in a pzmap

%% Longitudinal Poles
%%
figure(6); 
hold on
p = pzoptions;
p.Title.String = '';
p.XLabel.String = 'Real Axis';
p.XLabel.FontSize = 17;
p.YLabel.String = 'Imaginary Axis';
p.YLabel.FontSize = 17;
p.TickLabel.FontSize = 12;
pzmap(SS_long_ac, p);
sgrid;
a = findobj(gca,'type','line')
for i = 1:length(a)
    set(a(i),'markersize',13) %change marker size
    set(a(i), 'linewidth',3)  %change linewidth
end
hold off


%% Lateral Poles
%%
figure(7); 
hold on
p = pzoptions;
p.Title.String = '';
p.XLabel.String = 'Real Axis';
p.XLabel.FontSize = 17;
p.YLabel.String = 'Imaginary Axis';
p.YLabel.FontSize = 17;
p.TickLabel.FontSize = 12;
pzmap(SS_lat_ac, p);
sgrid;
a = findobj(gca,'type','line')
for i = 1:length(a)
    set(a(i),'markersize',13) %change marker size
    set(a(i), 'linewidth',3)  %change linewidth
end
hold off


