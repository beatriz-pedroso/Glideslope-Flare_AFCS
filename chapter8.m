%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The state space models 
%     and the trimmed values are determined. The actuator 
%     dynamics are implemented. This .m file includes the
%     simulink file "with_glideslope_and_flare.slx", in
%     which the flare and glideslope model is constructed.
%     Several figures with the relevant information are plotted.
% 
% Authors: 
%       Pedroso, Beatriz 
%       Silva, Pedro
%       Yikilmaz, Cansu
%================================================
clear;

global fi_flag_Simulink

newline = newline;

%% Trim aircraft to desired altitude and velocity
%%
altitude = 5000; %ft
V0 = 300; %ft/s

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
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, V0, altitude, FC_flag);

%% Find the state space model for the lofi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; 
trim_thrust_lin = trim_thrust_lo; 
trim_control_lin = trim_control_lo;
operating_point = operpoint('LIN_F16Block_og'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; 
operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); 
operating_point.Inputs(4).u = trim_control_lin(3);

SS_lo = linearize('LIN_F16Block_og');


%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Direction
%%%%%%%%%%%%%%%%%%%%%%%

long_states = [3 5 7 8 11];

long_inputs = [13 14]; 
long_outputs = [3 5 7 8 11];

SS_long_lo = ss(SS_lo.A(long_states,long_states), ...
                SS_lo.A(long_states,long_inputs), ...
                SS_lo.C(long_outputs,long_states), ...
                SS_lo.C(long_outputs,long_inputs));


%%
%% Aircraft + AFCS

%ENGINE AND ELEVATOR DYNAMICS

% Saturation values for total values of engine and elevator

th_sat_lower_tot = 1000; %lb
th_sat_upper_tot = 19000; %lb

elev_sat_lower_tot = -25; %deg
elev_sat_upper_tot = 25; %deg

%Saturation values for the deviation values of engine and elevator

th_sat_lower = th_sat_lower_tot - trim_thrust_lin;
th_sat_upper = th_sat_upper_tot - trim_thrust_lin;

elev_sat_lower = -(elev_sat_upper_tot + trim_control_lin(1));
elev_sat_upper = -(elev_sat_lower_tot + trim_control_lin(1));


%% Simulink simulation

% define initial flare altitude and tau
h_flare = 45;
tau = 2.867;

sim("with_glideslope_and_flare.slx")

% retrieve simulated data

time = ans.altitude.time;
altitude = ans.altitude.signals.values;
x_position = ans.x_position.signals.values;
error_angle = ans.glideslope_error.signals.values;
v_speed = ans.v_speed.signals.values;

% glideslope reference signal for plots

ramp_ref = ans.glideslope_ref.signals.values;

% flare initial position determination
% check at which array index altitude == h_flare

j=0;

for i = 1:length(time)
    if altitude(i, 1) == h_flare
        j = i;
    end
end

if j == 0
    disp("Error: Flare not intercepted")
end


x0_flare = x_position(j,1);
t0_flare = time(j,1);

% separate relevant glideslope and flare data for specific plots

% glideslope data 
    glideslope_time = zeros(j,1);
    glideslope_altitude = zeros(j,1);
    glideslope_x_position = zeros(j,1);
    glideslope_error = zeros(j,1);
    glideslope_ref = zeros(j,1);

for i=1:j
    glideslope_time(i,1) = time(i,1);
    glideslope_altitude(i,1) = altitude(i,1);
    glideslope_x_position(i,1) = x_position(1,1)-x_position(i,1);
    glideslope_error(i,1) = error_angle(i,1);
    glideslope_ref(i,1) = ramp_ref(i,1);
end

% flare data
    flare_time = zeros(length(time)-j+1,1);
    flare_altitude = zeros(length(time)-j+1,1);
    flare_x_position = zeros(length(time)-j+1,1);
    flare_v_speed = zeros(length(time)-j+1,1);
    fglideslope_ref = zeros(length(time)-j+1,1);

for i=j:length(time)
    flare_time(i-j+1,1) = time(i,1);
    flare_altitude(i-j+1,1) = altitude(i,1);
    flare_x_position(i-j+1,1) = x_position(1,1) - x_position(i,1);
    flare_v_speed(i-j+1,1) = v_speed(i,1);
    fglideslope_ref(i-j+1,1) = ramp_ref(i,1);
end
%% Flare design path

design_time = [flare_time(1):0.1:flare_time(length(flare_time))];
design_h = h_flare*(exp(-(design_time-flare_time(1))/tau));

%% Plots

%%%%%%%%%%%%%%%%%%%%%
%  Glideslope plots %
%%%%%%%%%%%%%%%%%%%%%

% error angle
figure(1)
grid on
hold on
plot(glideslope_time, glideslope_error)
plot(10, 0, 'o')
xlabel('time (seconds)')
ylabel('error angle (deg.)')
legend('glideslope error angle', 'error at glideslope interception')

% altitude vs time
figure(2)
grid on
axis([0 140 0 2100])
hold on
plot(glideslope_time, glideslope_altitude)
plot(glideslope_time, glideslope_ref, '--')
plot(10, 2000, 'o')
plot(glideslope_time(length(glideslope_time)), glideslope_altitude(length(glideslope_altitude)), 'o')
xlabel('time (seconds)')
ylabel('altitude above runway (ft)')
legend('glideslope path', 'glideslope reference signal', 'glideslope interception point', 'flare interception point')

% altitude vs x-position
figure(3)
grid on
axis([0 140*trim_state_lin(7) 0 2100])
hold on
plot(glideslope_x_position, glideslope_altitude)
plot(glideslope_x_position, glideslope_ref, '--')
plot(10*trim_state_lin(7), 2000, 'o')
plot(glideslope_x_position(length(glideslope_x_position)), glideslope_altitude(length(glideslope_altitude)), 'o')
xlabel('x-position (ft)')
ylabel('altitude above runway (ft)')
legend('glideslope path', 'glideslope reference signal', 'glideslope interception point', 'flare interception point')


% x-position vs time
figure(4)
grid on
axis([0 140 0 140*trim_state_lin(7)])
hold on
plot(glideslope_time, x_position(1)-glideslope_x_position)
plot(glideslope_time(length(glideslope_time)), x_position(1)-glideslope_x_position(length(glideslope_x_position)), 'o')
xlabel('time (seconds)')
ylabel('x-postion (ft)')
legend('x-position during glideslope', 'flare interception point')

%%%%%%%%%%%%%%%
% Flare plots %
%%%%%%%%%%%%%%%

% altitude vs time
figure(5)
grid on
axis([134.3 143 -3 50])
hold on
plot(flare_time, flare_altitude)
plot(design_time, design_h)
plot(flare_time, fglideslope_ref, '--')
plot(flare_time(1), flare_altitude(1), 'o')
plot(flare_time(length(flare_time)), flare_altitude(length(flare_altitude)), 'o')
xlabel('time (seconds)')
ylabel('altitude above runway (ft)')
legend('flare path', 'design path', 'glideslope reference signal', 'flare interception point', 'runway interception point')

% altitude vs x-position
figure(6)
grid on
axis([40500 43000 -3 50])
hold on
plot(flare_x_position, flare_altitude)
plot(flare_x_position, fglideslope_ref, '--')
plot(flare_x_position(1), flare_altitude(1), 'o')
plot(flare_x_position(length(flare_x_position)), flare_altitude(length(flare_altitude)), 'o')
xlabel('x-position (ft)')
ylabel('altitude above runway (ft)')
legend('flare path', 'glideslope reference signal', 'flare interception point', 'runway interception point')

% x-position vs time
figure(7)
grid on
axis([134.3 143 -2000 700])
hold on
plot(flare_time, x_position(1)-flare_x_position)
plot(glideslope_time(length(glideslope_time)), x_position(1)-glideslope_x_position(length(glideslope_x_position)), 'o')
xlabel('time (seconds)')
ylabel('x-postion (ft)')
legend('x-position during flare', 'flare interception point')

%%%%%%%%%%%%%%%%%%%%%%%
% Complete path plots %
%%%%%%%%%%%%%%%%%%%%%%%

% altitude vs time
figure(8)
grid on
axis([0 143 -50 2100])
hold on
plot(glideslope_time, glideslope_altitude)
plot(flare_time, flare_altitude)
plot(10, 2000, 'o')
plot(flare_time(1), flare_altitude(1), 'o')
plot(flare_time(length(flare_time)), flare_altitude(length(flare_altitude)), 'o')
xlabel('time (seconds)')
ylabel('altitude above runway (ft)')
legend('glideslope path', 'flare path', 'glideslope interception point', 'flare interception point', 'runway interception point')


% altitude vs x-position
figure(9)
grid on
axis([0 43000 -50 2100])
hold on
plot(glideslope_x_position, glideslope_altitude)
plot(flare_x_position, flare_altitude)
plot(10*trim_state_lin(7), 2000, 'o')
plot(flare_x_position(1), flare_altitude(1), 'o')
plot(flare_x_position(length(flare_x_position)), flare_altitude(length(flare_altitude)), 'o')
xlabel('x-position (ft)')
ylabel('altitude above runway (ft)')
legend('glideslope path', 'flare path', 'glideslope interception point', 'flare interception point', 'runway interception point')



%%%%%%%%%%%%%%%%%%%%%%%%
% Vertical speed plots %
%%%%%%%%%%%%%%%%%%%%%%%%

% complete path
figure(10)
grid on
axis([0 145 -23 1])
hold on
plot(time,v_speed)
plot(time(length(time)), v_speed(length(v_speed)),'o')
xlabel('time (seconds)')
ylabel('vertical speed (ft/s)')
legend('vertical speed', 'vertical speed at runway interception', 'Location', 'southeast')

% only flare
figure(11)
grid on
axis([134 143 -17 -1.5])
hold on
plot(flare_time,flare_v_speed)
plot(flare_time(length(flare_time)), flare_v_speed(length(flare_v_speed)),'o')
xlabel('time (seconds)')
ylabel('vertical speed (ft/s)')
legend('vertical speed', 'vertical speed at runway interception', 'Location', 'southeast')
