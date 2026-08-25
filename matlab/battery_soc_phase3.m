% Battery SOC Simulation - Phase 3 (Motor Model with Throttle Controller)
clear; clc;

% --- Battery parameters ---
Q = 20;          % Battery capacity in Amp-hours (Ah)
SOC0 = 100;        % Initial SOC in %

% --- Vehicle parameters ---
r_wheel = 0.3;     % wheel radius (meters)
m = 1200;          % vehicle mass (kg)

% --- Motor parameters ---
V = 48;        % Battery/supply voltage (Volts)
R = 0.5;       % Motor winding resistance (Ohms)
Kt = 0.5;      % Torque constant (Nm/A)
Ke = 0.5;      % Back-EMF constant (V*s/rad)
I_discharge_max = 50;   % Max discharge current (Amps)
I_charge_max = 15;      % Max charge/regen current (Amps) - lower, batteries accept charge more slowly

% --- Simulation settings ---
dt = 1/60;         % time step in hours (1 minute)
t_end = 1;         % simulate 1 hour
t = 0:dt:t_end;    % time vector
N = length(t);

% --- Build speed profile (motor angular speed, rad/s) ---
omega = zeros(1, N);

for k = 1:N
    time_min = t(k) * 60;
    
    if time_min <= 15
        omega(k) = 100 * (time_min / 15);          % ramp up 0 -> 100 rad/s
    elseif time_min <= 35
        omega(k) = 100;                              % cruising at 100 rad/s
    elseif time_min <= 45
        omega(k) = 100 * (1 - (time_min-35)/10);    % ramp down 100 -> 0 rad/s
    else
        omega(k) = 0;                                % idle
    end
end

% --- Build throttle/controller profile (-1 to 1) ---
throttle = zeros(1, N);

for k = 1:N
    time_min = t(k) * 60;
    
    if time_min <= 15
        throttle(k) = 1;        % full throttle - accelerating
    elseif time_min <= 35
        throttle(k) = 0.3;      % light throttle - maintaining cruise
    elseif time_min <= 45
        throttle(k) = -1;       % full regen braking
    else
        throttle(k) = 0;        % idle - no current demanded
    end
end

% --- Calculate raw motor current from physics ---
I_raw = (V - Ke .* omega) / R;

% --- Apply throttle to get actual current ---
I_load = throttle .* I_raw;

% --- Clamp current to safe limits ---
I_load(I_load > I_discharge_max) = I_discharge_max;
I_load(I_load < -I_charge_max) = -I_charge_max;

% --- Preallocate SOC array ---
SOC = zeros(1, N);
SOC(1) = SOC0;

% --- Calculate torque, force, and acceleration ---
T = Kt .* I_load;        % torque (Nm)
F = T / r_wheel;         % force at wheels (N)
a = F / m;                % acceleration (m/s^2)

% --- Simulate SOC ---
for k = 1:N-1
    SOC(k+1) = SOC(k) - (I_load(k) * dt / Q) * 100;
    if SOC(k+1) < 0
        SOC(k+1) = 0;
    elseif SOC(k+1) > 100
        SOC(k+1) = 100;
    end
end

% --- Plot everything ---
figure;

subplot(5,1,1);
plot(t, omega, 'g', 'LineWidth', 2);
ylabel('Speed (rad/s)');
title('Motor Speed Profile');
grid on;

subplot(5,1,2);
plot(t, throttle, 'm', 'LineWidth', 2);
ylabel('Throttle');
title('Throttle / Controller Input');
grid on;

subplot(5,1,3);
plot(t, I_load, 'r', 'LineWidth', 2);
ylabel('Current (A)');
title('Motor Current');
grid on;

subplot(5,1,4);
plot(t, a, 'c', 'LineWidth', 2);
ylabel('Accel (m/s^2)');
title('Vehicle Acceleration');
grid on;

subplot(5,1,5);
plot(t, SOC, 'b', 'LineWidth', 2);
xlabel('Time (hours)');
ylabel('SOC (%)');
title('Battery SOC vs Time');
grid on;