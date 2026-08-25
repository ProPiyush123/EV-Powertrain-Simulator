% Battery SOC Simulation - Phase 2 (Driving Profile)
clear; clc;

% --- Battery parameters ---
Q = 2.0;          % Battery capacity in Amp-hours (Ah)
SOC0 = 100;        % Initial SOC in %

% --- Simulation settings ---
dt = 1/60;         % time step in hours (1 minute)
t_end = 1;         % simulate 1 hour
t = 0:dt:t_end;    % time vector
N = length(t);

% --- Build the driving profile (current at each time step) ---
I_load = zeros(1, N);   % preallocate current array

for k = 1:N
    time_min = t(k) * 60;   % convert current time to minutes
    
    if time_min <= 15
        I_load(k) = 5 * (time_min / 15);      % ramp up 0 -> 5A
    elseif time_min <= 35
        I_load(k) = 3;                         % cruising at 3A
    elseif time_min <= 45
        I_load(k) = -1;                        % regen braking, -1A
    else
        I_load(k) = 0;                         % idle
    end
end

% --- Preallocate SOC array ---
SOC = zeros(1, N);
SOC(1) = SOC0;

% --- Simulate ---
for k = 1:N-1
    SOC(k+1) = SOC(k) - (I_load(k) * dt / Q) * 100;
    if SOC(k+1) < 0
        SOC(k+1) = 0;
    elseif SOC(k+1) > 100
        SOC(k+1) = 100;
    end
end

% --- Plot both SOC and Current ---
figure;
subplot(2,1,1);
plot(t, SOC, 'LineWidth', 2);
xlabel('Time (hours)');
ylabel('SOC (%)');
title('Battery SOC vs Time');
grid on;

subplot(2,1,2);
plot(t, I_load, 'r', 'LineWidth', 2);
xlabel('Time (hours)');
ylabel('Current (A)');
title('Driving Profile (Current Draw)');
grid on;