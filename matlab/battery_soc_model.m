% Battery SOC Simulation - Phase 1
clear; clc;

% --- Battery parameters ---
Q = 4.0;          % Battery capacity in Amp-hours (Ah)
SOC0 = 50;        % Initial SOC in %
I_load = 2.0;      % Constant discharge current in Amps

% --- Simulation settings ---
dt = 1/6;         % time step in hours (1 minute)
t_end = 4;         % simulate 2 hours
t = 0:dt:t_end;    % time vector
N = length(t);

% --- Preallocate SOC array ---
SOC = zeros(1, N);
SOC(1) = SOC0;

% --- Simulate ---
for k = 1:N-1
    SOC(k+1) = SOC(k) - (I_load * dt / Q) * 100;
    if SOC(k+1) < 0
        SOC(k+1) = 0;
    end
end

% --- Plot ---
plot(t, SOC, 'LineWidth', 2);
xlabel('Time (hours)');
ylabel('State of Charge (%)');
title('Battery SOC vs Time');
grid on;