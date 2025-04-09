%% Digital Controller Design
clear all; close all; clc;
%% System Definition
s = tf('s');
G_continuous = (1/(s+1))*(1/s);  % Plant transfer function
Ts = 0.01;  % Sample time (10ms)
G_discrete = c2d(G_continuous, Ts, 'zoh');
%% Controller Designs
% PID Controller - Adjusted tuning for better performance
C_pid = pidtune(G_discrete, 'PID', 2); % Initial tuning
% Further adjustment of Ki
Kp_pid = C_pid.Kp * 16;
Ki_pid = C_pid.Ki * 2.75; % Increased Ki
Kd_pid = C_pid.Kd * 4.5;
C_pid = pid(Kp_pid, Ki_pid, Kd_pid, Ts); % Re-create C_pid with modified Ki
C_pid.Tf = 1/100; % Set derivative filter coefficient (N)
C_pid.Ts = Ts; % Explicitly set the sample time

% Lead-Lag Compensator - Adjusted parameters for lower ramp error
PM = 70; wc = 5.5; % Slightly higher PM, lower wc
[mag,phase] = bode(G_continuous, wc);
phase_needed = PM - (180 + phase);
alpha = (1 - sind(phase_needed))/(1 + sind(phase_needed));
T_lead = 1/(wc*sqrt(alpha));
K_lead = sqrt(alpha)/mag;
K_lead = K_lead * 8; % Increase K_lead
beta = 25; % Further increased beta for lower ramp error
T_lag = 5/(wc/beta);
C_leadlag = c2d(K_lead*(1 + T_lead*s)/(1 + alpha*T_lead*s) * (1 + T_lag*s)/(1 + beta*T_lag*s), Ts, 'tustin');

%% Delete Existing Models
models = {'Digital_PID_System', 'Digital_LeadLag_System', 'Digital_PID_System_Ramp', 'Digital_LeadLag_System_Ramp'};
for i = 1:length(models)
    if exist(models{i}, 'file')
        close_system(models{i}, 0);
        delete([models{i}, '.slx']);
    end
end
%% PID Controller Model
new_system('Digital_PID_System');
open_system('Digital_PID_System');
% Add blocks
add_block('simulink/Sources/Step', 'Digital_PID_System/Reference', 'Time', '0');
add_block('simulink/Math Operations/Sum', 'Digital_PID_System/Sum', 'Inputs', '+-');
add_block('simulink/Discrete/Discrete PID Controller', 'Digital_PID_System/PID',...
    'P', num2str(C_pid.Kp), 'I', num2str(C_pid.Ki), 'D', num2str(C_pid.Kd),...
    'N', '100', 'SampleTime', num2str(Ts));
add_block('simulink/Discontinuities/Saturation', 'Digital_PID_System/Saturation',...
    'UpperLimit', '5', 'LowerLimit', '-5');
add_block('simulink/Discrete/Discrete Transfer Fcn', 'Digital_PID_System/Plant',...
    'Numerator', mat2str(cell2mat(G_discrete.Numerator)),...
    'Denominator', mat2str(cell2mat(G_discrete.Denominator)),...
    'SampleTime', num2str(Ts));
n = 1000;  % Encoder resolution
add_block('simulink/Discontinuities/Quantizer', 'Digital_PID_System/Quantizer',...
    'QuantizationInterval', num2str(2*pi/n));
add_block('simulink/Sinks/Scope', 'Digital_PID_System/Scope');
% Add To Workspace blocks for configuration
add_block('simulink/Sinks/To Workspace', 'Digital_PID_System/y_out',...
    'VariableName', 'y_pid_data', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', 'Digital_PID_System/u_out',...
    'VariableName', 'u_pid_data', 'SaveFormat', 'Timeseries');
% Connect blocks
add_line('Digital_PID_System', 'Reference/1', 'Sum/1');
add_line('Digital_PID_System', 'Sum/1', 'PID/1');
add_line('Digital_PID_System', 'PID/1', 'Saturation/1');
add_line('Digital_PID_System', 'Saturation/1', 'Plant/1');
add_line('Digital_PID_System', 'Plant/1', 'Quantizer/1');
add_line('Digital_PID_System', 'Quantizer/1', 'Sum/2');
add_line('Digital_PID_System', 'Plant/1', 'Scope/1');
add_line('Digital_PID_System', 'Plant/1', 'y_out/1');
add_line('Digital_PID_System', 'Saturation/1', 'u_out/1');
% Arrange System
Simulink.BlockDiagram.arrangeSystem('Digital_PID_System')
save_system('Digital_PID_System');
%% Lead-Lag Controller Model
new_system('Digital_LeadLag_System');
open_system('Digital_LeadLag_System');
% Add blocks
add_block('simulink/Sources/Step', 'Digital_LeadLag_System/Reference', 'Time', '0');
add_block('simulink/Math Operations/Sum', 'Digital_LeadLag_System/Sum', 'Inputs', '+-');
[num,den] = tfdata(C_leadlag, 'v');
add_block('simulink/Discrete/Discrete Transfer Fcn', 'Digital_LeadLag_System/Compensator',...
    'Numerator', mat2str(num), 'Denominator', mat2str(den), 'SampleTime', num2str(Ts));
add_block('simulink/Discontinuities/Saturation', 'Digital_LeadLag_System/Saturation',...
    'UpperLimit', '5', 'LowerLimit', '-5');
add_block('simulink/Discrete/Discrete Transfer Fcn', 'Digital_LeadLag_System/Plant',...
    'Numerator', mat2str(cell2mat(G_discrete.Numerator)),...
    'Denominator', mat2str(cell2mat(G_discrete.Denominator)),...
    'SampleTime', num2str(Ts));
add_block('simulink/Discontinuities/Quantizer', 'Digital_LeadLag_System/Quantizer',...
    'QuantizationInterval', num2str(2*pi/n));
add_block('simulink/Sinks/Scope', 'Digital_LeadLag_System/Scope');
% Add To Workspace blocks for configuration
add_block('simulink/Sinks/To Workspace', 'Digital_LeadLag_System/y_out',...
    'VariableName', 'y_ll_data', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', 'Digital_LeadLag_System/u_out',...
    'VariableName', 'u_ll_data', 'SaveFormat', 'Timeseries');
% Connect blocks
add_line('Digital_LeadLag_System', 'Reference/1', 'Sum/1');
add_line('Digital_LeadLag_System', 'Sum/1', 'Compensator/1');
add_line('Digital_LeadLag_System', 'Compensator/1', 'Saturation/1');
add_line('Digital_LeadLag_System', 'Saturation/1', 'Plant/1');
add_line('Digital_LeadLag_System', 'Plant/1', 'Quantizer/1');
add_line('Digital_LeadLag_System', 'Quantizer/1', 'Sum/2');
add_line('Digital_LeadLag_System', 'Plant/1', 'Scope/1');
add_line('Digital_LeadLag_System', 'Plant/1', 'y_out/1');
add_line('Digital_LeadLag_System', 'Saturation/1', 'u_out/1');
% Arrange System
Simulink.BlockDiagram.arrangeSystem('Digital_LeadLag_System')
save_system('Digital_LeadLag_System');


%% PID Controller Model (Ramp Input - New Name)
new_system('Digital_PID_System_Ramp');
open_system('Digital_PID_System_Ramp');
% Add blocks
add_block('simulink/Sources/Ramp', 'Digital_PID_System_Ramp/Reference', 'Start', '0', 'Slope', '1'); % Ramp input
add_block('simulink/Math Operations/Sum', 'Digital_PID_System_Ramp/Sum', 'Inputs', '+-');
add_block('simulink/Discrete/Discrete PID Controller', 'Digital_PID_System_Ramp/PID',...
    'P', num2str(C_pid.Kp), 'I', num2str(C_pid.Ki), 'D', num2str(C_pid.Kd),...
    'N', '100', 'SampleTime', num2str(Ts));
add_block('simulink/Discontinuities/Saturation', 'Digital_PID_System_Ramp/Saturation',...
    'UpperLimit', '5', 'LowerLimit', '-5');
add_block('simulink/Discrete/Discrete Transfer Fcn', 'Digital_PID_System_Ramp/Plant',...
    'Numerator', mat2str(cell2mat(G_discrete.Numerator)),...
    'Denominator', mat2str(cell2mat(G_discrete.Denominator)),...
    'SampleTime', num2str(Ts));
n = 1000;  % Encoder resolution
add_block('simulink/Discontinuities/Quantizer', 'Digital_PID_System_Ramp/Quantizer',...
    'QuantizationInterval', num2str(2*pi/n));
add_block('simulink/Sinks/Scope', 'Digital_PID_System_Ramp/Scope');
% Add To Workspace blocks for configuration
add_block('simulink/Sinks/To Workspace', 'Digital_PID_System_Ramp/y_out',...
    'VariableName', 'y_pid_ramp_data', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', 'Digital_PID_System_Ramp/u_out',...
    'VariableName', 'u_pid_ramp_data', 'SaveFormat', 'Timeseries'); % Add this line
add_block('simulink/Sinks/To Workspace', 'Digital_PID_System_Ramp/reference_out',...
    'VariableName', 'ref_pid_ramp_data', 'SaveFormat', 'Timeseries'); % Log the reference
% Connect blocks
add_line('Digital_PID_System_Ramp', 'Reference/1', 'Sum/1');
add_line('Digital_PID_System_Ramp', 'Sum/1', 'PID/1');
add_line('Digital_PID_System_Ramp', 'PID/1', 'Saturation/1');
add_line('Digital_PID_System_Ramp', 'Saturation/1', 'Plant/1');
add_line('Digital_PID_System_Ramp', 'Plant/1', 'Quantizer/1');
add_line('Digital_PID_System_Ramp', 'Quantizer/1', 'Sum/2');
add_line('Digital_PID_System_Ramp', 'Plant/1', 'Scope/1');
add_line('Digital_PID_System_Ramp', 'Plant/1', 'y_out/1');
add_line('Digital_PID_System_Ramp', 'Reference/1', 'reference_out/1'); % Connect reference
add_line('Digital_PID_System_Ramp', 'Saturation/1', 'u_out/1'); % This line will now work
% Arrange System
Simulink.BlockDiagram.arrangeSystem('Digital_PID_System_Ramp')
save_system('Digital_PID_System_Ramp');

%% Lead-Lag Controller Model (Ramp Input)
new_system('Digital_LeadLag_System_Ramp');
open_system('Digital_LeadLag_System_Ramp');
% Add blocks
add_block('simulink/Sources/Ramp', 'Digital_LeadLag_System_Ramp/Reference', 'Start', '0', 'Slope', '1'); % Ramp input
add_block('simulink/Math Operations/Sum', 'Digital_LeadLag_System_Ramp/Sum', 'Inputs', '+-');
[num,den] = tfdata(C_leadlag, 'v');
add_block('simulink/Discrete/Discrete Transfer Fcn', 'Digital_LeadLag_System_Ramp/Compensator',...
    'Numerator', mat2str(num), 'Denominator', mat2str(den), 'SampleTime', num2str(Ts));
add_block('simulink/Discontinuities/Saturation', 'Digital_LeadLag_System_Ramp/Saturation',...
    'UpperLimit', '5', 'LowerLimit', '-5');
add_block('simulink/Discrete/Discrete Transfer Fcn', 'Digital_LeadLag_System_Ramp/Plant',...
    'Numerator', mat2str(cell2mat(G_discrete.Numerator)),...
    'Denominator', mat2str(cell2mat(G_discrete.Denominator)),...
    'SampleTime', num2str(Ts));
add_block('simulink/Discontinuities/Quantizer', 'Digital_LeadLag_System_Ramp/Quantizer',...
    'QuantizationInterval', num2str(2*pi/n));
add_block('simulink/Sinks/Scope', 'Digital_LeadLag_System_Ramp/Scope');
% Add To Workspace blocks for configuration
add_block('simulink/Sinks/To Workspace', 'Digital_LeadLag_System_Ramp/y_out',...
    'VariableName', 'y_ll_ramp_data', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', 'Digital_LeadLag_System_Ramp/u_out',...
    'VariableName', 'u_ll_ramp_data', 'SaveFormat', 'Timeseries'); % Add this line
add_block('simulink/Sinks/To Workspace', 'Digital_LeadLag_System_Ramp/reference_out',...
    'VariableName', 'ref_ll_ramp_data', 'SaveFormat', 'Timeseries'); % Log the reference
% Connect blocks
add_line('Digital_LeadLag_System_Ramp', 'Reference/1', 'Sum/1');
add_line('Digital_LeadLag_System_Ramp', 'Sum/1', 'Compensator/1');
add_line('Digital_LeadLag_System_Ramp', 'Compensator/1', 'Saturation/1');
add_line('Digital_LeadLag_System_Ramp', 'Saturation/1', 'Plant/1');
add_line('Digital_LeadLag_System_Ramp', 'Plant/1', 'Quantizer/1');
add_line('Digital_LeadLag_System_Ramp', 'Quantizer/1', 'Sum/2');
add_line('Digital_LeadLag_System_Ramp', 'Plant/1', 'Scope/1');
add_line('Digital_LeadLag_System_Ramp', 'Plant/1', 'y_out/1');
add_line('Digital_LeadLag_System_Ramp', 'Reference/1', 'reference_out/1'); % Connect reference
add_line('Digital_LeadLag_System_Ramp', 'Saturation/1', 'u_out/1'); % This line will now work
% Arrange System
Simulink.BlockDiagram.arrangeSystem('Digital_LeadLag_System_Ramp')
save_system('Digital_LeadLag_System_Ramp');

%% Run Simulations and Calculate Ramp Error
simTime_ramp = 10; % Simulate for a longer time to reach steady state

% Run simulations with Ramp Input
simOut_pid_ramp = sim('Digital_PID_System_Ramp', 'StopTime', num2str(simTime_ramp));
simOut_ll_ramp = sim('Digital_LeadLag_System_Ramp', 'StopTime', num2str(simTime_ramp));

% Extract ramp data
t_pid_ramp = simOut_pid_ramp.get('y_pid_ramp_data').Time;
y_pid_ramp = simOut_pid_ramp.get('y_pid_ramp_data').Data;
ref_pid_ramp = simOut_pid_ramp.get('ref_pid_ramp_data').Data;

t_ll_ramp = simOut_ll_ramp.get('y_ll_ramp_data').Time;
y_ll_ramp = simOut_ll_ramp.get('y_ll_ramp_data').Data;
ref_ll_ramp = simOut_ll_ramp.get('ref_ll_ramp_data').Data;

% Calculate Steady-State Ramp Error from Simulation Data
% Define a time window at the end of the simulation to consider steady state
steady_state_time_window = 5; % Consider the last 5 seconds as steady state
steady_state_indices_pid = t_pid_ramp >= (simTime_ramp - steady_state_time_window);
steady_state_indices_ll = t_ll_ramp >= (simTime_ramp - steady_state_time_window);

% Calculate the mean absolute error in the steady-state region
ramp_error_pid_sim = mean(abs(ref_pid_ramp(steady_state_indices_pid) - y_pid_ramp(steady_state_indices_pid)));
ramp_error_leadlag_sim = mean(abs(ref_ll_ramp(steady_state_indices_ll) - y_ll_ramp(steady_state_indices_ll)));

% Run simulations with Step Input (for other metrics)
simTime_step = 10;
simOut_pid_step = sim('Digital_PID_System', 'StopTime', num2str(simTime_step));
simOut_ll_step = sim('Digital_LeadLag_System', 'StopTime', num2str(simTime_step));

% Extract step data
t_pid_step = simOut_pid_step.get('y_pid_data').Time;
y_pid_step = simOut_pid_step.get('y_pid_data').Data;
t_ll_step = simOut_ll_step.get('y_ll_data').Time;
y_ll_step = simOut_ll_step.get('y_ll_data').Data;

% Calculate Step Response Metrics
step_info_pid = stepinfo(y_pid_step, t_pid_step);
step_info_leadlag = stepinfo(y_ll_step, t_ll_step);

% Calculate Phase Margin (PM) and Gain Margin (GM)
[Gm_pid, Pm_pid] = margin(G_discrete * C_pid);
[Gm_leadlag, Pm_leadlag] = margin(G_discrete * C_leadlag);

%% Plotting Graphs
% Plotting System Response
if exist('y_pid_step', 'var') && exist('y_ll_step', 'var')
    figure;
    plot(t_pid_step, y_pid_step, 'b', t_ll_step, y_ll_step, 'r');
    title('System Response');
    xlabel('Time (s)'); ylabel('\theta(t)');
    legend('PID', 'Lead-Lag');
    grid on;
else
    error('Data logging failed. Check To Workspace blocks in Simulink models.');
end

% Plotting Bode diagram for PID
% Open-Loop Transfer Function with PID
OL_pid = G_discrete * C_pid;
% Calculate Gain and Phase Margins
[Gm_pid, Pm_pid, Wcg_pid, Wcp_pid] = margin(OL_pid);
Gm_pid_db = 20*log10(Gm_pid);
figure;
bode(OL_pid, 'b'); % Use the discrete-time plant
title(['Open-Loop Bode Diagram (PID)']);
subtitle(['Gm = ', sprintf('%.2f', Gm_pid_db), ' dB (at ', sprintf('%.2f', Wcg_pid), ' rad/s), Pm = ', sprintf('%.2f', Pm_pid), ' deg (at ', sprintf('%.2f', Wcp_pid), ' rad/s)']);
grid on;
margin(OL_pid); % Display Gain and Phase Margins

% Plotting Bode diagram for Lead-Lag
% Open-Loop Transfer Function with Lead-Lag
OL_ll = G_discrete * C_leadlag;
% Calculate Gain and Phase Margins
[Gm_ll, Pm_ll, Wcg_ll, Wcp_ll] = margin(OL_ll);
Gm_ll_db = 20*log10(Gm_ll);
figure;
bode(OL_ll, 'r'); % Use red color for Lead-Lag
title(['Open-Loop Bode Diagram (Lead-Lag)']);
subtitle(['Gm = ', sprintf('%.2f', Gm_ll_db), ' dB (at ', sprintf('%.2f', Wcg_ll), ' rad/s), Pm = ', sprintf('%.2f', Pm_ll), ' deg (at ', sprintf('%.2f', Wcp_ll), ' rad/s)']);
grid on;
margin(OL_ll); % Display Gain and Phase Margins
%% Calculate Performance Metrics
% Display all metrics
metrics = {
    'Overshoot (%)', step_info_pid.Overshoot, step_info_leadlag.Overshoot;
    'Ramp Error (Simulation)', ramp_error_pid_sim, ramp_error_leadlag_sim;
    'Settling Time (s)', step_info_pid.SettlingTime, step_info_leadlag.SettlingTime;
    'Rise Time (s)', step_info_pid.RiseTime, step_info_leadlag.RiseTime;
    'Peak Time (s)', step_info_pid.PeakTime, step_info_leadlag.PeakTime;
    'Peak Value', step_info_pid.Peak, step_info_leadlag.Peak;
    'Phase Margin (deg)', Pm_pid, Pm_leadlag;
    'Gain Margin (dB)', 20*log10(Gm_pid), 20*log10(Gm_leadlag);
};
% Create a table
metric_names = metrics(:, 1);
pid_values = cell2mat(metrics(:, 2));
leadlag_values = cell2mat(metrics(:, 3));
% Create the table
performance_table = table(pid_values, leadlag_values, 'RowNames', metric_names);
% Display the table
disp('Performance Metrics Comparison:');
disp(performance_table);