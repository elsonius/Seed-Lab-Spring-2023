%% RunControllerFullSim.m
% This script implements a fully tuned PD controller on the detailed DC motor model, and verifies these results. 
%
% required file: controllerfullsim.slx
%
%% Define motor parameters
Ra=1; % armaature resistance [Ohms]
Kt=.5; % motor torque constant [Nm/A]
Ke=.5; % back emf constant [Vs/rad]
J=.05; % Load inertia [Nm^2]
b=.5; % damping [Nm/s]
%% Define transfer function parameters
K=5; % gain []
sigma=0.303; % sigma []
%% Run a Simulation
%
% This simulation finalized the previous simulations by verifying that the
% controller works with the original detailed simulink model.
%
%% 
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('controllerfullsim')
%
% run the simulation
%
out=sim('controllerfullsim');
%% A Plot of the results
%
% We see that, with a tuned controller, the position closely follows the desired position per the given voltage.
% 
%
% figure
% plot(out.outputs)
% plot(out.inputs)

%t = tiledlayout(1,2);


%Figure plots:
%plot(scope1)
figure(1)
plot(out.Position, 'r');
title('Figure 1 (Output: Position Actual)')
% % xlabel('θ °');
% % ylabel('Array Factor / E Field Radiation Patern (dB)');
% % grid on
% % legend('N=10, β=0°, d=user value, f=915MHz','N=15, β=0°, d=user value, f=2.4GHz')
% % datacursormode on
% 
figure(2)
plot(out.DesiredPosition, 'b');
title('Figure 2 (Output: Desired Position)')

figure(3)
plot(out.Voltage, '--g');
title('Figure 2 (Output: Voltage)')

