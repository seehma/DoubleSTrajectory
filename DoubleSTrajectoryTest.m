%% Testscript for DoubleSTrajectory Planner
%
% Author:  Matthias Seehauser
% Date:    26.04.2015
% Version: 1.0
%
%

%% Initialize Workspace
clc;
clear;

%% Calculate the Trajectory by Function calling
%
[q, qp, qpp, qppp] = DoubleSTrajectoryFunction( 0.001, 0, -30, 0, 0, 20, -20, 60, -60, 120, -120 );

%% Drawing the Trajectory in Plots
%

% create a subplot with 4 drawings and draw position, speed, acceleration and jerk
figure(1)
subplot(2,2,1);
plot(q);
title('Position');
subplot(2,2,2);
plot(qp);
title('Speed');
subplot(2,2,3);
plot(qpp);
title('Acceleration');
subplot(2,2,4);
plot(qppp);
title('Jerk');
