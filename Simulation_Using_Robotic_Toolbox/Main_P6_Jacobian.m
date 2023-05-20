%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 17/02/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analysis and simulation of 6DOF Sainsmart RC Servo robot
% Uses Peter Corke's robotics toolbox
% CLASSICAL DH used here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

% set up robotics tool box
SetupRTB();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define DH table for 6DOF Sainsmart robot

% specify name of robot
robotName = '5DOF Arm';

% setup the Sainsmart 6DOF robot
[ SainSmartRobot midConfig controlLimits]  =  SetupSainsmart6DOF(robotName);

%Jacobian at position q
q = [0 pi pi/2 -pi 0 0];

%Calculate Jacobian
J = SainSmartRobot.jacobe(q);

%Display Jacobian
disp('Jacobian:');
disp(J);

%[tau,jac0] = SianSmartRobot.gravjac([0 0 0 0 0 0]);

%disp(tau);

%type of plot to get plot from the function
type_of_plot = 'Jacobian';

% put your code in the function to setup the robot workspace
w = GetRobotWorkspace(SainSmartRobot, midConfig, type_of_plot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% show static plot of robot configuation


%all code to get plots is in GetRobotWorkspace