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

endeffectorMass = 12;
tFinal = 10;
K = 1;
forces = 1;
Gain = 1;
mass = 1;
sim('simulink_model');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% specify name of robot
robotName = '5DOF Arm';

% setup the Sainsmart 6DOF robot
[ SainSmartRobot midConfig controlLimits]  =  SetupSainsmart6DOF(robotName);

%type of plot to get plot from the function
type_of_plot = 'animate';

% put your code in the function to setup the robot workspace
w = GetRobotWorkspace(SainSmartRobot, midConfig, type_of_plot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% generate animated end-effector trajectory
%https://youtube.com/shorts/lrMpiXwFVFM

%last plot takes a while to show up

%all code to get plots is in GetRobotWorkspace