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

% specify name of robot
robotName = 'SetupSainsmart6DOF';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup representation of robot using DH parameters and bui robotics tool box robot object

% To do
% place your DH-table for the robot here as MATLAB comments

% +---+-----------+-----------+-----------+-----------+
% |   |     theta |         d |         a |     alpha |
% +---+-----------+-----------+-----------+-----------+
% |  1|         0 |        108|       29.5|    -1.5708|
% |  2|   -1.5708 |          0|        120|          0|
% |  3|         0 |          0|         20|     1.5708|
% |  4|         0 |        120|          0|     1.5708|
% |  5|         0 |          0|          0|    -1.5708|
% |  6|         0 |         30|          0|          0|
% +---+-----------+-----------+-----------+-----------+
% To do
% put your code in the function to setup the Sainsmart 6DOF robot
[ SainSmartRobot midConfig controlLimits]  =  SetupSainsmart6DOF(robotName);
startup_rvc

