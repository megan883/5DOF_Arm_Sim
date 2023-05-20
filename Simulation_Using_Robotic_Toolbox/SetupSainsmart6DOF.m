function [ SainSmartRobot midConfig controlLimits]  =  SetupSainsmart6DOF(robotName)
% setup the Sainsmart6DOF robot
% define DH table for 6DOF RRRRRR robot
% Uses Peter Corke's robotics toolbox
% CLASSICAL DH used here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 31/03/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Link lengths for the arm
La = -218;
Lb = -13;
L2 = -400;
L3 = -500;
L4 = -100;

%Links 1 to 6 theta, z, x, alpha
L(1) = Link([ 0 La Lb pi/2]);
L(2) = Link([ 0 0  L2  0]);
L(3) = Link([ 0 0 0  -pi/2]);
L(4) = Link([ 0 L3  0  pi/2]);
L(5) = Link([ 0 0  L4 -pi/2]);

%Joining the links
disp('sainsmartrobot')
Rob = SerialLink (L)

%mid point for joint parameters
T11= 0;
T21= 0;
T31= 0;
T41= 0;
T51= 0;
%T61= 0;

%printing out positions
disp('midconfig')
T = [T11,T21,T31,T41,T51];
disp(T);

%min and max joint positions
T1= pi/2;
T2= pi/2;
T3= pi/2;
T4= pi/2;  
T5= pi/2;
%T6= pi/2;

%printing out positions
disp('controllimits')
T_min_max = [T1,T2,T3,T4,T5];
disp(T_min_max);

%Rob.plot = (T)
Rob.name = robotName;
SainSmartRobot=[Rob];
midConfig = [T];
controlLimits = [T_min_max];

% to do
% add your code here
