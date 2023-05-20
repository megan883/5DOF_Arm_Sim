%This script takes inputs of the joint angles, voint velocities and joint
%accelerations and outputs the torques for each joint and displays it in
%the terminal.

close all 
clear all 
clc 

%mass of the links 
m1 = 9.884; 
m2 = 7.02; 
m3 = 1.1902; 

%link lengths 
a1 = -0.2; 
a2 = 0.037; 
a3 = 0.4; 
a4 = 0.4; 

%lengths from the motor to the centre of mass(COM)
%motor 1 to COM of link 1
l1z = 0.089736;
l1y = 0.001676;
l1x = 0.009189;
%hyponenuse of x and y (distance from base to mass 1)
l1h = 0.00965;

%motor 2 to COM of link 2
l2z = 0;
l2y = 0.026217;
l2x = 0.180954;
%hyponenuse of x and y (distance from motor 1 to mass 2)
l2h = 0.1828;

%motor 3 to COM of link 3
l3z = 0;
l3y = 0;
l3x = 0.200000;

%acceleration due to gravity 
g = 9.81; 

%joint angles 
t1 = 0; 
t2 = 0; 
t3 = 0; 

%Joint velocities 
t1d = 0; 
t2d = 0; 
t3d = 0; 

%Joint accelerations
t1dd = 0; 
t2dd = 0; 
t3dd = 0; 

%the angle of mass 1 and mass 2
m1a = 0.174533;
m2a = 0.139626;

%Joint torques
T1 = (37249*m1*t1dd)/400000000 + (208849*m2*t1dd)/12500000 + a2^2*m2*t1dd + a2^2*m3*t1dd + (a3^2*m3*t1dd)/2 + (a4^2*m3*t1dd)/8 + (208849*m2*cos(2*t2 + 5030556812569863/18014398509481984)*t1dd)/12500000 - (208849*m2*sin(2*t2 + 5030556812569863/18014398509481984)*t1d*t2d)/6250000 + (457*a2*m2*cos(t2 + 5030556812569863/36028797018963968)*t1dd)/1250 + (a4^2*m3*cos(2*t2 + 2*t3)*t1dd)/8 + (a3^2*m3*cos(2*t2)*t1dd)/2 + (a3*a4*m3*cos(2*t2 + t3)*t1dd)/2 - (457*a2*m2*sin(t2 + 5030556812569863/36028797018963968)*t1d*t2d)/1250 - (a4^2*m3*sin(2*t2 + 2*t3)*t1d*t2d)/4 - (a4^2*m3*sin(2*t2 + 2*t3)*t1d*t3d)/4 + 2*a2*a3*m3*cos(t2)*t1dd + (a3*a4*m3*cos(t3)*t1dd)/2 + a2*a4*m3*cos(t2 + t3)*t1dd - a3^2*m3*sin(2*t2)*t1d*t2d - a3*a4*m3*sin(2*t2 + t3)*t1d*t2d - (a3*a4*m3*sin(2*t2 + t3)*t1d*t3d)/2 - 2*a2*a3*m3*sin(t2)*t1d*t2d - (a3*a4*m3*sin(t3)*t1d*t3d)/2 - a2*a4*m3*sin(t2 + t3)*t1d*t2d - a2*a4*m3*sin(t2 + t3)*t1d*t3d
T2 = (448317*m2*cos(t2))/250000 + (208849*m2*t2dd)/6250000 + a3^2*m3*t2dd + (a4^2*m3*t2dd)/4 + (a4^2*m3*t3dd)/4 + (981*a3*m3*cos(t2))/100 + (981*a4*m3*cos(t2 + t3))/200 + (208849*m2*sin(2*t2 + 5030556812569863/18014398509481984)*t1d^2)/12500000 + (a4^2*m3*sin(2*t2 + 2*t3)*t1d^2)/8 + (a3^2*m3*sin(2*t2)*t1d^2)/2 + (457*a2*m2*sin(t2 + 5030556812569863/36028797018963968)*t1d^2)/2500 + a2*a3*m3*sin(t2)*t1d^2 + (a2*a4*m3*sin(t2 + t3)*t1d^2)/2 + a3*a4*m3*cos(t3)*t2dd + (a3*a4*m3*cos(t3)*t3dd)/2 + (a3*a4*m3*sin(2*t2 + t3)*t1d^2)/2 - a3*a4*m3*sin(t3)*t2d*t3d - (a3*a4*m3*sin(t3)*t3d*t3d)/2
T3 = (a4^2*m3*t2dd)/4 + (a4^2*m3*t3dd)/4 + (981*a4*m3*cos(t2 + t3))/200 + (a4^2*m3*sin(2*t2 + 2*t3)*t1d^2)/8 + (a3*a4*m3*sin(t3)*t1d^2)/4 + (a3*a4*m3*sin(t3)*t2d^2)/2 + (a2*a4*m3*sin(t2 + t3)*t1d^2)/2 + (a3*a4*m3*cos(t3)*t2dd)/2 + (a3*a4*m3*sin(2*t2 + t3)*t1d^2)/4 - (a3*a4*m3*sin(t3)*t2d*t3d)/2 + (a3*a4*m3*sin(t3)*t2d*t3d)/2


%Displaying torques
disp("Torque of First Joint");
disp(T1);
disp("Torque of Second Joint");
disp(T2);
disp("Torque of Third Joint");
disp(T3);