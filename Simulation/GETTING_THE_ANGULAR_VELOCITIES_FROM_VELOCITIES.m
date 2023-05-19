clear all;
close all;
clc;

%% Thetas at initial position.
T = [pi/2, pi/4, 0, 0, pi/4];
L = [-0.2, -0.037, -0.4, -0.4, -0.1];

PLOT_VELOCITIES = 1;

SAMPLE_CNT = 200;
RADIUS = 0.5;

%% Create the circle trajectory.
THETA_CIRC = linspace(-pi, pi, SAMPLE_CNT);

for i = 1:length(THETA_CIRC)
    CIRC_X(i) = 0 + RADIUS * (cos(THETA_CIRC(i)));
    CIRC_Y(i) = 0 + RADIUS * sin(THETA_CIRC(i));
    CIRC_Z(i) = 0.8;
end
%% Store circle.
CIRC = [CIRC_X; CIRC_Y; CIRC_Z];

%% Get the predicted circle path.
THETA = IK_NEWTON_RAPHSON_CIRCLE_TRAJ(T, L);

%% Calculating the VELOCITIES.
for i = 1:length(THETA_CIRC(1, :))
    [J_PINV, J] = GETTING_JACOBIAN(THETA(:, i), L);
    
    %% velocity of the arm traveling around a circle of diameter 1m.
    %% Circumferance = 2*pi*r.
    %% r = 0.5 so circumference = pi
    %% velocity(v) = pi/s
    v = pi;
    
    %% velocity in the x direction
    vx = v*cos(THETA_CIRC(i));
    
    %% velocity in the y direction
    vy = v*sin(THETA_CIRC(i));
    
    %% matrix of velocities
    vm = [vx; vy; 0];
    
    %% plotting the velocities against angular velocities
    
    %THETA = IK_NEWTON_RAPHSON_CIRCLE_TRAJ(T, L);
    
   %[J_PINV, J] = GETTING_JACOBIAN(THETA(:, i), L);
    
    VELOCITIES = J_PINV * vm;
    
    STORE_VELOCITIES(:, i) = VELOCITIES;
end

%% Plot the VELOCITIES at each circle point.
if (PLOT_VELOCITIES == 1)
    figure;
    
    hold on
    
    title("Joint Velocity");
    
    plot(STORE_VELOCITIES(1, :), 'b.-');
    plot(STORE_VELOCITIES(2, :), 'r.-');
    plot(STORE_VELOCITIES(3, :), 'g.-');
    plot(STORE_VELOCITIES(4, :), 'm.-');
    plot(STORE_VELOCITIES(5, :), 'k.-');
    
    legend("Angular Velocity JOINT 1", "Angular Velocity JOINT 2", "Angular Velocity JOINT 3", "Angular Velocity JOINT 4", "Angular Velocity JOINT 5");
    xlabel("CIRCLE SAMPLE");
    ylabel("Angular Velocity [rad/s]");
    
    hold off;
end
