function CIRC = CALC_CIRCLE_TRAJ(T, L)
    %% Sample count and radius of circle.
    SAMPLE_CNT = 180;
    RADIUS = 0.5;
    % Show graph of circle
    SHOW_CIRCLE = 0;

    %% Get HM Transform of initial position.
    [HM, HMPos] = GetRRRRRRHM(T, L);
    
    %% Create the circle trajectory.
    THETA_CIRC = linspace(-pi, pi, SAMPLE_CNT);
    
    ENDPOINT = HMPos.P5;
    
    for i = 1:length(THETA_CIRC)
        CIRC_X(i) = ENDPOINT(1) + RADIUS * (cos(THETA_CIRC(i)));
        CIRC_Y(i) = ENDPOINT(2) + RADIUS * sin(THETA_CIRC(i));
        CIRC_Z(i) = ENDPOINT(3);
    end
    %% Store circle.
    CIRC = [CIRC_X; CIRC_Y; CIRC_Z];
    
    %% Show the circle on the graph.
    if (SHOW_CIRCLE == 1)
        figure;
        hold on;
        plot3(CIRC_X, CIRC_Y, CIRC_Z, 'b.');
        axis equal;
        axis square;
        xlabel('x[m]');
        ylabel('y[m]');
        zlabel('z[m]');
        title("CIRCLE TRAJECTORY");
        view(40, 40);
        hold off;
    end
end

