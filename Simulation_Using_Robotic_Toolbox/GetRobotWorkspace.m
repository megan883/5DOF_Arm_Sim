%note the last graph may take a while to generate
function w = GetRobotWorkspace(SainSmartRobot, midConfig, type_of_plot);

%switch statement to get correct type of plot
switch type_of_plot
    case 'Plot'
        
        %midConfig
        SainSmartRobot.plot  ([midConfig]);
        title('5DOF Arm');
    case 'Teach'
        
        %getting the teach graph
        SainSmartRobot.teach  ([midConfig]);
        title('5DOF:Interactive');
    case 'scatter_plot'
        
        %figure hold on to include all points and robot on graph
        figure
        hold on
        
        %changing limits so it all fits on graph
        zlim([-1400 1000]);
        
        %plotting all points
        for w = -pi:0.001:pi
            
            % creating random points between the min and max range i
            % allowed it to spin
            o = rand(1);
            i = rand(1);
            u = rand(1);
            y = rand(1);
            r = rand(1);
            %k = rand(1);
            T13= i*2*pi-pi/2;
            T23= o*2*pi-pi/2;
            T33= u*2*pi-pi/2;
            T43= y*2*pi-pi/2;
            T53= r*2*pi-pi/2;
            %T63= k*2*pi-pi/2;
            
            T3 = [T13,T23,T33,T43,T53];
            
            %getting the matrix of the end effector
            V = SainSmartRobot.fkine(T3)*[0;0;1];
            X = V(1,1);
            Y = V(2,1);
            Z = V(3,1);
            
            B = -X*tand(30);
            
            C = sqrt(X*X + Y*Y + Z*Z);
            
            if( X < 1000 && Y < 1000 && Z < 1000 && X > -1000 && Y > -1000)
            
            if( C > 600 )
            
            if( B > Z )
            
                %plotting each point
                plot3(X,Y,Z,'b.','MarkerSize',1);
            
            end
            
            end
            
            end
            
        end
        
        %plotting the arm on the graph after the points
        SainSmartRobot.plot(midConfig);
        title('5DOF:Working area');
        
    case 'animate'
        
        radius = 500;
        xCenter = 0;
        yCenter = 0;
        rows = 720;
        columns = 1280;
        theta = linspace(0, 360, 4*pi*radius); % More than needed to avoid gaps.
        x = xCenter + radius * cosd(theta);
        y = yCenter + radius * sind(theta);
        xy = round([x', y']);
        %xy = unique(xy, 'rows');
        x = xy(:, 1);
        y = xy(:, 2);

        for f = 1:120:6500
            
            bee = transl([x(f) y(f) -1000]);
            E = SainSmartRobot.ikunc(bee);
            SainSmartRobot.plot(E);
        end
        
        %input = out.getElement(
        %disp(out);
        
        %animating the robot drawing a square
%         for f = -300:20:300
%             
%             %generating the joint angles after specifying an end effector
%             %position
%             bee = transl([f 300 -700]);
%             E = SainSmartRobot.ikunc(bee);
%             SainSmartRobot.plot(E);
%             
%             %adding a title to the animation
%             title('5DOF Arm: Trajectory Animation');
%         end
%         
%         %one for loop for each of the sides of the square
%         for f = 300:-20:-300
%             
%             %transl gives bee the homeogenious version of the co-ordinates
%             bee = transl([300 f -700]);
%             
%             %storing the joint angles in E. 
%             E = SainSmartRobot.ikunc(bee);
%             
%             %plotting the joint angles
%             SainSmartRobot.plot(E);
%         end
%         for f = 300:-20:-300
%             bee = transl([f -300 -700]);
%             E = SainSmartRobot.ikunc(bee);
%             SainSmartRobot.plot(E);
%         end
%         for f = -300:20:300
%             bee = transl([-300 f -700]);
%             E = SainSmartRobot.ikunc(bee);
%             SainSmartRobot.plot(E);
%         end
        
        %enerate a new graph of the joint angle trajectories
        figure
        
        %hold on keeps all the plottes graphs on the same graph
        hold on
        
        % T is a time variable to plot along the x axis
        T = 1;
        
        %initualising a matirx
        A = zeros(7,40);
        
        %one for loop for each sides of the square
        for f = 90:10:590
            
            %bee stores the point as a homeogineous matrix
            bee = transl([f 150 -100]);
            
            % E strors the joint angles
            E = SainSmartRobot.ikunc(bee);
            
            
            %store each joint variable in a matrix
            A(1,T) = E(1);
            A(2,T) = E(2);
            A(3,T) = E(3);
            A(4,T) = E(4);
            A(5,T) = E(5);
            A(6,T) = E(6);
            A(7,T) = T;
            T = T + 1;
            
        end
        for f = 150:-10:-150
            bee = transl([590 f -100]);
            E = SainSmartRobot.ikunc(bee);
            A(1,T) = E(1);
            A(2,T) = E(2);
            A(3,T) = E(3);
            A(4,T) = E(4);
            A(5,T) = E(5);
            A(6,T) = E(6);
            A(7,T) = T;
            T = T + 1;
            
        end
        for f = 590:-10:90
            bee = transl([f -150 -100]);
            E = SainSmartRobot.ikunc(bee);
            A(1,T) = E(1);
            A(2,T) = E(2);
            A(3,T) = E(3);
            A(4,T) = E(4);
            A(5,T) = E(5);
            A(6,T) = E(6);
            A(7,T) = T;
            T = T + 1;
            
        end
        for f = -150:10:150
            bee = transl([90 f -100]);
            E = SainSmartRobot.ikunc(bee);
            A(1,T) = E(1);
            A(2,T) = E(2);
            A(3,T) = E(3);
            A(4,T) = E(4);
            A(5,T) = E(5);
            A(6,T) = E(6);
            A(7,T) = T;
            T = T + 1;
            
        end
        %plotting graphs
        plot(A(7,:), A(1,:), 'b-','LineWidth',2);
        plot(A(7,:), A(2,:), 'r-','LineWidth',2);
        plot(A(7,:), A(3,:), 'c-','LineWidth',2);
        plot(A(7,:), A(4,:), 'g--','LineWidth',2);
        plot(A(7,:), A(5,:), 'm--','LineWidth',2);
        plot(A(7,:), A(6,:), 'y--','LineWidth',2);
        
        %including a legend title x label and y label
        title('YOURIDHERE:SetupSainsmart6DOF:Anglular Trajectories');
        xlabel('Time Piont [Arb]','fontsize',14);
        ylabel('Angle [rad]','fontsize',14);
        legend('A1','A2','A3','A4','A5','A6');
    case 'Jacobian'
        
        %J = geometricJacobian(SainSmartRobot,midConfig,'L6');
end
% get the robot workspace
w = [];

% to do
% add your code here