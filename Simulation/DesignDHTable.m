function link = DesignDHTable(T, L)
    Find_J = 0;
    % Theta, d, a, alpha
    DH = [T(1), L(1), L(2), pi/2;
          T(2), 0, L(3), 0;
          T(3), 0, 0, -pi/2; 
          T(4), L(4), 0, pi/2;
          T(5), 0, L(5), -pi/2];
    
    % Uses Cell assignment to allocate seperate structs for each arm link.
    % Puts values into resuable struct.
    for i = 1:5 
        link{i} = struct('theta', DH(i, 1), 'alpha', DH(i, 4), 'a', DH(i, 3), 'd', DH(i, 2));
    end
    
    % Finding jacobian of endpoint.
    if (Find_J == 1)
        T1 = sym('T1');
        T2 = sym('T2');
        T3 = sym('T3');
        T4 = sym('T4');
        T5 = sym('T5');
        L1 = sym('L1');
        L2 = sym('L2');
        L3 = sym('L3');
        L4 = sym('L4');
        L5 = sym('L5');
        
        DH_J = [T1, L1, L2, pi/2;
                T2, 0, L3, 0;
                T3, 0, 0, -pi/2; 
                T4, L4, 0, pi/2;
                T5, 0, L5, -pi/2];
        for i = 1:5
            link{i} = struct('theta', DH_J(i, 1), 'alpha', DH_J(i, 4), 'a', DH_J(i, 3), 'd', DH_J(i, 2));
            
            RotZ = [cos(link{i}.theta) -sin(link{i}.theta) 0 0; sin(link{i}.theta) cos(link{i}.theta) 0 0; 0 0 1 0; 0 0 0 1];
            TransZ = [1 0 0 0; 0 1 0 0; 0 0 1 link{i}.d; 0 0 0 1];
            TransX = [1 0 0 link{i}.a; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            RotX = [1 0 0 0; 0 cos(link{i}.alpha) -sin(link{i}.alpha) 0; 0 sin(link{i}.alpha) cos(link{i}.alpha) 0; 0 0 0 1];
            
            % Get Completed Homogeneous Transformation.
            HT{i} = RotZ * TransZ * TransX * RotX;
        end
        
        HM50_P = (HT{1} * HT{2} * HT{3} * HT{4} * HT{5}) * [0; 0; 0; 1];
        
        J = jacobian(HM50_P, [T1, T2, T3, T4, T5]);
        
        J
    end
end

