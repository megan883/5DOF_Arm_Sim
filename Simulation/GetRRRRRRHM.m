function [HM, HMPos] = GetRRRRRRHM(T, L)
    %% Designing Devenit Hartenberg table.
    link = DesignDHTable(T, L);

    %% Calculating the homogeneous transform from the table.
    for i = 1:length(link)
        RotZ = [cos(link{i}.theta) -sin(link{i}.theta) 0 0; sin(link{i}.theta) cos(link{i}.theta) 0 0; 0 0 1 0; 0 0 0 1];
        TransZ = [1 0 0 0; 0 1 0 0; 0 0 1 link{i}.d; 0 0 0 1];
        TransX = [1 0 0 link{i}.a; 0 1 0 0; 0 0 1 0; 0 0 0 1];
        RotX = [1 0 0 0; 0 cos(link{i}.alpha) -sin(link{i}.alpha) 0; 0 sin(link{i}.alpha) cos(link{i}.alpha) 0; 0 0 0 1];
    
        % Get Completed Homogeneous Transformation.
        HT{i} = RotZ * TransZ * TransX * RotX;
    end

    %% Get the relation HM from the base to the particular joint.
    HM50 = HT{1} * HT{2} * HT{3} * HT{4} * HT{5};
    HM40 = HT{1} * HT{2} * HT{3} * HT{4};
    HM30 = HT{1} * HT{2} * HT{3};
    HM20 = HT{1} * HT{2};
    HM10 = HT{1};

    %% Store Positions and HM's.
    HM = struct('HM50', HM50, 'HM40', HM40, 'HM30', HM30, 'HM20', HM20, 'HM10', HM10);

    origin = [0; 0; 0; 1];

    HMPos = struct('P5', (HM50 * origin), 'P4', (HM40 * origin), 'P3', (HM30 * origin), 'P2', (HM20 * origin), 'P1', (HM10 * origin), 'PBASE', origin);
end

