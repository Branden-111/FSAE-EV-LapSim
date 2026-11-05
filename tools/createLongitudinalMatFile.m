function createLongitudinalMatFile()
    % This script creates a .mat file containing a structure 'B' with all
    % the necessary longitudinal coefficients and scaling factors from your .tir file.

    fprintf('Creating longitudinal coefficient file...\n');

    % Create the structure
    B = struct();

    % --- From [VERTICAL] section ---
    B.FZ0 = 1080; % Corresponds to FNOMIN

    % --- From [SCALING_COEFFICIENTS] section ---
    B.LCX  = 1; B.LMUX = 1; B.LEX  = 1;
    B.LKX  = 1; B.LHX  = 1; B.LVX  = 1;

    % --- From [LONGITUDINAL_COEFFICIENTS] section ---
    B.PCX1 = 1.5;
    B.PDX1 = 1.735;
    B.PDX2 = -0.3562;
    B.PDX3 = 1.0737;
    B.PEX1 = -0.27625;
    B.PEX2 = 2.9999;
    B.PEX3 = -0.59661;
    B.PEX4 = 0.25961;
    B.PKX1 = 49.2261;
    B.PKX2 = 14.171;
    B.PKX3 = -0.26898;
    B.PHX1 = -0.001592;
    B.PHX2 = -0.0021039;
    B.PVX1 = 0.06169;
    B.PVX2 = -0.099294;
    B.RBX1 = 5;
    B.RBX2 = 5;
    B.RCX1 = 1;
    B.REX1 = -1;
    B.REX2 = -0.1;
    B.RHX1 = 0;

    % Save the structure 'B' into the .mat file
    save('A1654run21_MF52_Fx.mat', 'B');

    fprintf('Successfully created A1654run21_MF52_Fx.mat.\n');
end