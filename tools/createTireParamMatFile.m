function createTireParamMatFile()
    % Define the name of the .mat file
    matFileName = 'A1654run21_MF52_Fy_GV12.mat';

    % Initialize a structure to hold the parameters
    tireParams = struct();

    % Parameters from the image and their default values (set to 1 if not found in .tir)
    % I will update these with actual values from your .tir data if they exist there.
    paramsList = {
        'FZ0', 1;
        % 'KY', 1; % Excluded as per request
        'LCX', 1;
        'LCY', 1;
        'LEX', 1;
        'LEY', 1;
        'LFZO', 1;
        'LGAY', 1;
        'LGAZ', 1;
        'LGYR', 1;
        'LHX', 1;
        'LHY', 1;
        'LKX', 1;
        'LKY', 1;
        'LMUX', 1;
        'LMUY', 1;
        'LRES', 1;
        'LS', 1;
        'LSGAL', 1;
        'LSGKP', 1;
        'LTR', 1;
        'LVX', 1;
        'LVY', 1;
        'LVYKA', 1;
        'LXAL', 1;
        'LYKA', 1;
    };

    % Parameters and their values extracted from your previously provided .tir file
    % (Only including the ones that match parameters in the `paramsList` above)
    tirValues = struct(...
        'FZ0', -1080, ...        % From [VERTICAL] FNOMIN
        'LFZO', 1, ...          % From [SCALING_COEFFICIENTS] LFZO
        'LCX', 1, ...           % From [SCALING_COEFFICIENTS] LCX
        'LMUX', 1, ...          % From [SCALING_COEFFICIENTS] LMUX
        'LEX', 1, ...           % From [SCALING_COEFFICIENTS] LEX
        'LKX', 1, ...           % From [SCALING_COEFFICIENTS] LKX
        'LHX', 1, ...           % From [SCALING_COEFFICIENTS] LHX
        'LVX', 1, ...           % From [SCALING_COEFFICIENTS] LVX
        'LCY', 1, ...           % From [SCALING_COEFFICIENTS] LCY
        'LMUY', 1, ...          % From [SCALING_COEFFICIENTS] LMUY
        'LEY', 1, ...           % From [SCALING_COEFFICIENTS] LEY
        'LKY', 1, ...           % From [SCALING_COEFFICIENTS] LKY
        'LHY', 1, ...           % From [SCALING_COEFFICIENTS] LHY
        'LVY', 1, ...           % From [SCALING_COEFFICIENTS] LVY
        'LTR', 1, ...           % From [SCALING_COEFFICIENTS] LTR
        'LRES', 1, ...          % From [SCALING_COEFFICIENTS] LRES
        'LXAL', 1, ...          % From [SCALING_COEFFICIENTS] LXAL
        'LYKA', 1, ...          % From [SCALING_COEFFICIENTS] LYKA
        'LVYKA', 1, ...         % From [SCALING_COEFFICIENTS] LVYKA
        'LS', 1 ...             % From [SCALING_COEFFICIENTS] LS
    );

    % Populate the tireParams structure
    for i = 1:size(paramsList, 1)
        paramName = paramsList{i, 1};
        
        % Check if the parameter exists in the .tir data
        if isfield(tirValues, paramName)
            tireParams.(paramName) = tirValues.(paramName);
        else
            tireParams.(paramName) = paramsList{i, 2}; % Use default (1)
        end
    end
    
    % Save the structure to a .mat file
    save(matFileName, '-struct', 'tireParams');

    fprintf('Successfully created %s with the specified parameters.\n', matFileName);
end