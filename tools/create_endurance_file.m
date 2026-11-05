function create_endurance_file()
    % This function creates a .mat file named 'endurance_racing_line1.mat'.
    % The file will contain a 1x150 double array where each element is 0.5.

    % 1. Define the filename for the output .mat file.
    filename = 'endurance_racing_line1.mat';

    % 2. Create the data array: a 1-row by 150-column array of doubles.
    %    The ones(rows, cols) function creates an array of the specified size
    %    filled with the value 1.0. We then multiply by 0.5.
    endurance_data = 0.5 * ones(1, 150);

    % 3. Save the data to the .mat file.
    %    The 'save' function stores MATLAB variables on disk.
    %    The first argument is the filename.
    %    The second argument is the name of the variable to save (as a string).
    save(filename, 'endurance_data');

    % 4. (Optional) Display a confirmation message in the Command Window.
    fprintf('Successfully created the file: %s\n', filename);
    disp('It contains a 1x150 double array named "endurance_data" with all values set to 0.5.');

end