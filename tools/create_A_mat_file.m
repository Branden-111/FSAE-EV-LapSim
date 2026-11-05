function create_A_mat_file()
% Creates a .mat file named A1654run21_MF52_Fy_GV12.mat
% and saves an 18-element array 'A' to it.
%
% BEFORE RUNNING:
%   You must edit this file to input your 18 values in the section
%   labeled "--- User Input Section ---".
%
% Syntax:
%   create_A_mat_file()
%
% Input:
%   None. The data is hard-coded directly into this file.
%
% Output:
%   A .mat file named 'A1654run21_MF52_Fy_GV12.mat' is created in the
%   current MATLAB directory. This file contains a single variable 'A'
%   which holds the 18 values you entered below.
%
% Example Usage:
%   % 1. Edit the values for A(1) through A(18) in this file.
%   % 2. Save the file.
%   % 3. Run the function from the command window:
%   create_A_mat_file

% --- User Input Section ---
% vvv EDIT THE 18 VALUES FOR ARRAY 'A' BELOW vvv
A(1) = 3.2621;   % Replace 0 with your 1st value
A(2) = 1.866;   % Replace 0 with your 2nd value
A(3) = -0.18985;   % Replace 0 with your 3rd value
A(4) = 0.39754;   % Replace 0 with your 4th value
A(5) = 2.1903;   % Replace 0 with your 5th value
A(6) = 0.73547;   % Replace 0 with your 6th value
A(7) = -0.0029248;   % Replace 0 with your 7th value
A(8) = 0.12407;   % Replace 0 with your 8th value
A(9) = -39.3272;   % Replace 0 with your 9th value
A(10) = 0.98437;  % Replace 0 with your 10th value
A(11) = 1;  % Replace 0 with your 11th value
A(12) = -0.0013502;  % Replace 0 with your 12th value
A(13) = 9.2792e-05;  % Replace 0 with your 13th value
A(14) = -0.158150205404914;  % Replace 0 with your 14th value          Not in TIR file
A(15) = 0.033147;  % Replace 0 with your 15th value
A(16) = 0.0074118;  % Replace 0 with your 16th value
A(17) = -2.2851;  % Replace 0 with your 17th value
A(18) = -1.3212;  % Replace 0 with your 18th value
% ^^^ NO MORE EDITS NEEDED BELOW THIS LINE ^^^


% --- Input Validation ---
% Check if the number of elements in the A array is exactly 18.
if numel(A) ~= 18
    % If not, display an error message and stop the function.
    error('Error: The array ''A'' must contain exactly 18 values. Please check your inputs above.');
end

% --- File Saving ---
% Define the desired filename as a string.
filename = 'A1654run21_MF52_Fy_GV12.mat';

% Use the save function to create the .mat file.
% The first argument is the filename.
% The second argument is the name of the variable to save, in this case, 'A'.
save(filename, 'A');

% --- Confirmation Message ---
% Display a message in the command window to confirm that the file was created.
fprintf('Successfully created the file:\n%s\n', filename);
fprintf('It contains the variable ''A'' with your 18 values.\n');

end

