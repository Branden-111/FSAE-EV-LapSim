function Fx = MF52_Fx_fcn(coeffs, inputs)
% Calculates Longitudinal Force (Fx) using Pacejka Magic Formula 5.2.
%
% INPUTS:
%   coeffs: A structure containing all Fx model coefficients (e.g., coeffs.PCX1).
%   inputs: A matrix where each row is [kappa, Fz, gamma].
%           kappa = longitudinal slip ratio (-)
%           Fz    = vertical load (N)
%           gamma = inclination angle (deg)

    % --- Unpack Inputs ---
    kappa = inputs(:, 1);
    Fz    = abs(inputs(:, 2)); % Ensure Fz is positive for calculations
    gamma = inputs(:, 3);

    % --- Normalize Vertical Load ---
    dfz = (Fz - coeffs.FZ0) / coeffs.FZ0;

    % --- Magic Formula 5.2 Equations ---
    ZETA1 = 1; % Assuming pure slip conditions

    Cx = coeffs.PCX1 * coeffs.LCX;
    mux = (coeffs.PDX1 + coeffs.PDX2 * dfz) .* (1 - coeffs.PDX3 * gamma.^2) * coeffs.LMUX;
    Dx = mux .* Fz * ZETA1;
    Kx = Fz .* (coeffs.PKX1 + coeffs.PKX2 * dfz) .* exp(coeffs.PKX3 * dfz) * coeffs.LKX;
    
    % Handle potential division by zero
    Bx = zeros(size(Cx));
    nonZeroDenom = Cx .* Dx ~= 0;
    Bx(nonZeroDenom) = Kx(nonZeroDenom) ./ (Cx(nonZeroDenom) .* Dx(nonZeroDenom));
    
    Ex = (coeffs.PEX1 + coeffs.PEX2 * dfz + coeffs.PEX3 * dfz.^2) .* (1 - coeffs.PEX4 .* sign(kappa)) * coeffs.LEX;
    Shx = (coeffs.PHX1 + coeffs.PHX2 * dfz) * coeffs.LHX;
    Svx = Fz .* (coeffs.PVX1 + coeffs.PVX2 * dfz) * coeffs.LVX * coeffs.LMUX * ZETA1;
    kappax = kappa + Shx;

    % --- Core Formula Calculation ---
    Fx = Dx .* sin(Cx .* atan(Bx .* kappax - Ex .* (Bx .* kappax - atan(Bx .* kappax)))) + Svx;
end