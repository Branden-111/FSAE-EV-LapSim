% Lap_Sim_EV.m - Electric Formula Bharat lap-time and points simulation
% Branden Colaco (SPCE Racing Electric), 2025-2026
%
% DERIVED WORK. Based on Optimum-Mindstorm / Lap_Sim.m by Jonathan Vogel
% (Clemson Formula SAE, 2019 Michigan dynamic-event lap sim), GNU GPL v3.
% Upstream: https://github.com/OxxoCodes/Optimum-Mindstorm
% Modified for an electric single-ratio car, Pacejka MF5.2 Fx/Fy tires,
% Kari Motor Speedway tracks and Formula Bharat scoring. See CHANGES.md.

% Tradespace Analysis Project (upstream project name)

clear
close all
clc
% The purpose of this code is to evaluate the points-scoring capacity of a
% virtual electric vehicle around the Formula Bharat 2026 dynamic-event tracks.


%% Section 0: Name all symbolic variables
% Don't touch this. This is just naming a bunch of variables and making
% them global so that all the other functions can access them
global r_max accel grip deccel lateral cornering gear shift_points...
    top_speed r_min path_boundaries tire_radius shift_time...
    powertrainpackage track_width path_boundaries_ax
%% Section 1: Input Tire Model
% this section is required, everything should be pre-loaded so no need to
% touch any of this, unless you want to change the tire being evaluated.
% The only things you might want to change are the scaling factors at the
% bottom of the section
%disp('2019 Michigan Endurance Points Analysis')
disp('Loading Tire Model')

% First we load in the lateral tire force model, which is a Pacejka model
% created by derek:
global FZ0 LFZO LCX LMUX LEX LKX  LHX LVX LCY LMUY LEY LKY LHY LVY ...
       LGAY Ltr LRES LGAZ LXAL LYKA LVYKA LS LSGKP  LSGAL LGYR KY
load('A1654run21_MF52_Fy_GV12.mat')
% then load in coefficients for Magic Formula 5.2 Tire Model:
load('A1654run21_MF52_Fy_12.mat')

% Next you load in the longitudinal tire model, which for now is just a
% CSAPS spline fit to the TTC data
% find your pathname and filename for the tire you want to load in
%filename = 'Hoosier_R25B_18.0x7.5-10_FX_12psi.mat';
%load(filename)
%load('A1654run21_MF52_Fx.mat');

load('A1654run21_MF52_Fx.mat'); % Pacejka MF5.2 longitudinal coefficients, structure 'B'

tire_radius = 9.05/12; %ft
tyreRadius = 0.2286;%tire_radius/3.28; % converts to meters

% finally, we have some scaling factors for longitudinal (x) and lateral
% (y) friction. You can use these to tune the lap sim to correlate better 
% to logged data 
sf_x = .6;
sf_y = .47;
%sf_y = 0.2248 * 0.67;
%% Section 2: Input Powertrain Model
% change whatever you want here, this is the 2018 powertrain package iirc
% just keep your units consistent please
disp('Loading Engine Model')
%engineSpeed = [6200:100:14100]; % 14100 RPM
engineSpeed = [...
364, 431, 492, 519, 532, 553, 613, 647, 681, 728, 788, 856, 937, 1020, ...
1060, 1100, 1150, 1190, 1220, 1270, 1300, 1340, 1370, 1380, 1400, 1420, 1430, ...
1450, 1460, 1480, 1490, 1500, 1520, 1540, 1580, 1620, 1650, 1680, 1720, 1750, ...
1800, 1850, 1900, 1970, 2010, 2070, 2120, 2180, 2220, 2300, 2330, 2370, 2420, ...
2490, 2540, 2590, 2650, 2690, 2720, 2750, 2780, 2820, 2870, 2910, 2940, 2980, ...
3010, 3030, 3080, 3120, 3170, 3210, 3250, 3270, 3340, 3400, 3480, 3540, 3610, ...
3670, 3730, 3770, 3810, 3910, 3960, 3980, 4000, 4010, 4020, 4040, 4060, 4070, ...
4090, 4110, 4130, 4150, 4160, 4180, 4190, 4200, 4220, 4230, 4250, 4260, 4270, ...
4280, 4290, 4310, 4320, 4330, 4350, 4360, 4370, 4390, 4400, 4410, 4430, 4450, ...
4470, 4480, 4490, 4510, 4520, 4540, 4550, 4560, 4580, 4590, 4600, 4620, 4640, ...
4660, 4680, 4690, 4700, 4720, 4740, 4760, 4780, 4800, 4820, 4840, 4850, 4860, ...
4870, 4890, 4900, 4910, 4930, 4940, 4950, 4970, 4990, 5010, 5030, 5060, 5080, ...
5100, 5120, 5150, 5190, 5220, 5260, 5300, 5350, 5400, 5450, 5480, 5510, 5550, ...
5580, 5610, 5650, 5690, 5740, 5770, 5820, 5870, 5900, 5940, 5970, 6000, 6040, ...
6100, 6160, 6190, 6240, 6290, 6330, 6390, 6450, 6490, 6520, 6560, 6620, 6660, ...
6700, 6750, 6790, 6830, 6870, 6930, 6970, 7010, 7040, 7070, 7100, 7140, 7190, ...
7240, 7300, 7500];
% torque should be in N-m:
%engineTq = [41.57 42.98 44.43 45.65 46.44 47.09 47.52 48.58 49.57 50.41 51.43 51.48 51 49.311 48.94 48.66 49.62 49.60 47.89 47.91 48.09 48.57 49.07 49.31 49.58 49.56 49.84 50.10 50.00 50.00 50.75 51.25 52.01 52.44 52.59 52.73 53.34 53.72 52.11 52.25 51.66 50.5 50.34 50.50 50.50 50.55 50.63 50.17 50.80 49.73 49.35 49.11 48.65 48.28 48.28 47.99 47.68 47.43 47.07 46.67 45.49 45.37 44.67 43.8 43.0 42.3 42.00 41.96 41.70 40.43 39.83 38.60 38.46 37.56 36.34 35.35 33.75 33.54 32.63 31.63];
engineTq = [...
80.70, 80.70, 80.20, 79.70, 79.50, 79.40, 79.40, 79.40, 79.40, 79.40, 79.40, ...
79.40, 79.40, 79.40, 79.40, 79.40, 79.40, 79.40, 79.40, 79.40, 79.40, 79.40, ...
79.40, 79.40, 79.20, 79.10, 78.90, 78.80, 78.70, 78.50, 78.40, 78.20, 78.20, ...
78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, ...
78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, ...
78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, ...
78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, 78.20, ...
78.20, 78.20, 78.20, 78.20, 77.90, 77.70, 77.50, 77.40, 77.20, 76.90, 76.80, ...
76.70, 76.40, 76.10, 75.80, 75.50, 75.20, 74.90, 74.60, 74.40, 74.10, 73.80, ...
73.30, 73.10, 72.80, 72.50, 72.10, 71.60, 71.30, 71.10, 70.60, 70.30, 70.00, ...
69.80, 69.50, 69.00, 68.80, 68.20, 67.80, 67.30, 67.00, 66.70, 66.50, 66.20, ...
65.90, 65.60, 65.20, 64.90, 64.60, 64.20, 63.70, 63.30, 62.70, 62.40, 62.20, ...
61.70, 61.30, 60.90, 60.30, 59.90, 59.30, 59.00, 58.70, 58.40, 58.10, 57.80, ...
57.60, 57.30, 57.00, 56.70, 56.40, 56.10, 55.80, 55.60, 55.30, 54.80, 54.40, ...
54.00, 53.50, 52.80, 52.00, 51.20, 50.40, 49.40, 48.10, 46.90, 46.20, 45.90, ...
45.50, 44.90, 44.50, 43.90, 43.30, 42.60, 41.90, 41.30, 40.60, 39.80, 39.20, ...
38.60, 38.20, 37.80, 37.30, 36.60, 35.90, 35.50, 34.90, 34.30, 33.90, 33.30, ...
32.70, 32.30, 31.90, 31.60, 30.90, 30.40, 29.90, 29.30, 28.90, 28.30, 27.80, ...
27.30, 26.80, 26.60, 26.30, 26.10, 25.80, 25.70, 25.30, 24.80, 24.40, 24.00, ...
23.00, 22.40, 21.40];
primaryReduction = 1;%76/36;
gear = [8.5];%[33/12, 32/16, 30/18, 26/18, 30/23, 29/24]; % transmission gear ratios
finalDrive = 1;%40/12; % large sprocket/small sprocket
shiftpoint = 14000; % optimal shiftpoint for most gears [RPM]
drivetrainLosses = .85; % percent of torque that makes it to the rear wheels 
shift_time = .25; % seconds
T_lock = 100; % differential locking torque (0 =  open, 1 = locked)

track_width = 9.842;

% Intermediary Calcs/Save your results into the workspace
gearTot = gear(end)*finalDrive*primaryReduction;
VMAX = floor(3.28*shiftpoint/(gearTot/tyreRadius*60/(2*pi)));
T_lock = T_lock/100;
powertrainpackage = {engineSpeed engineTq primaryReduction gear finalDrive shiftpoint drivetrainLosses};
%% Section 3: Vehicle Architecture
disp('Loading Vehicle Characteristics')
% These are the basic vehicle architecture primary inputs:
LLTD = 54.56;%51.5; % Front lateral load transfer distribution (%)
W = 738.549;%660; % vehicle + driver weight (lbs)
WDF = 48.51;%50; % front weight distribution (%)
cg = 0.92290;%13.2/12; % center of gravity height (ft)
l = 5.151;%60.5/12; % wheelbase (ft)
twf = 4.078;%46/12; % front track width (ft)
twr = 3.937008;%44/12; % rear track width (ft)

% some intermediary calcs you don't have to touch
LLTD = LLTD/100;
WDF = WDF/100;
m = W/32.2; % mass (lbm)
WF = W*WDF; % front weight
WR = W*(1-WDF); % rear weight
a = l*(1-WDF); % front axle to cg
b = l*WDF; % rear axle to cg
tw = twf;
%% Section 4: Input Suspension Kinematics
disp('Loading Suspension Kinematics')
% this section is actually optional. So if you set everything to zero, you
% can essentially leave this portion out of the analysis. Useful if you are
% only trying to explore some higher level relationships

% Pitch and roll gradients define how much the car's gonna move around
rg_f = 0.4268; % front roll gradient (deg/g)
rg_r = 0.6402; % rear roll gradient (deg/g)
pg = 0.597; % pitch gradient (deg/g)
WRF = 227.6; % front and rear ride rates (lbs/in)
WRR = 145.2; 

% then you can select your camber alignment
IA_staticf = 0;%-2; % front static camber angle (deg)
IA_staticr = 0;%-2; % rear static camber angle (deg)
IA_compensationf = -2.390; % front camber compensation (%)
IA_compensationr = -0.318; % rear camber compensation (%)

% lastly you can select your kingpin axis parameters
casterf = 2.03; % front caster angle (deg)
KPIf = 7.87;%0; % front kingpin inclination angle (deg)
casterr = 2.06;
KPIr = 7.12;%0;

% intermediary calcs, plz ignore
IA_staticf = deg2rad(IA_staticf); % front static camber angle (deg)
IA_staticr = deg2rad(IA_staticr); % rear static camber angle (deg)
IA_compensationf = IA_compensationf/100; % front camber compensation (%)
IA_compensationr = IA_compensationr/100; % rear camber compensation (%)
casterf = deg2rad(casterf);
KPIf = deg2rad(KPIf);
casterr = deg2rad(casterr);
KPIr = deg2rad(KPIr);
IA_roll_inducedf = asin(2/twf/12);
IA_roll_inducedr = asin(2/twr/12);
IA_gainf = IA_roll_inducedf*IA_compensationf;
IA_gainr = IA_roll_inducedr*IA_compensationr;
%% Section 5: Input Aero Parameters
disp('Loading Aero Model')
Cl = .0418; %279/418
Cd = .0184; % .0245
CoP = 48.51;%48; % front downforce distribution (%)

% Intermediary Calculations
CoP = CoP/100;
%% Section 6: Generate GGV Diagram
% this is where the m e a t of the lap sim takes place. The GGV diagram is
% built by finding a maximum cornering, braking, and acceleration capacity
% for any given speed
bran = 0;
disp('Generating g-g-V Diagram')

deltar = 0;
deltaf = 0;
velocity = 1:5:90; % range of velocities at which sim will evaluate (ft/s)
radii = [13:10:155]; % range of turn radii at which sim will evaluate (ft)

% First we will evaluate our Acceleration Capacity
g = 1; % g is a gear indicator, and it will start at 1
spcount = 1; % spcount is keeping track of how many gearshifts there are
% shift_points tracks the actual shift point velocities
shift_points(1) = 0; 
disp('     Acceleration Envelope')
for  i = 1:1:length(velocity) % for each velocity
    gp = g; % Current gear = current gear (wow!)
    V = velocity(i); % find velocity
    DF = Cl*V^2; % calculate downforce (lbs)
    % calculate f/r suspension drop from downforce (in)
    dxf = DF*CoP/2/WRF; 
    dxr = DF*(1-CoP)/2/WRR;
    % from rh drop, find camber gain (deg)
    IA_0f = IA_staticf - dxf*IA_gainf;
    IA_0r = IA_staticr - dxr*IA_gainr;
    % find load on each tire (lbs)
    wf = (WF+DF*CoP)/2;
    wr = (WR+DF*(1-CoP))/2;
    
    % now we actually sweep through with acceleration
    Ax = 0; % starting guess of zero g's
    WS = W/2; % weight of one half-car
    pitch = -Ax*pg*pi/180; % pitch angle (rad)
    % recalculate wheel loads due to load transfer (lbs)
    wf = wf-Ax*cg*WS/l; 
    wr = wr+Ax*cg*WS/l;
    % recalculate camber angles due to pitch
    IA_f = -l*12*sin(pitch)/2*IA_gainf + IA_0f;
    IA_r = l*12*sin(pitch)/2*IA_gainr + IA_0r;
    % select a range of slip ratios (sl) [-]
    sl = [0:.01:.11];
    % evaluate the tractive force capacity from each tire for the range of
    % slip ratios
    for k = 1:length(sl)  
        fxf(k) = MF52_Fx_fcn(B, [sl(k), -wf, rad2deg(-IA_f)]) * sf_x;
        fxr(k) = MF52_Fx_fcn(B, [sl(k), -wr, rad2deg(-IA_r)]) * sf_x;
    end
    % find max force capacity from each tire:
    fxf(find(abs(fxf) > 1000)) = [];
    fxr(find(abs(fxr) > 1000)) = [];
    FXF = max(fxf);
    FXR = max(fxr);
    % Calculate total tire tractive force (lbs)
    FX = abs(2*FXR);
    % calculate total lateral acceleration capacity (g's)
    AX = FX/W;
    AX_diff = AX-Ax;
    while AX_diff>0
        %disp([Ax AX])
        Ax = Ax+.01;
        WS = W/2;
        pitch = -Ax*pg*pi/180;
        wf = (WF+DF*CoP)/2;
        wr = (WR+DF*(1-CoP))/2;
        wf = wf-Ax*cg*WS/l/24;
        wr = wr+Ax*cg*WS/l/24;
        IA_f = -l*12*sin(pitch)/2*IA_gainf + IA_0f;% - KPIf*(1-cos(deltaf)) + casterf*sin(deltaf);
        IA_r = l*12*sin(pitch)/2*IA_gainr + IA_0r;% - KPIr*(1-cos(deltar)) + casterf*sin(deltar);
        FZ_vals = [-250:1:-50];
        sl = [0:.01:.11];
        for k = 1:length(sl)
            fxf(k) = MF52_Fx_fcn(B, [sl(k), -wf, rad2deg(-IA_f)]) * sf_x;
            fxr(k) = MF52_Fx_fcn(B, [sl(k), -wr, rad2deg(-IA_r)]) * sf_x;
        end
        fxf(find(abs(fxf) > 1000)) = [];
        fxr(find(abs(fxr) > 1000)) = [];
        FXF = max(fxf);
        FXR = max(fxr);
        FX = abs(2*FXR);
        AX = FX/W;
        AX_diff = AX-Ax;
    end
    A_xr(i) = AX;
    output = Powertrainlapsim(max(7.5,V/3.28)); % 7.5 reg, 10 launch
    FX = output(1)*.2248;
    FX = FX-Cd*V^2;
    fx(i) = FX/W;
    AX(i) = min(FX/W,A_xr(i));
    output = Powertrainlapsim(V/3.28);
    g = output(2);
    gear(i) = g;
    if g>gp
        spcount = spcount+1;
        shift_points(spcount) = V;
    end
    A_Xr(i) = AX(i);
end
A_Xr(A_Xr < 0) = 0;

% from these results, you can create the first part of the GGV diagram
% input for the lap sim codes:
% accel is the maximum acceleration capacity as a function of velocity
% (power limited) and grip is the same but (tire limited)
accel = csaps(velocity,A_Xr);
grip = csaps(velocity,A_xr);

% Next we explore the cornering envelope. First we define AYP, which is the
% starting guess for lateral acceleration capacity at a given speed
AYP = 1;
disp('     Cornering Envelope')
% for cornering performance, it makes more sense to evaluate a set of
% cornering radii, instead of speeds
for turn = 1:1:length(radii)
    bran = 0;
    turn
    % first define your vehicle characteristics:
        a = l*(1-WDF);
        b = l*WDF;
        R = radii(turn);
        % update speed and downforce
        V = sqrt(R*32.2*AYP);
        DF = Cl*V^2; 
        % from downforce, update suspension travel (in):
        dxf = DF*CoP/2/WRF; 
        dxr = DF*(1-CoP)/2/WRR; 
        % from suspension heave, update static camber (rad):
        IA_0f = IA_staticf - dxf*IA_gainf; 
        IA_0r = IA_staticr - dxr*IA_gainr; 
        % update load on each axle (lbs)
        wf = (WF+DF*CoP)/2;
        wr = (WR+DF*(1-CoP))/2;
        % guess ackermann steer angle as starting steer angle
        % delta = l/R;
        % ddelta = delta*.01;
        a_i = 85.1020;
        p_i = 0.94714;
        c_i = 0.10365;
        theta_i = deg2rad(0.058966 + (a_i ./ (R - c_i)).^(1/p_i));
        a_o = 90.2279;
        p_o = 0.99910;
        theta_o = deg2rad(0.0017074 + (a_o ./ (R +0.12316)).^(1/p_o));
        dthetai = theta_i*0.01;
        dthetao = theta_o*0.01;
        %delta = (theta_i+theta_o)/2
        %ddelta = delta*.01;
        % assume vehicle sideslip starts at 0 (rad)
        beta = deg2rad(0);
        A_y = V^2/R;
        % calculate lateral load transfer (lbs)
        WT = A_y*cg*W/mean([twf twr])/32.2/12;
        % split f/r using LLTD
        WTF = WT*LLTD;
        WTR = WT*(1-LLTD);
        % calculate f/r roll (rad)
        phif = A_y*rg_f*pi/180/32.2;
        phir = A_y*rg_r*pi/180/32.2;
        % update individual wheel loads 
        wfin = wf-WTF;
        wfout = wf+WTF;
        wrin = wr-WTR;
        wrout = wr+WTR;
        % update individual wheel camber (from roll, then from steer
        % effects)
        IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
        IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
        IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
        IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir;
        % calculate yaw rate
        r = A_y/V;
        % from yaw, sideslip and steer you can get slip angles
        a_fi = beta+a*r/V-theta_i;
        a_fo = beta+a*r/V-theta_o;
        a_r = beta-b*r/V;
        % with slip angles, load and camber, calculate lateral force at
        % the front
        F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
        F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
        % before you calculate the rears, you ned to see what the diff is
        % doing
        % calculate the drag from aero and the front tires
        % Calculate induced drag for each front wheel separately
        F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
        F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
        
        % Sum aero drag and the two induced drag components
        F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
        % calculate the grip penalty assuming the rears must overcome that
        % drag
        rscale = 1-(F_x/W/fnval(grip,V))^2;
        % now calculate rear tire forces, with said penalty
        F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
        F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
        % sum of forces and moments
        F_y = F_fin+F_fout+F_rin+F_rout;
        M_z_diff = F_x*T_lock*twr/2; % incl the differential contribution     
        M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
        % calculate resultant lateral acceleration
        AY = F_y/(W/32.2);
        % compare to the initial guess
        diff_AY = A_y-AY;
        diff_AY;
        % vary the sideslip angle (B) until the initial guess and resultant
        % AY match up
        while diff_AY < 0
            beta = beta + .0025;
            A_y = V^2/R;
            WT = A_y*cg*W/mean([twf twr])/32.2/12;
            WTF = WT*LLTD;
            WTR = WT*(1-LLTD);
            phif = A_y*rg_f*pi/180/32.2;
            phir = A_y*rg_r*pi/180/32.2;
            wfin = wf-WTF;
            wfout = wf+WTF;
            wrin = wr-WTR;
            wrout = wr+WTR;
            IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
            IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
            IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
            IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
            r = A_y/V;
            a_fi = beta+a*r/V-theta_i;
            a_fo = beta+a*r/V-theta_o;
            a_r = beta-b*r/V;
            F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
            F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
            % Calculate induced drag for each front wheel separately
            F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
            F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
            
            % Sum aero drag and the two induced drag components
            F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
            rscale = 1-(F_x/W/fnval(grip,V))^2;
            F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
            F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
            F_y = F_fin+F_fout+F_rin+F_rout;
            M_z_diff = F_x*T_lock*twr/2;       
            M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
            AY = F_y/(W/32.2);
            diff_AY = A_y-AY; 
        end
        while diff_AY > 0
            beta = beta - .0025;
            A_y = V^2/R;
            WT = A_y*cg*W/mean([twf twr])/32.2/12;
            WTF = WT*LLTD;
            WTR = WT*(1-LLTD);
            phif = A_y*rg_f*pi/180/32.2;
            phir = A_y*rg_r*pi/180/32.2;
            wfin = wf-WTF;
            wfout = wf+WTF;
            wrin = wr-WTR;
            wrout = wr+WTR;
            IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
            IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
            IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
            IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
            r = A_y/V;
            a_fi = beta+a*r/V-theta_i;
            a_fo = beta+a*r/V-theta_o;
            a_r = beta-b*r/V;
            F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
            F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
            % Calculate induced drag for each front wheel separately
            F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
            F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
            
            % Sum aero drag and the two induced drag components
            F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
            rscale = 1-(F_x/W/fnval(grip,V))^2;
            F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
            F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
            F_y = F_fin+F_fout+F_rin+F_rout;
            M_z_diff = F_x*T_lock*twr/2;          
            M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
            AY = F_y/(W/32.2);
            diff_AY = A_y-AY; 
            bran = bran + 1;
            if bran > 1000
                %diff_AY = 0;
                break;
            end
        end
        % at that point, check the yaw moment. Re-run the above loop^ but
        % this time, steer angle is being varied until moment comes out to
        % zero-ish:
        while M_z < 0 
            %delta = delta+ddelta;
            theta_i = theta_i + dthetai;
            theta_o = theta_o + dthetao;
            beta = deg2rad(0);
            A_y = V^2/R;
            WT = A_y*cg*W/mean([twf twr])/32.2/12;
            WTF = WT*LLTD;
            WTR = WT*(1-LLTD);
            phif = A_y*rg_f*pi/180/32.2;
            phir = A_y*rg_r*pi/180/32.2;
            wfin = wf-WTF;
            wfout = wf+WTF;
            wrin = wr-WTR;
            wrout = wr+WTR;
            IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
            IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
            IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
            IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
            r = A_y/V;
            a_fi = beta+a*r/V-theta_i;
            a_fo = beta+a*r/V-theta_o;
            a_r = beta-b*r/V;
            F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
            F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
            % Calculate induced drag for each front wheel separately
            F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
            F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
            
            % Sum aero drag and the two induced drag components
            F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
            rscale = 1-(F_x/W/fnval(grip,V))^2;
            F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
            F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
            F_y = F_fin+F_fout+F_rin+F_rout;
            M_z_diff = F_x*T_lock*twr/2;      
            M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
            AY = F_y/(W/32.2);
            diff_AY = A_y-AY;
            while diff_AY < 0
                beta = beta + .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
            while diff_AY > 0
                beta = beta - .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
                bran = bran + 1;
                if bran > 2000
                    %diff_AY = 0;
                    break;
                end
            end
        end
        while M_z > 0 
            %delta = delta-ddelta;
            theta_i = theta_i - dthetai;
            theta_o = theta_o - dthetao;
            beta = deg2rad(0);
            A_y = V^2/R;
            WT = A_y*cg*W/mean([twf twr])/32.2/12;
            WTF = WT*LLTD;
            WTR = WT*(1-LLTD);
            phif = A_y*rg_f*pi/180/32.2;
            phir = A_y*rg_r*pi/180/32.2;
            wfin = wf-WTF;
            wfout = wf+WTF;
            wrin = wr-WTR;
            wrout = wr+WTR;
            IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
            IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
            IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
            IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
            r = A_y/V;
            a_fi = beta+a*r/V-theta_i;
            a_fo = beta+a*r/V-theta_o;
            a_r = beta-b*r/V;
            F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
            F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
            % Calculate induced drag for each front wheel separately
            F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
            F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
            
            % Sum aero drag and the two induced drag components
            F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
            rscale = 1-(F_x/W/fnval(grip,V))^2;
            F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
            F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
            F_y = F_fin+F_fout+F_rin+F_rout;
            M_z_diff = F_x*T_lock*twr/2; 
            M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
            AY = F_y/(W/32.2);
            diff_AY = A_y-AY;
            while diff_AY < 0
                beta = beta + .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
            while diff_AY > 0
                beta = beta - .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
                bran = bran + 1;
                if bran > 5000
                    %diff_AY = 0;
                    break
                end
            end
        end
        % then re run all of THAT, slowly increasing your AY guess 
        % until the front tire maxes out aka slip
        % angle of 12
        while a_fo > deg2rad(-12)
            AYP = AYP+.005;
            a = l*(1-WDF);
            b = l*WDF;
            R = radii(turn);
            V = sqrt(R*32.2*AYP);
            DF = Cl*V^2; 
            dxf = DF*CoP/2/WRF; 
            dxr = DF*(1-CoP)/2/WRR; 
            IA_0f = IA_staticf - dxf*IA_gainf; 
            IA_0r = IA_staticr - dxr*IA_gainr; 
            wf = (WF+DF*CoP)/2;
            wr = (WR+DF*(1-CoP))/2;
            a_i = 85.1020;
            p_i = 0.94714;
            c_i = 0.10365;
            theta_i = deg2rad(0.058966 + (a_i ./ (R - c_i)).^(1/p_i));
            a_o = 90.2279;
            p_o = 0.99910;
            theta_o = deg2rad(0.0017074 + (a_o ./ (R +0.12316)).^(1/p_o));
            %delta = (theta_i+theta_o)/2;
            %ddelta = delta*.005;
            dthetai = theta_i*0.005;
            dthetao = theta_o*0.005;
            beta = deg2rad(0);
            A_y = V^2/R;
            WT = A_y*cg*W/mean([twf twr])/32.2/12;
            WTF = WT*LLTD;
            WTR = WT*(1-LLTD);
            phif = A_y*rg_f*pi/180/32.2;
            phir = A_y*rg_r*pi/180/32.2;
            wfin = wf-WTF;
            wfout = wf+WTF;
            wrin = wr-WTR;
            wrout = wr+WTR;
            IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
            IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
            IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
            IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
            r = A_y/V;
            a_fi = beta+a*r/V-theta_i;
            a_fo = beta+a*r/V-theta_o;
            a_r = beta-b*r/V;
            F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
            F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
            % Calculate induced drag for each front wheel separately
            F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
            F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
            
            % Sum aero drag and the two induced drag components
            F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
            rscale = 1-(F_x/W/fnval(grip,V))^2;
            F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
            F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
            F_y = F_fin+F_fout+F_rin+F_rout;
            M_z_diff = F_x*T_lock*twr/2; 
            M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
            AY = F_y/(W/32.2);
            diff_AY = A_y-AY;
            while diff_AY < 0
                beta = beta + .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
            while diff_AY > 0
                beta = beta - .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
            %rad2deg([a_f a_r])
            while M_z > 0 
                %delta = delta-ddelta;
                theta_i = theta_i - dthetai;
                theta_o = theta_o - dthetao;
                beta = deg2rad(0);
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY;
                while diff_AY < 0
                    beta = beta + .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_fi = beta+a*r/V-theta_i;
                    a_fo = beta+a*r/V-theta_o;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                    % Calculate induced drag for each front wheel separately
                    F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                    F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                    
                    % Sum aero drag and the two induced drag components
                    F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                    rscale = 1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
                while diff_AY > 0
                    beta = beta - .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_fi = beta+a*r/V-theta_i;
                    a_fo = beta+a*r/V-theta_o;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                    % Calculate induced drag for each front wheel separately
                    F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                    F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                    
                    % Sum aero drag and the two induced drag components
                    F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                    rscale = 1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
            end
            while M_z < 0 
                %delta = delta+ddelta;
                theta_i = theta_i + dthetai;
                theta_o = theta_o + dthetao;
                beta = deg2rad(0);
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY;
                while diff_AY < 0
                    beta = beta + .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_fi = beta+a*r/V-theta_i;
                    a_fo = beta+a*r/V-theta_o;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                    % Calculate induced drag for each front wheel separately
                    F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                    F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                    
                    % Sum aero drag and the two induced drag components
                    F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out; 
                    rscale = 1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
                while diff_AY > 0
                    beta = beta - .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_fi = beta+a*r/V-theta_i;
                    a_fo = beta+a*r/V-theta_o;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                    % Calculate induced drag for each front wheel separately
                    F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                    F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                    
                    % Sum aero drag and the two induced drag components
                    F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                    rscale = 1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                    if a_fi < deg2rad(-12)
                        diff_AY = -1;
                    end
                end
                if a_fi < deg2rad(-12)
                    M_z = 1;
                end
            end
        end
        % once you've exceeded the capability of the fronts, take one small
        % step back and that is your max lateral acceleration capacity
        AYP = AYP-.005;
        a = l*(1-WDF);
        b = l*WDF;
        R = radii(turn);
        V = sqrt(R*32.2*AYP);
        DF = Cl*V^2; 
        dxf = DF*CoP/2/WRF; 
        dxr = DF*(1-CoP)/2/WRR; 
        IA_0f = IA_staticf - dxf*IA_gainf; 
        IA_0r = IA_staticr - dxr*IA_gainr; 
        wf = (WF+DF*CoP)/2;
        wr = (WR+DF*(1-CoP))/2;
        a_i = 85.1020;
        p_i = 0.94714;
        c_i = 0.10365;
        theta_i = deg2rad(0.058966 + (a_i ./ (R - c_i)).^(1/p_i));
        a_o = 90.2279;
        p_o = 0.99910;
        theta_o = deg2rad(0.0017074 + (a_o ./ (R +0.12316)).^(1/p_o));
        dthetai = theta_i*0.01;
        dthetao = theta_o*0.01;
        %delta = (theta_i+theta_o)/2;
        %ddelta = delta*.01;
        beta = deg2rad(0);
        A_y = V^2/R;
        WT = A_y*cg*W/mean([twf twr])/32.2/12;
        WTF = WT*LLTD;
        WTR = WT*(1-LLTD);
        phif = A_y*rg_f*pi/180/32.2;
        phir = A_y*rg_r*pi/180/32.2;
        wfin = wf-WTF;
        wfout = wf+WTF;
        wrin = wr-WTR;
        wrout = wr+WTR;
        IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
        IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
        IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
        IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
        r = A_y/V;
        a_fi = beta+a*r/V-theta_i;
        a_fo = beta+a*r/V-theta_o;
        a_r = beta-b*r/V;
        F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
        F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
        % Calculate induced drag for each front wheel separately
        F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
        F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
        
        % Sum aero drag and the two induced drag components
        F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
        rscale = 1-(F_x/W/fnval(grip,V))^2;
        F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
        F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
        F_y = F_fin+F_fout+F_rin+F_rout;
        M_z_diff = F_x*T_lock*twr/2; 
        M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
        AY = F_y/(W/32.2);
        diff_AY = A_y-AY;
        while diff_AY < 0
                beta = beta + .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
        while diff_AY > 0
                beta = beta - .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
                bran = bran + 1;
                if bran > 6000
                    %diff_AY = 0;
                    break;
                end
            end
        %rad2deg([a_f a_r])
        while M_z < 0 
                %delta = delta+ddelta;
                theta_i = theta_i + dthetai;
                theta_o = theta_o + dthetao;
                beta = deg2rad(0);
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY;
                while diff_AY < 0
                    beta = beta + .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_fi = beta+a*r/V-theta_i;
                    a_fo = beta+a*r/V-theta_o;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                    % Calculate induced drag for each front wheel separately
                    F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                    F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                    
                    % Sum aero drag and the two induced drag components
                    F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                    rscale = 1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
                while diff_AY > 0
                    beta = beta - .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_fi = beta+a*r/V-theta_i;
                    a_fo = beta+a*r/V-theta_o;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                    % Calculate induced drag for each front wheel separately
                    F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                    F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                    
                    % Sum aero drag and the two induced drag components
                    F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                    rscale = 1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2;     
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                    if a_fi < deg2rad(-12)
                        diff_AY = -1;
                    end
                end
            end
        while M_z > 0 
                %delta = delta-ddelta;
                theta_i = theta_i - dthetai;
                theta_o = theta_o - dthetao;
                beta = deg2rad(0);
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_fi = beta+a*r/V-theta_i;
                a_fo = beta+a*r/V-theta_o;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                % Calculate induced drag for each front wheel separately
                F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                
                % Sum aero drag and the two induced drag components
                F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                rscale = 1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY;
                while diff_AY < 0
                    beta = beta + .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_fi = beta+a*r/V-theta_i;
                    a_fo = beta+a*r/V-theta_o;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                    % Calculate induced drag for each front wheel separately
                    F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                    F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                    
                    % Sum aero drag and the two induced drag components
                    F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                    rscale = 1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
                while diff_AY > 0
                    beta = beta - .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(theta_i)) - casterf*sin(theta_i) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(theta_o)) - casterf*sin(theta_o) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_fi = beta+a*r/V-theta_i;
                    a_fo = beta+a*r/V-theta_o;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_fi) wfin -rad2deg(IA_f_in)])*sf_y*cos(theta_i);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_fo) wfout -rad2deg(IA_f_out)])*sf_y*cos(theta_o);
                    % Calculate induced drag for each front wheel separately
                    F_x_induced_in = F_fin * (sin(theta_i) / cos(theta_i));
                    F_x_induced_out = F_fout * (sin(theta_o) / cos(theta_o));
                    
                    % Sum aero drag and the two induced drag components
                    F_x = Cd*V^2 + F_x_induced_in + F_x_induced_out;
                    rscale = 1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                    bran = bran + 1;
                if bran > 7000
                    %diff_AY = 0;
                    break;
                end
                end
        end
        % --- CHANGE IT TO THIS ---
        beta_deg = rad2deg(beta);
        afi = rad2deg(a_fi);
        afo = rad2deg(a_fo);
        ar = rad2deg(a_r);
        steer = rad2deg((theta_i+theta_o)/2);
        % UG = rad2deg(delta-l/R)*32.2/AY;
        % Ugradient(turn) = UG;
        % --- ADD THESE LINES ---
        % slip_angle_f(turn) = af;
        % slip_angle_r(turn) = ar;
        % sideslip_angle(turn) = beta_deg;
        % -----------------------
        %F_lat = fnval([rad2deg(a_f);-wf;0],full_send_y)*.45*cos(delta);
        %F_drag = fnval([rad2deg(a_f);-wf;0],full_send_y)*.45*sin(delta);
        skid = 2*pi*R/V;
        steering(turn) = steer;
        speed(turn) = V;
        lateralg(turn) = AY/32.2;
        %toc
end
% Lateral Acceleration

% Braking Performance
velocity = 15:5:80;
disp('     Braking Envelope')
% the braking sim works exactly the same as acceleration, except now all 4
% tires are contributing to the total braking capacity
for  i = 1:1:length(velocity)
    V = velocity(i);
    DF = Cl*V^2;
    dxf = DF*CoP/2/WRF;
    dxr = DF*(1-CoP)/2/WRR;
    IA_0f = IA_staticf - dxf*IA_gainf;
    IA_0r = IA_staticr - dxr*IA_gainr;
    wf = (WF+DF*CoP)/2;
    wr = (WR+DF*(1-CoP))/2;
    Ax = 1;
    WS = W/2;
    pitch = Ax*pg*pi/180;
    wf = wf+Ax*cg*WS/l/24;
    wr = wr-Ax*cg*WS/l/24;
    IA_f = -l*12*sin(pitch)/2*IA_gainf + IA_0f;% - KPIf*(1-cos(deltaf)) + casterf*sin(deltaf);
    IA_r = l*12*sin(pitch)/2*IA_gainr + IA_0r;% - KPIr*(1-cos(deltar)) + casterf*sin(deltar);
    FZ_vals = [-250:1:-50];
    sl = [-.15:.01:0];
    for k = 1:length(sl)
        fxf(k) = MF52_Fx_fcn(B, [sl(k), -wf, rad2deg(-IA_f)]) * sf_x;
        fxr(k) = MF52_Fx_fcn(B, [sl(k), -wr, rad2deg(-IA_r)]) * sf_x;
    end
    fxf(find(abs(fxf) > 1000)) = [];
    fxr(find(abs(fxr) > 1000)) = [];
    FXF = min(fxf);
    FXR = min(fxr);
    FX = abs(2*FXF+2*FXR);
    AX = FX/W;
    AX_diff = AX-Ax;
    while AX_diff>0
        %disp([Ax AX])
        Ax = Ax+.01;
        WS = W/2;
        pitch = Ax*pg*pi/180;
        wf = (WF+DF*CoP)/2;
        wr = (WR+DF*(1-CoP))/2;
        wf = wf+Ax*cg*WS/l/24;
        wr = wr-Ax*cg*WS/l/24;
        IA_f = -l*12*sin(pitch)/2*IA_gainf + IA_0f;% - KPIf*(1-cos(deltaf)) + casterf*sin(deltaf);
        IA_r = l*12*sin(pitch)/2*IA_gainr + IA_0r;% - KPIr*(1-cos(deltar)) + casterf*sin(deltar);
        FZ_vals = [-250:1:-50];
        sl = [-.15:.01:0];
        for k = 1:length(sl)
            fxf(k) = MF52_Fx_fcn(B, [sl(k), -wf, rad2deg(-IA_f)]) * sf_x;
            fxr(k) = MF52_Fx_fcn(B, [sl(k), -wr, rad2deg(-IA_r)]) * sf_x;
        end
        fxf(find(abs(fxf) > 1000)) = [];
        fxr(find(abs(fxr) > 1000)) = [];
        FXF = min(fxf);
        FXR = min(fxr);
        FX = abs(2*FXF+2*FXR);
        AX = FX/W;
        AX_diff = AX-Ax;
    end
    A_X(i) = AX;
end

velocity_y = lateralg.*32.2.*radii;
velocity_y = sqrt(velocity_y);

r_max = max(radii);
spcount = spcount+1;
shift_points(spcount) = V+1;
top_speed = V;
VMAX = top_speed;

% make the rest of your functions for the GGV diagram
% braking as a function of speed
deccel = csaps(velocity,A_X);
velocity = 15:5:80;
% lateral g's as a function of velocity
lateral = csaps(velocity_y,lateralg);
radii = velocity_y.^2./lateralg/32.2;
% max velocity as a function of instantaneous turn radius
cornering = csaps(radii,velocity_y);
%% Section 7: Load Endurance Track Coordinates
disp('Loading Endurance Track Coordinates')
[data text] = xlsread('FB2026EnduranceCoordinatesNew.xlsx','Sheet1');

% the coordinates are now contained within 'data'. This is a 5 column
% matrix that contains a set of defined 'gates' that the car must mavigate
% through
% Column 1: Gate #
% Column 2: Outside boundary, x coordinate
% Column 3: Outside boundary, y coordinate
% Column 4: Inside boundary, x coordinate
% Column 5: Inside boundary, y coordinate

% sort the data into "inside" and "outside" cones
outside = data(:,2:3);
inside = data(:,4:5);
t = [1:length(outside)];
% define the minimum turn radius of the car
r_min = 4.5*3.28;
r_min = r_min-tw/2;
pp_out = spline(t,outside');
pp_in = spline(t,inside');

for i = 1:1:length(outside)
    % isolate individual gates
    gate_in = inside(i,:);
    gate_out = outside(i,:);
    % create the line that connects the two cones together
    x1 = gate_in(1);
    x2 = gate_out(1);
    y1 = gate_in(2);
    y2 = gate_out(2);
    % polynomial expression for the line:
    coeff = polyfit([x1, x2], [y1, y2], 1);
    % adjust the width of the gate for the width of the car:
    gate_width = sqrt((x2-x1)^2+(y2-y1)^2);
    path_width = gate_width-tw;
    x_fs = tw/(2*gate_width);
    % update the gate boundaries based on said new width
    x_bound = [min(x1,x2)+x_fs*abs(x2-x1),max(x1,x2)-x_fs*abs(x2-x1)];
    path_boundaries(i,:) = [coeff x_bound];
end
%% Seciton 8: Load Endurance Racing Line
disp('Loading Endurance Racing Line')
% xx = load('endurance_racing_line.mat');
% xx = xx.endurance_racing_line;
xx = ones(1, 150) * 0.5;

%% Section 9: Optimize Endurance Racing Line
% The pre-loaded racing line should work for most applications; however,
% if you have the need to re-evaluate or generate a new optimized racing
% line, simply un-comment the code below:


% disp('Optimizing Endurance Racing Line')
% A = eye(length(xx));
% b = ones(length(xx),1);
% lb = zeros(1,length(xx));
% ub = ones(1,length(xx));
% options = optimoptions('fmincon',...
%     'Algorithm','sqp','Display','iter','ConstraintTolerance',1e-12);
% options = optimoptions(options,'MaxIter', 10000, 'MaxFunEvals', 1000000,'ConstraintTolerance',1e-12,'DiffMaxChange',.1);
% 
% x = fmincon(@lap_time,xx,[],[],[],[],lb,ub,@track_curvature,options);
% xx = x;
% x(end+1) = x(1);
% x(end+1) = x(2);
%% Section 10: Generate Final Endurance Trajectory
x = xx;
% Plot finished line
x(end+1) = x(1);
x(end+1) = x(2);
for i = 1:1:length(x)
    % for each gate, find the position defined between the cones
    coeff = path_boundaries(i,1:2);
    x2 = max(path_boundaries(i,3:4));
    x1 = min(path_boundaries(i,3:4));
    position = x(i);
    % place the car within via linear interpolation
    x3 = x1+position*(x2-x1);
    y3 = polyval(coeff,x3);
    %plot(x3,y3,'og')
    % the actual car's trajectory defined in x-y coordinates:
    path_points(i,:) = [x3 y3];
end

x = linspace(1,t(end-1),1000);
ppv = pchip(t,path_points');
vehicle_path = ppval(ppv,x);
vehicle_path_EN = vehicle_path;
Length = arclength(vehicle_path(1,:),vehicle_path(2,:));
Length_endurance = Length; % ft, kept for Section 24 (Length is reassigned for autocross below)
%% Section 11: Simulate Endurance Lap
disp('Plotting Vehicle Trajectory')
[laptime time_elapsed velocity acceleration lateral_accel gear_counter path_length weights distance] = lap_information(xx);
%% Section 12: Load Autocross Track Coordinates
disp('Loading Autocross Track Coordinates')
%[data text] = xlsread('Autocross_Coordinates_2.xlsx','Scaled');
[data text] = xlsread('FB2026AutoCrossCoordinatesNew.xlsx','Sheet1');
outside = data(:,2:3);
inside = data(:,4:5);
t = [1:length(outside)];
r_min = 4.5*3.28;
r_min = r_min-tw/2;
pp_out = spline(t,outside');
pp_in = spline(t,inside');

%plot(outside(:,1),outside(:,2),'ok')
%hold on
%plot(inside(:,1),inside(:,2),'ok')
clear path_boundaries
for i = 1:1:length(outside)
    gate_in = inside(i,:);
    gate_out = outside(i,:);
    %plot([gate_in(1) gate_out(1)],[gate_in(2) gate_out(2)],'-k')
    x1 = gate_in(1);
    x2 = gate_out(1);
    y1 = gate_in(2);
    y2 = gate_out(2);
    coeff = polyfit([x1, x2], [y1, y2], 1);
    gate_width = sqrt((x2-x1)^2+(y2-y1)^2);
    path_width = gate_width-tw;
    x_fs = tw/(2*gate_width);
    x_bound = [min(x1,x2)+x_fs*abs(x2-x1),max(x1,x2)-x_fs*abs(x2-x1)];
    path_boundaries_ax(i,:) = [coeff x_bound];
    %text(round(x1),round(y1),num2str(i))
end


%save('path_boundaries.mat','path_boundaries');
%% Section 13: Load Autocross Racing Line
disp('Loading Autocross Racing Line')
% xx = load('autocross_racing_line.mat');
% xx = xx.autocross_racing_line;
xx = ones(1, 150) * 0.5;
%% Section 14: Optimize Autocross Racing Line
% Same applies here, optimizing the line is optional but if you want,
% simply un-comment the lines of code below:


% disp('Optimizing Racing Line')
% A = eye(length(xx));
% b = ones(length(xx),1);
% lb = zeros(1,length(xx));
% ub = ones(1,length(xx));
% options = optimoptions('fmincon',...
%     'Algorithm','sqp','Display','iter','ConstraintTolerance',1e-12);
% options = optimoptions(options,'MaxIter', 10000, 'MaxFunEvals', 1000000,'ConstraintTolerance',1e-12,'DiffMaxChange',.1);
% 
% x = fmincon(@lap_time_sprint,xx,[],[],[],[],lb,ub,@track_curvature_sprint,options);
% xx_auto = x;
% % x(end+1) = x(1);
% % x(end+1) = x(2);
%% Section 15: Generate Final Autocross Trajectory
xx_auto = xx;
x = xx_auto;
%Plot finished line

for i = 1:1:length(x)
    coeff = path_boundaries_ax(i,1:2);
    x2 = max(path_boundaries_ax(i,3:4));
    x1 = min(path_boundaries_ax(i,3:4));
    position = x(i);
    x3 = x1+position*(x2-x1);
    y3 = polyval(coeff,x3);
    %plot(x3,y3,'og')
    path_points_ax(i,:) = [x3 y3];
end
x = linspace(1,t(end),1000);
ppv = pchip(t,path_points_ax');
vehicle_path = ppval(ppv,x);
vehicle_path_AX = vehicle_path;
Length = arclength(vehicle_path(1,:),vehicle_path(2,:));
%% Section 16: Simulate Autocross Lap
disp('Plotting Vehicle Trajectory')
[laptime_ax time_elapsed_ax velocity_ax, acceleration_ax lateral_accel_ax gear_counter_ax path_length_ax weights_ax distance_ax] = lap_information_sprint(xx_auto);
%% Section 17: Calculate Dynamic Event Points
disp('Calculating Points at Competition')
% calculate endurance score
% Tmin = 115.249;
% Tmax = Tmin*1.45;
% Endurance_Score =250*((Tmax/(laptime+13))-1)/(Tmax/Tmin-1)+25;
Pmax = 325;
Tfast = 2054;
Tteam = (laptime/1.5)*22;
Tmax = 1.1*Tfast;
if Tteam>Tfast
    Endurance_Score = 0.9*Pmax*(((Tmax/Tteam)-1)/0.1) + 0.1*Pmax
    if Endurance_Score < 0
        Endurance_Score = 0;
    elseif Endurance_Score > Pmax
        Endurance_Score = Pmax;
    end
else
    Endurance_Score = Pmax;
end

% Calculate autocross score
Pmaxax = 100;
Tfastax = 121.303;
Tmaxax = Tfastax*1.25;
Tteamax = laptime_ax;
if Tteamax>Tfastax
    Autocross_Score = 0.95*Pmaxax*(((Tmaxax/Tteamax)-1)/0.25) + 0.05*Pmaxax;
    if Autocross_Score < 0
        Autocross_Score = 0;
    elseif Autocross_Score > Pmaxax
        Autocross_Score = Pmaxax;
    end
else
    Autocross_Score = Pmaxax;
end
Autocross_Score;

% Skidpad Analysis
% define skidpad turn radius (ft)
path_radius = 25+tw/2+.5;
% determine speed possible to take:
speed = fnval(cornering,path_radius);
% calculate skidpad time
skidpad_time = path_radius*2*pi/speed;
% calculate score from Formula Bharat reference times (see Section 17 constants)
Pmaxsk = 75;
Tfastsk = 6.91;
Tmaxsk = Tfastsk*1.25;
Tteamsk = skidpad_time;
if Tteamsk>Tfastsk
    Skidpad_Score = 0.95*Pmaxsk*((((Tmaxsk/Tteamsk)^2)-1)/0.5625) + 0.05*Pmaxsk
    if Skidpad_Score < 0
        Skidpad_Score = 0;
    elseif Skidpad_Score > Pmaxsk
        Skidpad_Score = Pmaxsk;
    end
else
    Skidpad_Score = Pmaxsk;
end


% Acceleration Analysis
% start at speed 0, gear 1, etc
count = 0;
v = 0;
vel = v;
gears = find((shift_points-vel)>0);
gear = gears(1)-1;
newgear = gear;
time_shifting = 0;
interval = 1;
% accel track is 247 feet, so I am defining 247 segments of 1 foot:
segment = 1:1:247;
t_accel_elapsed = 0;
clear dt_f v_f
% little quickie accel sim:
for i = 1:1:length(segment)
    d = 1;

    %gear = newgear;
    % find what gear you are in
    gears = find((shift_points-vel)>0);
    newgear = gears(1)-1;
    
    % compare to previous iteration, to detect an upshift
    if newgear > gear
        shifting = 1;
    else
        shifting = 0;
    end

    vmax = VMAX;
    % determine instantaneous acceleration capacity
    AX = fnval(accel,vel);
    dd = d/interval;
    for j = 1:1:interval
        count = count+1;
        vehicle_gear(count) = gear;
        if shifting == 1 & vel < vmax;
            % if you are shifting, then you are not accelerating, but
            % continue to travel forward at constant velocity
            dt_f(count) = dd/vel;
            time_shifting = time_shifting+dt_f(count);
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
            vel = vel;
        elseif vel < vmax
            % if you are not shifting, and top speed has not been achieved
            % then you keep accelerating at maximum capacity possible
            ax_f(count) = AX;
            tt = roots([0.5*32.2*ax_f(count) vel -dd]);
            dt_f(count) = max(tt);
            dv = 32.2*ax_f(count)*dt_f(count);
            dvmax = vmax-vel;
            dv_f(count) = min(dv,dvmax);
            v_f(count) = vel+dv_f(count); 
            vel = v_f(count);
            gears = find((shift_points-vel)>0);
            newgear = gears(1)-1;
            if newgear > gear
                shifting = 1;
            end
        else
            % if you are not shifting but you are at top speed, then just
            % hold top speed
            vel = vmax;
            dt_f(count) = dd/vel;
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
        end
        if time_shifting > shift_time
            shifting = 0;
            time_shifting = 0;
            gear = newgear;
        end
    end
    if shifting == 1
        gear = gear;
    else
        gear = newgear;
    end
    t_accel_elapsed = t_accel_elapsed+dt_f(count);
    t_accel(i) = t_accel_elapsed;
end
accel_time = sum(dt_f(2:end))+.1;
% calculate accel score:
Pmaxac = 75;
Tfastac = 5.5;
Tmaxac = Tfastac*1.5;
Tteamac = accel_time;
if Tteamac>Tfastac
    Accel_Score = 0.95*Pmaxac*(((Tmaxac/Tteamac)-1)/0.5) + 0.05*Pmaxac;
    if Accel_Score < 0
        Accel_Score = 0;
    elseif Accel_Score > Pmaxac
        Accel_Score = Pmaxac;
    end
else
    Accel_Score = Pmaxac;
end
Accel_Score;
Total_Points = Accel_Score+Skidpad_Score+Autocross_Score+Endurance_Score

results = [time_elapsed' velocity'];
% xlswrite('logged_data.xlsx',results,'sim_data')

%% Section 18: Generate Load Cases
disp('Generating Load Cases')
% find all three worst case acceleration cases:
AX_min = min(acceleration);
AX_max = max(acceleration);
AY_max = max(lateral_accel);
% then find where they took place
% take the first match if several samples share the extreme value
VX_min = velocity(find(acceleration == AX_min));
VX_max = velocity(find(acceleration == AX_max));
VY_max = velocity(find(lateral_accel == AY_max));

VX_min = VX_min(1);
VX_max = VX_max(1);
VY_max = max(VY_max);
frontF = zeros(3,3);
rearF = zeros(3,3);
% then calculate loads based on those speeds and accelerations: 
% see documentation spreadsheet for translation
frontF(3,:) = [WF/2 + Cl*VX_max^2*CoP/2 - WF*AX_max*cg/l/2 , WF/2 + Cl*VX_min^2*CoP/2 - WF*AX_min*cg/l/2, WF/2 + Cl*VY_max^2*CoP/2 + WF*AY_max*cg/tw/2];
rearF(3,:) = [WR/2 + Cl*VX_max^2*(1-CoP)/2 + WR*AX_max*cg/l/2 , WR/2 + Cl*VX_min^2*(1-CoP)/2 + WR*AX_min*cg/l/2, WR/2 + Cl*VY_max^2*(1-CoP)/2 + WR*AY_max*cg/tw/2];
frontF(2,:) = [0 0 (WF/2+WF*AY_max*cg/tw/2)*AY_max];
rearF(2,:) = [0 0 (WR/2+WR*AY_max*cg/tw/2)*AY_max];
frontF(1,:) = [0 -(WF/2 -WF*AX_min*cg/l/2)*AX_min 0];
rearF(1,:) = [W*AX_max/2 -(WR/2 +WR*AX_min*cg/l/2)*AX_min 0];
%% Section 19: Plot Results
disp('Plotting Results')
% This is just to make some pretty pictures, feel free to comment this out
figure
plot(distance,velocity,'k')
title('Endurance Simulation Velocity Trace')
xlabel('Distance Travelled (d) [ft]')
ylabel('Velocity (V) [ft/s]')
figure
plot(distance,acceleration,distance,lateral_accel)
title('Endurance Simulation Acceleration Traces')
xlabel('Distance Travelled (d) [ft]')
ylabel('Acceleration [g]')
legend('Longitudinal','Lateral')
figure
plot(distance_ax,velocity_ax,'k')
title('Autocross Simulation Velocity Trace')
xlabel('Distance Travelled (d) [ft]')
ylabel('Velocity (V) [ft/s]')
figure
plot(distance_ax,acceleration_ax,distance_ax,lateral_accel_ax)
title('Autocross Simulation Acceleration Traces')
xlabel('Distance Travelled (d) [ft]')
ylabel('Acceleration [g]')
legend('Longitudinal','Lateral')
disp('Analysis Complete')

%% -----------------------------------------------------------------
%%% Section 20: Plot GGV Diagram
% This script plots the 3D GGV (g-g-V) diagram for the vehicle.
% It uses the spline functions created in Section 6 to generate a
% smooth,elliptical 3D surface plot representing the 
% vehicle's performance envelope.
%
% This code should be run AFTER Section 6 is complete.
% -----------------------------------------------------------------

disp('Plotting GGV Diagram')

% 1. Create a high-resolution velocity vector and angle vector
% We use the min/max speeds from the simulation to set the range
v_min_plot = min(min(velocity), min(velocity_y));
v_max_plot = max(max(velocity), max(velocity_y));

N_vel = 20; % Number of velocity steps
N_angle = 40; % Number of points around the ellipse

plot_vel = linspace(v_min_plot, v_max_plot, N_vel);
theta = linspace(0, 2*pi, N_angle);

% 2. Evaluate the pure G limits at these velocities using the splines
ggv_accel = fnval(accel, plot_vel);       % Max positive longitudinal G (Ax)
ggv_deccel = -fnval(deccel, plot_vel);     % Max negative longitudinal G (Ax)
ggv_lateral_pos = fnval(lateral, plot_vel); % Max positive lateral G (Ay)
ggv_lateral_neg = -fnval(lateral, plot_vel); % Max negative lateral G (Ay)

% 3. Generate the 3D surface coordinates
% We will create (X,Y,Z) matrices for the surf command
X_plot = zeros(N_vel, N_angle); % Lateral G (for X-axis)
Y_plot = zeros(N_vel, N_angle); % Longitudinal G (for Y-axis)
Z_plot = zeros(N_vel, N_angle); % Velocity (for Z-axis)

for i = 1:N_vel % Loop through each velocity slice
    
    % Get the max G values for this specific velocity
    ax_pos = ggv_accel(i);
    ax_neg = ggv_deccel(i);  % This is already negative
    ay_pos = ggv_lateral_pos(i);
    ay_neg = ggv_lateral_neg(i); % This is already negative

    % Generate the points for the "ellipse" at this velocity
    for j = 1:N_angle
        t = theta(j);
        cos_t = cos(t);
        sin_t = sin(t);
        
        % Determine longitudinal G (Y-axis of plot)
        if cos_t >= 0 % Accelerating
            Y_plot(i, j) = ax_pos * cos_t;
        else % Braking
            Y_plot(i, j) = abs(ax_neg) * cos_t;
        end

        % Determine lateral G (X-axis of plot)
        if sin_t >= 0 % Left Turn
            X_plot(i, j) = ay_pos * sin_t;
        else % Right Turn
            X_plot(i, j) = abs(ay_neg) * sin_t;
        end
        
        % Set the velocity (Z-axis)
        Z_plot(i, j) = plot_vel(i);
    end
end

% 6. Plot the smooth surface
figure; % Create a new figure
h = surf(X_plot, Y_plot, Z_plot);

% 7. Add formatting
title('Vehicle Performance Envelope (GGV Diagram)');
xlabel('Lateral Acceleration Capacity (g)');
ylabel('Longitudinal Acceleration Capacity (g)');
zlabel('Vehicle Speed (ft/s)');

% Set visual properties
set(h, 'FaceAlpha', 0.8, 'FaceColor', 'interp', 'EdgeColor', 'none');
view(60, 25); % Set a 3D viewing angle (azimuth, elevation)
grid on;
axis tight;
colorbar; % Add a color bar to show speed


%% -----------------------------------------------------------------
% %% Section 21: Plot Yaw Balance (Understeer Gradient)
%
% This script plots the vehicle's yaw balance characteristics.
% It uses the 'Ugradient' and 'lateralg' variables calculated during 
% the GGV cornering simulation (Section 6).
%
% This plot is the standard way to visualize understeer/oversteer
% behavior as a function of cornering force.
%
% This code MUST be run AFTER Section 6 is complete.
% -----------------------------------------------------------------

% disp('Plotting Yaw Balance (Understeer Gradient)')
% 
% % 1. Create a new figure for the yaw balance plot
% figure;
% 
% % 2. Plot Understeer Gradient vs. Lateral G
% % 'lateralg' and 'Ugradient' are calculated in Section 6
% h = plot(lateralg, Ugradient, 'b-o', 'LineWidth', 2, 'MarkerSize', 5);
% hold on;
% 
% % 3. Plot the neutral steer line (y=0) for reference
% % This line represents a perfectly balanced, neutral-steer car.
% plot(xlim, [0 0], 'k--', 'LineWidth', 1.5);
% 
% % 4. Add formatting
% title('Vehicle Yaw Balance');
% xlabel('Lateral Acceleration (g)');
% ylabel('Understeer Gradient (deg/g)');
% legend('Vehicle Characteristic', 'Neutral Steer (y=0)', 'Location', 'northwest');
% grid on;
% hold off;


% -----------------------------------------------------------------
%% Section 22: Plot Slip Angles vs. Lateral G
%
% This script plots the front and rear slip angles (a_f, a_r)
% and the vehicle sideslip angle (beta) as a function of
% lateral acceleration.
%
% This code MUST be run AFTER Section 6 is complete and the
% slip_angle_f/r and sideslip_angle arrays have been created.
% -----------------------------------------------------------------
% disp('Plotting Slip Angles vs. Lateral G')
% 
% figure;
% plot(lateralg, slip_angle_f, 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r', 'MarkerSize', 4);
% hold on;
% plot(lateralg, slip_angle_r, 'b-s', 'LineWidth', 2, 'MarkerFaceColor', 'b', 'MarkerSize', 4);
% plot(lateralg, sideslip_angle, 'g--.', 'LineWidth', 1.5, 'MarkerSize', 10);
% 
% % Plot the zero line for reference
% plot(xlim, [0 0], 'k:', 'LineWidth', 1);
% 
% title('Vehicle Slip Characteristics');
% xlabel('Lateral Acceleration (g)');
% ylabel('Angle (degrees)');
% legend('Front Slip Angle (a_f)', 'Rear Slip Angle (a_r)', 'Vehicle Sideslip (beta)', 'Location', 'best');
% grid on;
% hold off;
%% Section 23: Final Score Summary
% This section prints a clean summary of all dynamic event times and scores.

disp(' ') % Add a blank line for spacing
disp('=======================================')
disp('      FINAL EVENT SCORES SUMMARY       ')
disp('=======================================')

% Use fprintf for neatly formatted output
fprintf('%-12s Time: %7.3f s   |   Score: %6.2f pts\n', 'Acceleration:', accel_time, Accel_Score);
fprintf('%-12s Time: %7.3f s   |   Score: %6.2f pts\n', 'Skidpad:', skidpad_time, Skidpad_Score);
fprintf('%-12s Time: %7.3f s   |   Score: %6.2f pts\n', 'Autocross:', laptime_ax, Autocross_Score);
fprintf('%-12s Time: %7.3f s   |   Score: %6.2f pts\n', 'Endurance:', laptime, Endurance_Score);

disp('---------------------------------------')

% Display the total score (using <strong> for bold text in MATLAB command window)
fprintf('<strong>TOTAL DYNAMIC POINTS: %6.2f pts</strong>\n', Total_Points);

disp('=======================================')
disp(' ')

%% Section 24: Energy Consumption Analysis
disp('Calculating Energy Consumption')

% --- 1. Initialize Variables ---
% Conversion factors
ftlb_to_joules = 1.35582;
joules_to_kwh = 2.77778e-7;

% Rolling Resistance Coefficient (Approximate for FSAE tires)
C_rr = 0.015; 

% Pre-allocate arrays
N = length(distance) - 1;
ds_vec = zeros(1, N);
dt_vec = zeros(1, N);
power_vec = zeros(1, N);
energy_vec = zeros(1, N);
cumulative_energy = zeros(1, N);

% --- 2. Iterate through Trajectory ---
for i = 1:N
    % A. Calculate ds (Distance between gates)
    ds = distance(i+1) - distance(i);
    ds_vec(i) = ds;
    
    % B. Calculate dt (Differential Time)
    % Average velocity in this segment (ft/s)
    v_avg = (velocity(i) + velocity(i+1)) / 2;
    
    % Protect against divide by zero at start line
    if v_avg < 0.1
        v_avg = 0.1;
    end
    
    dt = ds / v_avg;
    dt_vec(i) = dt;
    
    % C. Calculate Forces
    % 1. Inertial Force (F = ma)
    % acceleration(i) is in g's, convert to ft/s^2
    accel_ft_s2 = acceleration(i) * 32.2;
    mass_slugs = W / 32.2;
    F_inertia = mass_slugs * accel_ft_s2;
    
    % 2. Aero Drag Force (F_aero = Cd * v^2)
    % Note: Your Cd variable (.0184) seems to already include density/area 
    % based on Section 6 logic: FX = FX-Cd*V^2
    F_aero = Cd * v_avg^2;
    
    % 3. Rolling Resistance (F_rr = C_rr * W)
    F_rr = C_rr * W;
    
    % Total Tractive Force required at the contact patch
    F_total = F_inertia + F_aero + F_rr;
    
    % D. Calculate Motor Mechanics
    % 1. Angular Velocity (rad/s)
    omega_wheel = v_avg / tyreRadius; 
    omega_motor = omega_wheel * gearTot;
    
    % 2. Motor Torque (ft-lbs)
    % Torque at wheel = Force * Radius
    torque_wheel = F_total * tyreRadius;
    % Torque at motor = Torque wheel / Gear Ratio
    torque_motor_mech = torque_wheel / gearTot;
    
    % E. Calculate Power (P = Torque * Omega)
    % This is mechanical power at the motor shaft
    power_mechanical_ftlb_s = torque_motor_mech * omega_motor;
    
    % Apply Efficiency to get Electrical Power
    % If power > 0, we are consuming energy (divide by efficiency)
    % If power < 0, we are regen braking (multiply by efficiency)
    
    if power_mechanical_ftlb_s > 0
        % Motoring: Battery Power > Mechanical Power
        power_electrical = power_mechanical_ftlb_s / drivetrainLosses;
    else
        % Braking/Regen: Battery Power < Mechanical Power
        % Note: Set this to 0 if you do not want to simulate Regen
        regen_efficiency = 0.6; % Conservative estimate for regen capture
        power_electrical = power_mechanical_ftlb_s * regen_efficiency;
    end
    
    power_vec(i) = power_electrical; % ft-lb/s
    
    % F. Calculate Energy (E = P * dt)
    dE = power_electrical * dt; % ft-lb
    energy_vec(i) = dE;
    
    if i == 1
        cumulative_energy(i) = dE;
    else
        cumulative_energy(i) = cumulative_energy(i-1) + dE;
    end
end

% --- 3. Summation and Conversions ---
total_energy_ftlb = sum(energy_vec);
total_energy_joules = total_energy_ftlb * ftlb_to_joules;
total_energy_kwh = total_energy_joules * joules_to_kwh;

% Calculate average power
avg_power_kw = (mean(power_vec) * ftlb_to_joules) / 1000;

% --- 4. Display Results ---
disp(' ')
disp('=======================================')
disp('      ENERGY CONSUMPTION ANALYSIS      ')
disp('=======================================')
fprintf('Lap Energy (Mechanical): %6.2f ft-lbs\n', sum(energy_vec(energy_vec>0)) * drivetrainLosses);
fprintf('Lap Energy (Electrical): %6.4f kWh\n', total_energy_kwh);
fprintf('Average Power Draw:      %6.2f kW\n', avg_power_kw);
fprintf('Total Endurance Energy:  %6.4f kWh (assuming 22km / 1 lap length)\n', total_energy_kwh * (22000 / (Length_endurance/3.28)));
disp('=======================================')
disp(' ')

% --- 5. Plot Power Usage ---
figure
plot(distance(1:end-1), power_vec .* ftlb_to_joules ./ 1000)
title('Power Consumption Trace')
xlabel('Distance [ft]')
ylabel('Power [kW]')
grid on