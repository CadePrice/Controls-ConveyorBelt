function P = conveyor_belt_params(ovr)
%CONVEYOR_BELT_PARAMS  Every number the conveyor models use, in one place.
%   P = conveyor_belt_params()      defaults
%   P = conveyor_belt_params(ovr)    defaults, then the fields of struct ovr
%                                replace them (this is how the GUIs work)
%
%   Units are in the names: _mm, _mms (mm per second), _s, _V, _A, _N, _kg.
%   Values marked [ASSUMED] are best guesses for the real parts. Each one
%   says how to measure it; when you have a measurement, type it in here.
%
%   Sections 1-6 are yours to edit. Everything below "DON'T EDIT" comes
%   from the CAD files in CAD/ or is worked out from the sections above.

%% 1. What should it do ---------------------------------------------------
P.targetSpeed_mms   = 100;    % belt speed you want (items ride on the belt, so they arrive at this speed too)
P.stepSize_mms      = 50;     % then the target jumps by this much...
P.stepTime_s        = 10;     % ...at this time
P.stopTime_s        = 100;    % how long the GUIs simulate (the .slx Stop Time box is 100 s)
P.firstDrop_s       = 0.5;    % first item leaves your hand at this time
P.dropEvery_s       = 1.5;    % then one more item every this many seconds
P.dropHeight_mm     = 20;     % how far above the belt you let go

% Open-loop model only: the PWM duty pulse you send to the motor (0 = off, 100 = full supply)
P.openStartDuty_pct = 0;      % duty (%) before and after the pulse
P.openPulseDuty_pct = 20;     % extra duty (%) during the pulse (20 % gives about 120 mm/s)
P.openPulseStart_s  = 1;      % pulse starts
P.openPulseWidth_s  = 79;     % pulse lasts (on from 1 s to 80 s, then the belt coasts to a stop)

%% 2. Gains (PID running on the Arduino) ---------------------------------
% error = target belt speed - belt speed from the motor's encoder (mm/s)
% controller output is PWM duty in % (0-100, like analogWrite 0-255)
P.Kp    = 0.05;    % % duty per (mm/s) of error
P.Ki    = 0.5;     % % duty per mm (error added up over time)
P.Kd    = 0;       % % duty per (mm/s per s). Leave at 0 at first: it amplifies encoder noise.
P.Nfilt = 10;      % D-term filter (1/s). Keep below 60 or the discrete filter goes unstable.

%% 3. Actuator: Arduino PWM -> MOSFET driver -> TS25GA 12 V 600 RPM gearmotor (drives the roller directly)
P.supplyV              = 12.0;   % [ASSUMED] adaptor voltage. Measure: multimeter across the supply with the motor running.
P.driverDrop_V         = 0;      % voltage lost in the driver. Logic-level MOSFET (IRLZ44N) ~0. L298N ~2: measure motor-terminal volts at full PWM vs supply.
P.motorRatedV          = 12.0;   % voltage the motor's speed spec is quoted at
P.outputRPM            = 600;    % no-load speed of the OUTPUT shaft at motorRatedV (from the listing).
                                 % Measure: run it at 12 V with nothing attached, count encoder pulses for 1 s.
P.gearRatio            = 10;     % [ASSUMED] built-in gearbox. Measure: turn the output shaft exactly one turn by hand
                                 % and count the encoder pulses on one channel; ratio = pulses / encPPR.
P.noLoadCurrent_A      = 0.12;   % [ASSUMED] Measure: multimeter in series, motor alone (nothing attached) at 12 V.
P.motorR_ohm           = 6.0;    % [ASSUMED] Measure: ohmmeter across the motor terminals; turn the shaft a little and take the LOWEST steady reading.
P.rotorJ_kgm2          = 8e-7;   % [ASSUMED] 370-size armature. If the open-loop rise is faster/slower than real, scale this.
P.gearEff              = 0.8;    % [ASSUMED] built-in metal gearbox. Lower it if the real belt is slower under load than the model.
P.gearboxDragCurrent_A = 0;      % extra friction outside the gearmotor (none: the shaft goes straight into the roller)

%% 4. Mechanical -----------------------------------------------------------
P.beltDragCurrent_A = 0.02;   % [ASSUMED] Measure: current with the belt on minus current with no belt (same duty).
P.itemMass_kg       = 0.005;  % [ASSUMED] printed 25 mm cube, 15 % infill. Measure: kitchen scale.
P.itemFriction      = 0.6;    % [ASSUMED] item on TPU belt. Measure: tilt the belt until an item slides; friction = tan(angle).
P.bedFriction       = 0.3;    % [ASSUMED] TPU belt on the PLA bed (with tape). Extra drag per newton of item weight.
P.plaChain          = 0;      % belt type: 0 = printed TPU belt on crowned rollers,
                              %            1 = PLA snap-link chain on 12-tooth sprockets.
                              % Run build_conveyor_models again after changing this (it swaps the 3-D parts).

%% 5. Encoder (feedback) and the Arduino's speed estimate ------------------
% The Hall encoder on the back of the motor counts motor-shaft turns. The motor drives
% the roller directly through its gearbox and the belt doesn't slip, so
% belt speed = motor speed / gearRatio x roller radius.
P.controlPeriod_s = 0.01;  % Arduino works out the speed and runs the PID every 10 ms.
                           % Must be a whole number of 10 ms steps (0.01, 0.02, 0.03 ...).
P.encPPR          = 11;    % [ASSUMED] pulses per motor turn on ONE channel (common for these motors)
P.encDecode       = 4;     % edges counted per pulse: 4 = both edges of A and B (two interrupts),
                           % 2 = both edges of A only, 1 = rising edge of A only
P.encWindow       = 5;     % belt speed = (count now - count 5 samples ago) / (5 x 10 ms), in mm/s

%% 6. Real-world effects (0 = off) ----------------------------------------
P.delay_s      = 0.02;  % [ASSUMED] Arduino loop + I2C + PWM update delay
P.encNoise_counts = 0.1; % [ASSUMED] encoder edge jitter (counts, std). Hall edges are not perfectly evenly spaced.
P.dragNoise_N  = 0;     % random extra belt drag (belt seam, dirt), standard deviation in N
P.noiseSeed    = 1;     % change to get a different (but repeatable) noise pattern

%% ======================= DON'T EDIT BELOW THIS LINE =======================

%% Overrides from the GUIs ------------------------------------------------
if nargin > 0 && ~isempty(ovr)
    fn = fieldnames(ovr);
    for q = 1:numel(fn)
        if ~isfield(P, fn{q})
            error('conveyor_belt_params:unknownField', ...
                'Override "%s" is not a parameter in sections 1-6 of conveyor_belt_params.m.', fn{q});
        end
        P.(fn{q}) = ovr.(fn{q});
    end
end

%% Fixed values from the CAD (CAD/*.step). Change the CAD, then these. ------
P.nItems        = 8;          % items in the model (structural: rebuild if changed)
P.rEff_m        = 0.0159;     % crowned roller 15.3 mm + half the 1.2 mm belt (PLA chain: see derived numbers)
P.xDrive_m      = 0.100;      % drive (head) roller axis, items fall off past here
P.xIdler_m      = -0.100;     % idler (tail) roller axis
P.zAxis_m       = 0.070;      % roller axis height above the table
P.beltTop_m     = 0.0865;     % top surface of the belt
P.itemSize_m    = 0.025;      % cube edge
P.xDrop_m       = -0.085;     % where items land (centre of the drop chute)
P.zBinFloor_m   = 0.002;      % inside floor of catch_bin.step
P.J_rollerIdler = 3.50e-6;    % kg m^2, from CAD volume x PLA 1.24 g/cm^3 x 0.6 fill
P.m_belt_kg     = 0.0327;     % TPU 95A belt, 1.2 mm x 46 mm x 490 mm
P.printDensity_kgm3 = 1240*0.6;  % PLA x ~0.6 effective fill, used by every File Solid
% part placement (m) for the 3-D view
P.bearingY_m = [0.031 -0.038];
% numerical settings: chosen so the model is stable with ode4 at a fixed 10 ms step
P.fixedStep_s   = 0.01;      % the models' solver step (Model Settings > Solver)
P.contactOmega  = 80;        % rad/s, how stiff item-belt contact is (scales with item mass)
P.contactZeta   = 0.7;       % contact damping ratio
P.frictionRate  = 50;        % 1/s, how quickly the belt drags an item up to belt speed (capped by friction)
P.vSlip         = 0.005;     % m/s, belt drag is smoothed below this speed
P.wSlip         = 22;        % rad/s at the motor, motor friction is smoothed below this speed
P.handOmega     = 40;        % rad/s, how firmly the "hand" carries items back to the chute
P.zCarry_m      = 0.175;     % height items pass over the chute on the way back
P.zUnder_m      = -0.05;     % items travel back this far below the table top (out of sight)
P.xReturn_m     = -0.16;     % where they come back up, just upstream of the frame
P.tableSize_m   = [0.45 0.26 0.01];     % display-only table top under the conveyor
P.tableCentre_m = [0.02 0 -0.005];
P.binFriction = 0.5;
P.dragNoiseTs_s = 0.01;
P.g = 9.80665;

%% Derived numbers (worked out from everything above) ---------------------
wNoLoad      = P.outputRPM*P.gearRatio*2*pi/60;                       % motor shaft, rad/s
P.Ke         = (P.motorRatedV - P.noLoadCurrent_A*P.motorR_ohm)/wNoLoad;  % V s/rad = N m/A
P.Tfric_Nm   = P.Ke*(P.noLoadCurrent_A + P.gearboxDragCurrent_A);    % friction torque at the motor
if P.plaChain   % PLA link chain: pitch line, top surface and mass from CAD/chain_link_PLA.step
    P.rEff_m = 0.017562;  P.beltTop_m = 0.08986;  P.m_belt_kg = 0.0922;  P.J_rollerIdler = 2.9e-6;
end
P.beltDrag_N = P.Ke*P.beltDragCurrent_A*P.gearRatio*P.gearEff/P.rEff_m;
N = P.gearRatio;
P.J_drive    = N^2*P.rotorJ_kgm2 + P.J_rollerIdler + P.m_belt_kg*P.rEff_m^2;  % kg m^2 seen by the drive roller
                                                      % (the drive roller's own inertia comes from its STEP files)
P.restOffset_m = P.g/P.contactOmega^2;    % contact surface raised by the static squash, so items sit on the belt
P.itemDensity_kgm3 = P.itemMass_kg/P.itemSize_m^3;
P.zWait_m    = P.beltTop_m + P.itemSize_m/2 + P.dropHeight_mm/1000;
P.dropTimes_s = P.firstDrop_s + P.dropEvery_s*(0:P.nItems-1);   % first drop of each item
P.cycle_s     = P.nItems*P.dropEvery_s;                         % each item is dropped again every cycle
P.handTime_s  = min(1, 0.25*P.cycle_s);                         % last part of its cycle: carried back to the chute
nStep = P.controlPeriod_s/P.fixedStep_s;
if abs(nStep - round(nStep)) > 1e-9 || round(nStep) < 1
    error('conveyor_belt_params:controlPeriod', ...
        'controlPeriod_s = %g must be a whole number of %g s steps (e.g. 0.01, 0.02, 0.03).', P.controlPeriod_s, P.fixedStep_s);
end
P.openPulseEnd_s = P.openPulseStart_s + P.openPulseWidth_s;
P.encWindow      = min(max(round(P.encWindow), 1), 40);
P.encDecode      = min(max(round(P.encDecode), 1), 4);
P.countsPerMotorRev = P.encPPR*P.encDecode;               % 44 with x4 decoding
P.countsPerRollerRev = P.countsPerMotorRev*P.gearRatio;   % 440
P.encNoiseCov    = P.encNoise_counts^2*P.controlPeriod_s;
P.mmPerCount     = 2*pi*P.rEff_m*1000/P.countsPerRollerRev;  % belt travel per encoder count (0.227 mm)
P.dragNoiseCov   = P.dragNoise_N^2*P.dragNoiseTs_s;
P.delayBlock_s   = max(P.delay_s, 0);
end
