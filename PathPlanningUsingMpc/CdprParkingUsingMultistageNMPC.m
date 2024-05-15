%% CDPR Path Planning Using Multistage Nonlinear MPC
% This example shows how to use multistage nonlinear model predictive
% control (MPC) as a path planner to find the optimal trajectory for CDPR
% in the presence of static obstacles.

%% Overview
% An MPC controller uses an internal model to predict plant behavior. Given
% the current states of the plant, based on the prediction model, MPC finds
% an optimal control sequence that minimizes cost and satisfies the
% constraints specified across the prediction horizon. Because MPC finds
% the plant future trajectory at the same time, it can work as a powerful
% tool to solve trajectory optimization problems, such as autonomous
% parking of a vehicle and motion planning of a robot arm.
%
% In such trajectory optimization problems the plant, cost function, and
% constraints can often be nonlinear. Therefore, you need to use nonlinear
% MPC controller for problem formulation and solution. In this example, you
% design a nonlinear MPC controller that finds an optimal route to
% automatically park CDPR from its initial position to its target position,
% which is between two static obstacles. You can then pass the generated
% path to a low-level controller as a reference signal, so that it can
% execute the parking maneuver in real time.
%
% This example requires Optimization Toolbox(TM) and Robotics System
% Toolbox(TM) software.

%% CDPR System
% The following figure shows the kinematics model of the CDPR system.
%
% <<../kinematics model with rear axle as origin.png>>
%
%%
% The states for this model are:
%
% # |x| (center of the CDPR rear axle, global x position)
% # |y| (center of the CDPR rear axle, global y position)
% # |theta| (CDPR orientation, global angle, 0 = east)
%
% The inputs for this model are:
%
% # |alpha| (CDPR steering angle)
% # |v| (CDPR longitudinal velocity)
%
% For this model, length and position are in meters, velocity is in m/s,
% and angles are in radians.
%
close all, fclose all;
delete("result.csv");
%%
% Define the following model parameters.
%
% * |L| (cdpr length)
% * |W| (cdpr width)
% * |Lwheel| (wheel diameter)
% * |Wwheel| (wheel width)
params = struct('L',1.114,...
    'W',1.09,...
    'Lwheel',0.235,...
    'Wwheel',0.05);
enable_obs = 1;

%%
% The nonlinear model is implemented in the |CdprStateFcn| function and its
% manually-derived analytical Jacobian (which is used to speed up
% optimization) is in the |CdprStateJacobianFcn| function .
%
% In this example, since you use MPC as a path planner instead of a
% low-level path-following controller, the CDPR's longitudinal velocity is
% used as one of the manipulated variables instead of the acceleration.

%% Path Planning Problem
% The goal is to find a viable path that brings the CDPR system from any
% initial position to the target position (the green cross in the following
% figure) in 20 seconds using reverse parking. In the process, the planned
% path must avoid collisions with obstacles.
if ~enable_obs
    initialPose = [-3;0;0;0;0];
    targetPose = [1;1;pi/6;0;0];
else
    initialPose = [-4.5;0;0;0;0];
    targetPose = [4.5;0;0;0;0];
end
CdprPlot(enable_obs,initialPose,targetPose,params);

%%
% The initial plant inputs (steering angle and longitudinal velocity) are
% both |0|.
u0 = zeros(2,1);

%%
% The initial position must be valid. Use the inequality constraint
% function |CdprIneqConFcn| to check for validity. Details about this
% function are discussed in the next section. Here, collision detection is
% carried out by specific Robotics System Toolbox functions.
cineq = CdprIneqConFcn(1,initialPose,u0,[params.L;params.W]);
if any(cineq>0)
    fprintf('Initial pose is not valid.\n');
    return
end

%% Path Planning Using Multistage Nonlinear MPC
% Compared with the generic nonlinear MPC controller (|nlmpc| object),
% multistage nonlinear MPC provides you with a more flexible and efficient
% way to implement MPC with staged costs and constraints. This flexibility
% and efficiency is especially useful for trajectory planning.
%
% A multistage nonlinear MPC controller with prediction horizon |p| defines
% |p+1| stages, representing time |k| (current time), |k+1|, ..., |k+p|.
% For each stage, you can specify stage-specific cost, inequality
% constraint, and equality constraint functions. These functions depend
% only on the plant state and input values at that stage. Given the current
% plant states |x[k]|, MPC finds the manipulated variable (MV) trajectory
% (from time |k| to |k+p-1|) to optimize the summed stage costs (from time
% |k| to |k+p|), satisfying all the stage constraints (from time |k| to
% |k+p|).
%
% In this example, the plant has four states and two inputs (both MVs).
% Choose the prediction horizon |p| and sample time |Ts| such that p*Ts =
% 20.

% Create the multistage nonlinear MPC controller.
p = 300;
nlobj = nlmpcMultistage(p,5,2);
nlobj.Ts = 0.05;

%%
% Specify the prediction model and its analytical Jacobian in the
% controller object. Since the model requires three parameters (|M|, |L1|,
% and |L2|), set |Model.ParameterLength| to |3|.
nlobj.Model.StateFcn = "CdprStateFcn";
nlobj.Model.StateJacFcn = "CdprStateJacobianFcn";
nlobj.Model.ParameterLength = 1;

%%
% Specify hard bounds on the two manipulated variables. The steering angle
% must remain in the range +/- 45 degrees. The maximum forward speed is 2
% m/s and the maximum reverse speed is 2 m/s.
nlobj.States(4).Min = -2;   % Minimum steering angle
nlobj.States(4).Max = 2;    % Maximum steering angle
nlobj.States(5).Min = -pi/4;    % Minimum velocity (reverse)
nlobj.States(5).Max = pi/4;     % Maximum velocity (forward)


nlobj.MV(1).Min = -3;     % Minimum steering angular velocity
nlobj.MV(1).Max =  3;     % Maximum steering angular velocity
nlobj.MV(2).Min = -3;       % Minimum acceleration 
nlobj.MV(2).Max =  3;       % Maximum acceleration

nlobj.MV(1).RateMax = 1;  % Maximum steering angle increment
nlobj.MV(2).RateMax = 1;  % Maximum velocity increment

%%
% You can use different ways to define the cost terms. For example, you
% might want to minimize time, fuel consumption, or speed. For this
% example, minimize speed and steering angle in the quadratic format to
% promote safety.
%
% Since MVs are only valid from stage 1 to stage |p|, you do not need to
% define any stage cost for the last stage, |p+1|. The five model settings,
% |L|, |W|are stage parameters for stages 1 to |p| and are used by the
% inequality constraint functions.
for ct=1:p
    nlobj.Stages(ct).CostFcn = "CdprCost";
    nlobj.Stages(ct).CostJacFcn = "CdprCostGradientFcn";
    nlobj.Stages(ct).ParameterLength = 2;
end

%%
% You can use inequality constraints to avoid collision during the parking
% process. Use the |CdprIneqConFcn| function to check whether CDPR collide
% with any of the two static obstacles at a specific stage. When either the
% CDPR gets within the 1 m safety zone around the obstacles, a collision is
% detected.
%
% In general, check for such collisions for all the prediction steps (from
% stage 2 to stage |p+1|). In this example, however, you only need to check
% stages from 2 to |p| because the last stage is already taken care of by
% the equality constraint function.
%
% For this example, do not provide an analytical Jacobian for the
% inequality constraint functions because it is too complicated to derive
% manually. Therefore, the controller uses a finite-difference method
% (numerical perturbation) to estimate the Jacobian at run time.
if enable_obs
    for ct=2:p
        nlobj.Stages(ct).IneqConFcn = "CdprIneqConFcn";
    end
end
%%
% Use terminal state for the last stage to ensure successful parking at the
% target position. In this example, the target position is provided as a
% run-time signal.  Here we use a dummy finite value to let MPC know which
% states will have terminal values at run time.
nlobj.Model.TerminalState = zeros(5,1);

%%
% At the end of multistage nonlinear MPC design, you can use the
% |validateFcns| command with random initial plant states and inputs to
% check whether any of the user-defined state, cost, and constraint
% function as well as any analytical Jacobian function, has a problem.
%
% You must provide all the defined state functions and stage parameters to
% the controller at run time. |StageParameter| contains all the stage
% parameters stacked into a single vector.  We also use |TerminalState| to
% specify terminal state at run time.
simdata = getSimulationData(nlobj,'TerminalState');
simdata.StateFcnParameter = params.L;
simdata.StageParameter = repmat([params.L;params.W],p,1);
simdata.TerminalState = targetPose;
validateFcns(nlobj,[2;3;0.5;0;0],[0.1;0.2],simdata);

%%
% Since the default nonlinear programming solver |fmincon| searches for a
% local minimum, you must provide a good initial guess for the decision
% variables, especially for trajectory optimization problems that usually
% involve a complicated (likely nonconvex) solution space.
%
% This example has 244 decision variables, the plant states and inputs (6
% in total) for each of the first p (40) stages and plant states (4) for
% the last stage |p+1|. The |CdprInitialGuess| function uses simple
% heuristics to generate the initial guess. The initial guess is displayed
% as dots in the following animation plot.
[simdata.InitialGuess, XY0] = CdprInitialGuess(enable_obs,initialPose,targetPose,u0,p);

%% Trajectory Planning and Simulation Result
% Use the |nlmpcmove| function to find the optimal parking path, which
% typically takes ten to twenty seconds, depending on the initial position.
fprintf('Path Planner is running...\n');
tic;[~,~,info] = nlmpcmove(nlobj,initialPose,u0,simdata);t=toc;
fprintf('Calculation Time = %s; Objective cost = %s; ExitFlag = %s; Iterations = %s\n',...
    num2str(t),num2str(info.Cost),num2str(info.ExitFlag),num2str(info.Iterations));

%% animate and plot
% Two plots are generated. One is the animation of the parking process,
% where blue circles indicate the optimal path and the initial guess is
% shown as a dot. The other displays the optimal trajectory of plant states
% and control moves.
CdprPlot(enable_obs, initialPose, targetPose, params, info, XY0);

%% analyze results
analyzeResults(info, targetPose, t);

%% calculate and save alpha&v for each wheelset
num = size(info.MVopt, 1);
alpha = zeros(num, 4);
v = zeros(num, 4);
alpha(:, 1) = info.Xopt(:, 5);
alpha(:, 2) = info.Xopt(:, 5);
v(:, 1) = info.Xopt(:, 4)./cos(alpha(:, 1));
v(:, 2) = info.Xopt(:, 4)./cos(alpha(:, 2));
v(:, 3) = info.Xopt(:, 4);
v(:, 4) = info.Xopt(:, 4);
writematrix([alpha v],'result.csv','WriteMode','append');

%%
% You can try other initial X-Y positions in the Path Planning Problem
% section by changing the first two parameters of |initialPose|, as long as
% the positions are valid.
%
% If |ExitFlag| is negative, the nonlinear MPC controller fails to find an
% optimal solution and you cannot trust the returned trajectory. In that
% case, you might need to provide a better initial guess and specify it in
% |simdata.InitialGuess| before calling |nlmpcmove|.
