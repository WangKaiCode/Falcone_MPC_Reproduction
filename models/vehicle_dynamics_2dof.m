% =========================================================
% Function: vehicle_dynamics_2dof
%
% Purpose:
%   Calculate the lateral and yaw dynamics of a vehicle
%   using the linear 2-DOF bicycle model.
%
% Reference:
%   Rajamani, Vehicle Dynamics and Control, 2nd Edition
%   Section 2.3
%
% Inputs:
%   state      - Current vehicle states [vy; r]
%   delta_f    - Front wheel steering angle (rad)
%   p          - Vehicle parameter structure
%
% Outputs:
%   dstate     - State derivatives [dvy; dr]
% =========================================================

% Step 1: Extract current vehicle states
%   Get lateral velocity vy
%   Get yaw rate r

% Step 2: Extract vehicle parameters
%   Get mass m
%   Get yaw moment of inertia Iz
%   Get front axle distance lf
%   Get rear axle distance lr
%   Get front cornering stiffness Caf
%   Get rear cornering stiffness Car
%   Get longitudinal velocity vx

% Step 3: Calculate front and rear wheel velocity angles
%   Calculate front wheel velocity angle theta_vf
%   Calculate rear wheel velocity angle theta_vr
%   Use the small-angle approximation

% Step 4: Calculate front and rear tire slip angles
%   alpha_f = front steering angle - front velocity angle
%   alpha_r = - rear velocity angle

% Step 5: Calculate lateral tire forces
%   Calculate front axle lateral force Fyf
%   Calculate rear axle lateral force Fyr
%   Use the linear tire cornering stiffness model

% Step 6: Calculate lateral acceleration
%   Apply Newton's second law in the lateral direction
%   Calculate dvy

% Step 7: Calculate yaw angular acceleration
%   Apply moment balance around the vehicle CG
%   Calculate dr

% Step 8: Assemble the state derivatives
%   Return [dvy; dr]