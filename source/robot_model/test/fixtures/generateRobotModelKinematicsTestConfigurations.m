function generateRobotModelKinematicsTestConfigurations(urdf, nConfigurations, seed)
% Generate configurations and expected results for a robot model
%
%   generateRobotModelKinematicsTestConfigurations expects a file 
%   `panda_arm.urdf` in the same folder and generates 3 random 
%   configurations of joint position and velocity. It calculates the 
%   forward position and velocity kinematics for the end effector and 
%   link 4 and prints out a C++ formatted result to be used in test 
%   fixtures.
%
%   generateRobotModelKinematicsTestConfigurations(urdf) takes a filepath 
%   to a urdf robot model to load.
%
%   generateRobotModelKinematicsTestConfigurations(urdf, nConfigurations) 
%   generates a specified number of random configurations (default 3).
%
%   generateRobotModelKinematicsTestConfigurations(..., seed) allows
%   setting the seed for the random number generator. The input is either
%   'default' for repeatable behaviour or 'shuffle' for a time-based seed.
%
%   Requires Robotics System Toolbox

arguments
    urdf string = 'panda_arm.urdf';
    nConfigurations (1,1) double = 3;
    seed (1,:) char {mustBeMember(seed,{'default','shuffle'})} = 'default'
end

% initialise the random number generator
rng(seed)
disp(rng)

% load the robot model
robot = importrobot(urdf);
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% helper function to format numeric array [a, b, c] into '{a, b, c}'
num2stdvec = @(A) (['{', strjoin(compose('%f', A), ', ') '}']);

% code generation (declarations)
fprintf('std::vector<state_representation::JointState> test_configs;\n');
fprintf('std::vector<Eigen::MatrixXd> test_jacobian_ee_expects;\n');
fprintf('std::vector<state_representation::CartesianPose> test_fk_ee_expects;\n');
fprintf('std::vector<state_representation::CartesianPose> test_fk_link4_expects;\n');
fprintf('std::vector<state_representation::CartesianTwist> test_velocity_fk_expects;\n');
fprintf('std::vector<Eigen::MatrixXd> test_damped_jacobian_ee_expects;\n');
fprintf('Eigen::Matrix<double, 6, 1> twist;\n');
fprintf('Eigen::Matrix<double, 7, 1> joint_vel_damped;\n');
fprintf('std::vector<double> test_dls_lambdas;\n');
fprintf('std::vector<state_representation::CartesianTwist> test_ee_velocities;\n');
fprintf('std::vector<state_representation::JointVelocities> test_velocity_damped_ik_expects;\n');

% generate the configurations
for conf = 1:nConfigurations
    % configurations
    q = robot.randomConfiguration;
    v = rand(size(q))*2 - 1;
    v_ee = rand(6,1)*2 - 1;
    dls_lambda = 10^(-rand());
    
    % transforms
    ee_T = robot.getTransform(q, 'panda_link8');
    link4_T = robot.getTransform(q, 'panda_link4');
    
    % jacobian
    ee_jac = robot.geometricJacobian(q, 'panda_link8');
    ee_jac_r(1:3,:) = ee_jac(4:6,:);
    ee_jac_r(4:6,:) = ee_jac(1:3,:);
    ee_jac_damped = pinv(pseudoInverseMat(ee_jac_r, dls_lambda));
    
    % expected results
    ee_pose = [tform2trvec(ee_T), tform2quat(ee_T)]; % (1x7)
    link4_pose = [tform2trvec(link4_T), tform2quat(link4_T)]; % (1x7)
    ee_twist = ee_jac * v'; % (1x6)
    joint_vel_damped = ee_jac_damped\v_ee; % (1x7)
    
    % code generation (definitions)
    fprintf('\n// Random test configuration %i:\n', conf);
    fprintf(['state_representation::JointState config%i', ...
        '(franka->get_robot_name(), franka->get_joint_frames());\n'], ...
        conf);
    fprintf('config%i.set_positions(std::vector<double>%s);\n', conf, num2stdvec(q));
    fprintf('config%i.set_velocities(std::vector<double>%s);\n', conf, num2stdvec(v));
    fprintf('state_representation::CartesianTwist test_ee_velocity%i("franka");\n', conf);
    fprintf('test_ee_velocity%i.set_data(std::vector<double>%s);\n', conf, num2stdvec(v_ee));
    fprintf('test_ee_velocities.push_back(test_ee_velocity%i);\n', conf);
    fprintf('test_dls_lambdas.push_back(%f);\n', dls_lambda);
    fprintf('test_configs.push_back(config%i);\n', conf);
    
    fprintf('\n// Expected results for configuration %i:\n', conf);
    
    % populate Jacobian matrix (note that the robotics toolbox puts angular
    % first, so the order needs to be swapped)
    fprintf('Eigen::MatrixXd jac%i(6, 7);\n', conf);
    fprintf('jac%i << %f, %f, %f, %f, %f, %f, %f, \n', conf, ee_jac(4, :));
    fprintf('\t%f, %f, %f, %f, %f, %f, %f, \n', ee_jac(5, :));
    fprintf('\t%f, %f, %f, %f, %f, %f, %f, \n', ee_jac(6, :));
    fprintf('\t%f, %f, %f, %f, %f, %f, %f, \n', ee_jac(1, :));
    fprintf('\t%f, %f, %f, %f, %f, %f, %f, \n', ee_jac(2, :));
    fprintf('\t%f, %f, %f, %f, %f, %f, %f; \n', ee_jac(3, :));
    fprintf('test_jacobian_ee_expects.emplace_back(jac%i);\n', conf);
    
    % populate damped Jacobian matrix
    fprintf('Eigen::MatrixXd damped_jac%i(6, 7);\n', conf);
    fprintf('damped_jac%i << %f, %f, %f, %f, %f, %f, %f, \n', conf, ee_jac_damped(4, :));
    fprintf('\t%f, %f, %f, %f, %f, %f, %f, \n', ee_jac_damped(5, :));
    fprintf('\t%f, %f, %f, %f, %f, %f, %f, \n', ee_jac_damped(6, :));
    fprintf('\t%f, %f, %f, %f, %f, %f, %f, \n', ee_jac_damped(1, :));
    fprintf('\t%f, %f, %f, %f, %f, %f, %f, \n', ee_jac_damped(2, :));
    fprintf('\t%f, %f, %f, %f, %f, %f, %f; \n', ee_jac_damped(3, :));
    fprintf('test_damped_jacobian_ee_expects.emplace_back(damped_jac%i);\n', conf);
    
    % populate forward kinematics results
    fprintf(['test_fk_ee_expects.emplace_back(state_representation::CartesianPose' ...
        '("ee", Eigen::Vector3d(%f, %f, %f), Eigen::Quaterniond(%f, %f, %f, %f), ', ...
        'franka->get_base_frame()));\n'], ee_pose(1:3), ee_pose(4:7));
    fprintf(['test_fk_link4_expects.emplace_back(state_representation::CartesianPose' ...
        '("link4", Eigen::Vector3d(%f, %f, %f), Eigen::Quaterniond(%f, %f, %f, %f), ', ...
        'franka->get_base_frame()));\n'], link4_pose(1:3), link4_pose(4:7));
    
    % populate forward velocity results (note that the robotics toolbox
    % puts angular first, so the order needs to be swapped)
    fprintf('twist << %f, %f, %f, %f, %f, %f;\n', ee_twist([4:6, 1:3]));
    fprintf(['test_velocity_fk_expects.emplace_back', ...
        '(state_representation::CartesianTwist("ee", twist, franka->get_base_frame()));\n']);

    % populate damped inverse velocity results
    fprintf('joint_vel_damped << %f, %f, %f, %f, %f, %f, %f;\n', joint_vel_damped);
    fprintf(['test_velocity_damped_ik_expects.emplace_back', ...
        '(state_representation::JointVelocities("franka", joint_vel_damped));\n']);
end