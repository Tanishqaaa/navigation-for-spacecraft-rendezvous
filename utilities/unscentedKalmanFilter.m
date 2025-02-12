classdef unscentedKalmanFilter
    properties
        % state vector =[position(3),velocity(3),quaternion(4),angularVelocity(3),acc_bias (3),gyro_bias(3),gps_bias(clock-1),lidar_bias(3),star_bias(3)]
        x
        % state covariance matrix
        P
        % process noise covariance matrix
        Q
        % sensor/ measurement noise covaraince matrix
        R
        % UKF paramters
        alpha
        beta
        kappa
        % time step
        dt

    end
    methods
        function obj = unscentedKalmanFilter(stateVector, stateCovariance, processNoise, measurementNoise, time)
            obj.x=stateVector;
            obj.P=stateCovariance;
            obj.Q=processNoise;
            obj.R=measurementNoise;
            obj.dt=time;
            obj.alpha=1e-3;
            obj.beta=2;
            obj.kappa=0;            
        end
        function [xUpdate,pUpdate]=updateStep(obj,z)

            % Generate sigma points
            [sigmaPoints,weights] = generateSigmaPoints(obj);

            % Predict
            [xPred, covPred]= predictStep(obj,sigmaPoints,weights);

            % Update
            [xUpdate,pUpdate] = correctStep(obj,sigmaPoints,weights,xPred,covPred,z);

        end
        function [sigmaPoints,weights]=generateSigmaPoints(obj)

            % compute scaling parameter
            n = lenght(obj.x);
            lambda = obj.alpha^2 * (n + obj.kappa) - n;
    
            % compute cholesky decompistion
            S = sqrt((n+lambda)*obj.P); %check if right

            % generate sigma points
            sigmaPoints=zeros(n,2*n+1);
            sigmaPoints(:,1)=obj.x;
            for i = 1: n
                sigmaPoints(:,i+1) = obj.x + S(:,i);
                sigmaPoints(:,i+n+1)= obj.x - S(:,i); 
            end

            % assign weights to sigma points
            weights = zeros(1,2*n+1);
            weights(1)= lambda / (n + lambda);
            weights(2:end) = 1 / (2 * (n + lambda));

        end
        function [xPred, covPred]= predictStep(obj,sigmaPoints,weights)
            n = length(obj.x);
            numberSigmaPoints = size(sigmaPoints,2);
            xPred = zeros(n,1);
            % compute mean
            for i = 1:numberSigmaPoints
                xPred = xPred + weights(i) * obj.processModel(sigmaPoints(:,i));                
            end
            % compute covariance
            covPred = zeros(n,n);
            for i = 1: numberSigmaPoints
                dx = obj.processModel(sigmaPoints(:,i)) - xPred;
                covPred = covPred + weights(i) * (dx *dx') ;
            end
            covPred = covPred + obj.Q;
        end
        function [xUpdate,pUpdate] = correctStep(obj,sigmaPoints,weights,xPred,covPred,z)
            n = length(obj.x);
            numberSigmaPoints = size(sigmaPoints,2);
            m = lenght(z);
            % Transform sigma points through the measurement function
            predZ = zeros(m,numberSigmaPoints);
            for i= 1 : numberSigmaPoints
                predZ(:,i) = obj.measurementModel(sigmaPoints(:,i));
            end
            % compute the predicted measurement mean
            predMeasure = zeros(m,1);
            for i = 1:numberSigmaPoints
                predMeasure = predMeasure + weights(i) * predZ(:, i);
            end
            % compute the measurement covariance Pzz and cross covariance
            % Pxz
            Pzz = zeros(m, m);
            Pxz = zeros(n, m);
            for i = 1:numberSigmaPoints
                dz = predZ(:, i) - predMeasure;
                dx = obj.processModel(sigmaPoints(:, i)) - xPred;
                Pzz = Pzz + weights(i) * (dz * dz');
                Pxz = Pxz + weights(i) * (dx * dz');
            end
            Pzz = Pzz + obj.R;
            % compute the Kalman gain
            K = Pxz / Pzz;
            % z is the actual measurement
            xUpdate = xPred + K * (z - predMeasure);
            pUpdate = covPred - K * Pzz * K';           

        end
        function [xNext] = processModel(obj,x)
        % orbit dynamics & attitude dynamics - changes based on the range / phase
        % from 500 km - 5 km
        % orbital elements and pertubation equation
        % attitude dynamics with flexible panels

         % Extract state variables
        r = x(1:3);  % Position (m)
        v = x(4:6);  % Velocity (m/s)
        q = x(7:10); % Quaternion (attitude)
        w = x(11:13); % Angular velocity (rad/s)
        acc_bias = x(14:16); % Accelerometer bias
        gyro_bias = x(17:19); % Gyro bias
        gps_bias = x(20); % GPS clock bias
        lidar_bias = x(21:23); % Lidar bias
        star_bias = x(24:26); % Star tracker bias
    
        % Constants
        mu = 3.986e14; % Earth's gravitational parameter (m^3/s^2)
        Re = 6378.137e3; % Earth's radius (m)
        J2 = 1.08263e-3; % J2 perturbation coefficient
        Cd = 2.2; % Drag coefficient
        A = 1.0; % Cross-sectional area (m²)
        m = 500; % Satellite mass (kg)
        H = 7500; % Scale height (m)

        r_norm = norm(r);

        % Compute two body acceleration
        a_gravity = -mu * r / r_norm^3;  % Two-body gravitational acceleration

        % Compute J2 acceleration (check J2 equation)
        factor = (3/2) * J2 * (mu / r_norm^2) * (Re^2 / r_norm^2);        
        ax_J2 = factor * (r(1)/r_norm) * (5 * (r(3)/r_norm)^2 - 1);
        ay_J2 = factor * (r(2)/r_norm) * (5 * (r(3)/r_norm)^2 - 1);
        az_J2 = factor * (r(3)/r_norm) * (5 * (r(3)/r_norm)^2 - 3);
        a_perturb = [ax_J2; ay_J2; az_J2]; % J2 perturbation acceleration

        % For acceleration due to drag
        % Compute atmospheric density (exponential model)
        h = r_norm - Re; % Altitude (m)
        rho_0 = 3.9e-10; % Reference density at 200 km (kg/m³)
        rho = rho_0 * exp(-(h - 200e3) / H);

        % Compute relative velocity w.r.t atmosphere
        v_rel = v; % Assuming non-rotating atmosphere (can be modified)
    
        % Compute drag acceleration
        a_drag = -0.5 * Cd * (A/m) * rho * norm(v_rel) * v_rel;

        % Total acceleration      
        a_total = a_gravity + a_perturb + a_drag - acc_bias; % Include bias
    
        % Update position and velocity using 
        % r_next = r + v * obj.dt;
        % v_next = v + a_total * obj.dt;
    
        % Corrected angular velocity (gyro bias correction)
        omega = w - obj.gyro_bias; 
        
        % Skew-symmetric matrix for angular velocity
        Omega = [ 0,   -omega(1), -omega(2), -omega(3);
                  omega(1),  0,    omega(3), -omega(2);
                  omega(2), -omega(3), 0,    omega(1);
                  omega(3),  omega(2), -omega(1), 0 ];
    
        % Define the quaternion differential equation
        q_dot = @(q) 0.5 * Omega * q; % Quaternion rate of change
        
        % Step size for Runge-Kutta method
        dt = obj.dt;
    
        % RK4 integration
        k1 = q_dot(q); 
        k2 = q_dot(q + 0.5 * k1 * dt); 
        k3 = q_dot(q + 0.5 * k2 * dt); 
        k4 = q_dot(q + k3 * dt); 
    
        % Update the quaternion using RK4
        q_next = q + (k1 + 2*k2 + 2*k3 + k4) * dt / 6;
        
        % Normalize quaternion to avoid drift
        q_next = q_next / norm(q_next);
    
        % Angular velocity remains the same (no external torques assumed)
        w_next = w; 
    
        % Bias evolution as a random walk (check bias values)
        acc_bias_next = acc_bias + randn(3,1) * 1e-6 * obj.dt; % Small noise
        gyro_bias_next = gyro_bias + randn(3,1) * 1e-6 * obj.dt;
        gps_bias_next = gps_bias + randn * 1e-9 * obj.dt;
        lidar_bias_next = lidar_bias + randn(3,1) * 1e-6 * obj.dt;
        star_bias_next = star_bias + randn(3,1) * 1e-6 * obj.dt;
    
        % Concatenate updated state
        xNext = [r_next; v_next; q_next; w_next; acc_bias_next; gyro_bias_next; gps_bias_next; lidar_bias_next; star_bias_next];

        % from 5 km to docking 2 m
        % Clohessy Wiltshire equation
        % attitude dynamics with flexible panels    

        % from 2 m - docking
        % Clohessy Wiltshire equation
        % relative attitude dynamics

        end

        function [zNext] = measurementModel(obj,x)
        % changes based on the sensor

        % GPS

        % LIDAR

        % Star tracker

        % IMU

        % Optical sensor

        end

    end

end