classdef unscentedKalmanFilter
    properties
        % state vector =[position(3),velocity(3),quaternion(4),angularVelocity(3),accBias (3),gyroBias(3),gpsBias(clock-1),lidarBias(3),starBias(3)]
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
            obj.x=spacecraft(stateVector);
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

            % Propagate sigma points
            propagated_points = zeros(numberSigmaPoints);
            for i = 1:numberSigmaPoints
                temp_state = SpacecraftState(sigmaPoints(:,i));
                temp_state = temp_state.propagateState(obj.dt);
                propagated_points(:,i) = [temp_state.position; temp_state.velocity;
                                        temp_state.attitude; temp_state.angular_velocity;temp_state.accBias;temp_state.gyroBias;temp_state.gpsBias;temp_state.lidarBias;temp_state.starBias];
            end

            % compute mean
            for i = 1:numberSigmaPoints
                xPred = xPred + weights(i) * propagated_points(:,i);                
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
                dx = propagated_points(:,i) - xPred;
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
      
        function [zNext] = measurementModel(obj, x)
        % Extract state variables
        position = obj.x(1:3);
        velocity = obj.x(4:6);
        quaternion = obj.x(7:10);
        angularVelocity = obj.x(11:13);
        accBias = obj.x(14:16);
        gyroBias = obj.x(17:19);
        gpsBias = obj.x(20);
        lidarBias = obj.x(21:23);
        starBias = obj.x(24:26);
    
        % GNSS Measurement Model 
        % GNSS measures receiver clock bias
        % 1 m position correction - standard
        % 0.05 m/s doppler based velocity erros
        gps_measurement = [position + gpsBias + randn(3,1) * 1; velocity + randn(3,1) * 0.005]; % GPS noise
    
        % LIDAR Measurement Model
        % LIDAR provides range and direction, affected by misalignment
        % 1 mrad misalignment error
        % position error of 5 cm
        lidar_rotation_error = eye(3) + diag(randn(3,1) * 0.001); % Small misalignment matrix
        lidar_measurement = lidar_rotation_error * (position + lidarBias + randn(3,1) * 0.05); 
    
        % Star Tracker Measurement Model 
        % Star tracker measures attitude (quaternion) with bias and noise
        star_tracker_measurement = quaternion + starBias + randn(4,1) * 0.001;
    
        % IMU Measurement Model 
        % IMU gives angular velocity and acceleration with drift & noise
        % 0.1 % scale factor in gyro
        % 0.01 m/s2 acc bias noise
        imu_measurement = [(angularVelocity + gyroBias) * (1 + randn(1) * 0.001); % Gyro bias & scale error
                           accBias + randn(3,1) * 0.01]; % Accelerometer bias & noise
    
        % --- Sensor Fusion Output ---
        zNext = [gps_measurement; lidar_measurement; star_tracker_measurement; imu_measurement];
    
        end

    end

end