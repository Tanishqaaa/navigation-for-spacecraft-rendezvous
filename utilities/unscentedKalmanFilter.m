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