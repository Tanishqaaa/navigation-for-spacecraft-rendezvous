classdef unscentedKalmanFilter
    properties
        % state vector =[position,velocity,quaternion,angular velocity]
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
                predMeasure = z_pred + weights(i) * predZ(:, i);
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
            xUpdate = xPred + K * (z - predMeasure);
            pUpdate = covPred - K * Pzz * K';           

        end
        function [xNext] = processModel(obj,x)
        % orbit dynamics & attitude dynamics - changes based on the range / phase
        % from 500 km - 5 km
        % orbital elements and pertubation equation
        % attitude dynamics with flexible panels


        % from 5 km to docking 2 m
        % Clohessy Wiltshire equation
        % attitude dynamics with flexible panels    

        % from 2 m - final approach
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