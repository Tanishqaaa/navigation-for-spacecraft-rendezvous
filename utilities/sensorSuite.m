classdef sensorSuite
    properties
        starAttitude
        lidarDistance
        lidarDirection
        gyroAnguarlVelocity
        accAcceleration
        gpsPosition
        gpsVelocity
        last_measurement_time
    end
    
    methods
        function obj = sensorSuite()
            obj.last_measurement_time = 0;
            % Initialize sensor properties
        end
        
        function measurements = getMeasurements(obj)
            % Simulate sensor measurements
            % In real implementation, would:
            % 1. Read from actual hardware
            % 2. Apply calibration
            % 3. Convert to engineering units
            % 4. Check validity
            
            % Simulated GPS measurement
            % gps_pos = randn(3,1) * 0.01;  % 10m position noise
            % gps_vel = randn(3,1) * 0.001; % 1mm/s velocity noise
            % 
            % % Simulated attitude measurement from star tracker
            % attitude_noise = randn(4,1) * 0.001; % 0.001 rad attitude noise
            % 
            % % Simulated angular velocity from IMU
            % omega_noise = randn(3,1) * 0.0001; % 0.0001 rad/s angular velocity noise
            
            measurements = [gpsPosition,gpsVelocity,lidarDistance,lidarDirection,accAcceleration,gyroAnguarlVelocity,starAttitude]; %should be same order as in sensor model
        end
    end
end
