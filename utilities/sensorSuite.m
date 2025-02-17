classdef sensorSuite
    properties
        starAttitude
        lidarDistance
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
            
            gps_measurement = [gpsPosition;gpsVelocity];
            lidar_measurement = [lidarDistance];
            star_tracker_measurement = [starAttitude];
            imu_measurement = [gyroAngularVelocity;accAcceleration];
            
            measurements = [gps_measurement; lidar_measurement; star_tracker_measurement; imu_measurement]; %should be same order as in sensor model
        end
    end
end
