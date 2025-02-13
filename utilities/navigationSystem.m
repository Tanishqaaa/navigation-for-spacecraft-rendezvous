classdef navigationSystem
    properties
    spacecraft
    ukf
    sensor

    end
    methods
        function obj = navigationSystem(initialState)
            obj.spacecraft = spacecraft(initialState);
            obj.ukf = unscentedKalmanFilter(); %give it parameters
            obj.sensor = sensorSuite();

        end
        function [state,covariance] = performNavigation(obj)
            % get sensor measurements
            measurements = obj.sensor.getMeasurements();
            % get the states
            [xUpdate,pUpdate]=obj.ukf.updateStep(measurements);

        end
    end
end