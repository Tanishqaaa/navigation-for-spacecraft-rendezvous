clc;
clear;
close all;

% initialise the parameters
parameters;
navSystem = navigationSystem(stateVector,stateCovariance, measurementNoise, time);
startTime = 0;
endTime = 3600 ;
while startTime < endTime
    [updatedState,updatedCovariance] = navSystem.performNavigation();
    startTime = startTime + navSystem.dt;

end 

