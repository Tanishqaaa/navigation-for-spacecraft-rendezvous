clc;
clear;
close all;

% initialise the parameters
parameters;

% Not needed but can do to simplify variable names
x0 = initialState;
P0 = stateCovariance;
Q = processNoise;
R = measurementNoise;
dt = time;
realMeasurements = z;

% create an instance
ukf = unscentedKalmanFilter(x0,P0,Q,R,dt);

% create xUpdate and pUpdate
[xUpdate,pUpdate] = ukf.updateStep(ukf,realMeasurements);