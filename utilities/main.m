clc;
clear;
close all;

parameters;

x0 = initialState;
P0 = stateCovariance;
Q = processNoise;
R = measurementNoise;
dt = time;
realMeasurements = z;

ukf = unscentedKalmanFilter(x0,P0,Q,R,dt);