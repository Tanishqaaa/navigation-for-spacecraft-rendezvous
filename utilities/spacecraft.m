classdef spacecraft
    properties
        position
        velocity
        attitude
        angularVelocity
        accBias 
        gyroBias
        gpsBias
        lidarBias
        starBias
    end
    methods
        function obj = spacecraft(stateVector)
            obj.position = stateVector(1:3);
            obj.velocity = stateVector(4:6);
            obj.attitude = stateVector(7:10);
            obj.angularVelocity = stateVector(11:13);
            obj.accBias = stateVector(14:16);
            obj.gyroBias = stateVector(17:19);
            obj.gpsBias = stateVector(20); 
            obj.lidarBias = stateVector(21:23);
            obj.starBias = stateVector(24:26);

        end
        function obj=propogateState(obj,dt)
        % orbit dynamics & attitude dynamics - changes based on the range / phase
        % from 500 km - 5 km
        % orbital elements and pertubation equation
        % attitude dynamics with flexible panels

         % Extract state variables for simplicity
        r = obj.position;  % Position (m)
        v = obj.velocity;  % Velocity (m/s)
        q = obj.attitude; % Quaternion (attitude)
        w = obj.angularVelocity; % Angular velocity (rad/s)        
    
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
        aTotal = a_gravity + a_perturb + a_drag - obj.accBias; % Include bias
    
        % Update position and velocity using 
        % Simple RK4 integration (check)
        k1_pos = v;
        k1_vel = aTotal;
        
        k2_pos = v + 0.5*dt*k1_vel;
        k2_vel = -mu * (r + 0.5*dt*k1_pos) / norm(r + 0.5*dt*k1_pos)^3;
        
        k3_pos = v + 0.5*dt*k2_vel;
        k3_vel = -mu * (r + 0.5*dt*k2_pos) / norm(r + 0.5*dt*k2_pos)^3;
        
        k4_pos = v + dt*k3_vel;
        k4_vel = -mu * (r + dt*k3_pos) / norm(r + dt*k3_pos)^3;
        
        obj.position = r + (dt/6)*(k1_pos + 2*k2_pos + 2*k3_pos + k4_pos);
        obj.velocity = v + (dt/6)*(k1_vel + 2*k2_vel + 2*k3_vel + k4_vel);
    
        % Corrected angular velocity (gyro bias correction)
        omega = w - obj.gyroBias; 
        
        % Skew-symmetric matrix for angular velocity
        Omega = [ 0,   -omega(1), -omega(2), -omega(3);
                  omega(1),  0,    omega(3), -omega(2);
                  omega(2), -omega(3), 0,    omega(1);
                  omega(3),  omega(2), -omega(1), 0 ];
    
        % Define the quaternion differential equation
        q_dot = @(q) 0.5 * Omega * q; % Quaternion rate of change        
    
        % RK4 integration
        k1 = q_dot(q); 
        k2 = q_dot(q + 0.5 * k1 * dt); 
        k3 = q_dot(q + 0.5 * k2 * dt); 
        k4 = q_dot(q + k3 * dt); 
    
        % Update the quaternion using RK4
        q_next = q + (k1 + 2*k2 + 2*k3 + k4) * dt / 6;
        
        % Normalize quaternion to avoid drift
        obj.attitude = q_next / norm(q_next);
    
        % Angular velocity remains the same (no external torques assumed)
        %w_next = w; 
    
        % Bias evolution as a random walk (check bias values)
        obj.accBias = obj.accBias + randn(3,1) * 1e-6 * dt; % Small noise
        obj.gyroBias = obj.gyroBias + randn(3,1) * 1e-6 * dt;
        obj.gpsBias = obj.gpsBias + randn * 1e-9 * dt;
        obj.lidarBias = obj.lidarBias + randn(3,1) * 1e-6 * dt;
        obj.starBias = obj.starBias + randn(3,1) * 1e-6 * dt;

        end

    end
       
end