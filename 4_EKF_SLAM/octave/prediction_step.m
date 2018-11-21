function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

N = size(mu,1);
old_mu = mu;

% Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)

motion = [u.t * cos(mu(3)+u.r1); u.t * sin(mu(3)+u.r1); normalize_angle(u.r1 + u.r2)];

mu = mu + eye(3,N)' * motion;
mu(3) = normalize_angle(mu(3));

% Compute the 3x3 Jacobian Gx of the motion model

Gx = eye(3) + [0, 0, -u.t * sin(old_mu(3)+u.r1);
               0, 0, u.t * cos(old_mu(3)+u.r1) ;
               0, 0, 0];

% Construct the full Jacobian G

G = eye(N);
G(1:3,1:3) = Gx;

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% Compute the predicted sigma after incorporating the motion

sigma = G * sigma * G' + R;

end
