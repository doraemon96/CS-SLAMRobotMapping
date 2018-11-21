function [mu, sigma, observedLandmarks] = correction_step(mu, sigma, z, observedLandmarks)
% Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model
% The employed sensor model measures the range and bearing of a landmark
% mu: 2N+3 x 1 vector representing the state mean.
% The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
% The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
% sigma: 2N+3 x 2N+3 is the covariance matrix
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.
% observedLandmarks(j) is false if the landmark with id = j has never been observed before.

% Number of measurements in this time step
m = size(z, 2); 
mu_size = size(mu, 1);

% Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
% expectedZ: vectorized form of all expected measurements in the same form.
% They are initialized here and should be filled out in the for loop below
Z = zeros(m*2, 1);
expectedZ = zeros(m*2, 1);

% Iterate over the measurements and compute the H matrix
% (stacked Jacobian blocks of the measurement function)
% H will be 2m x 2N+3
H = [];

for i = 1:m
	% Get the id of the landmark corresponding to the i-th observation
	landmarkId = z(i).id;
	% If the landmark is obeserved for the first time:
	if(observedLandmarks(landmarkId)==false)
		% Initialize its pose in mu based on the measurement and the current robot pose:
		mu(2*landmarkId+2) = mu(1) + z(i).range * cos(z(i).bearing + mu(3));
    mu(2*landmarkId+3) = mu(2) + z(i).range * sin(z(i).bearing + mu(3));
		% Indicate in the observedLandmarks vector that this landmark has been observed
		observedLandmarks(landmarkId) = true;
	endif

	% Add the landmark measurement to the Z vector
	Z(i*2-1) = z(i).range;
  Z(i*2)   = z(i).bearing;
  
	% Use the current estimate of the landmark pose
	% to compute the corresponding expected measurement in expectedZ:

  delta = [mu(2*landmarkId+2) - mu(1); mu(2*landmarkId+3) - mu(2)];
  sqrtq = sqrt(delta' * delta);
  
  expectedZ(i*2-1) = sqrtq;
  expectedZ(i*2)   = normalize_angle(atan2(delta(2),delta(1)) - mu(3));
  
	% Compute the Jacobian Hi of the measurement function h for this observation
  % TODO: map to the high dimensional space without using F:
  Hi = 1/(delta' * delta) * [ -sqrtq * delta(1), -sqrtq * delta(2), 0, sqrtq * delta(1), sqrtq * delta(2) ;
                              delta(2), -delta(1),  -(delta' * delta), -delta(2), delta(1)];
  
  F = zeros(5,mu_size);
  F(1:3,1:3) = eye(3);
  F(4,2*landmarkId+2) = 1;
  F(5,2*landmarkId+3) = 1;
	% Augment H with the new Hi
	H = [H;Hi*F];	
endfor

% Construct the sensor noise matrix Q

Q = eye(2*m) * 0.01;

% Compute the Kalman gain

K = sigma * H' /(H * sigma * H' + Q);

% Compute the difference between the expected and recorded measurements.
% Remember to normalize the bearings after subtracting!
% (hint: use the normalize_all_bearings function available in tools)

dZ = normalize_all_bearings(Z - expectedZ);

% Finish the correction step by computing the new mu and sigma.
% Normalize theta in the robot pose.

mu = mu + K * dZ;
sigma = (eye(mu_size) - K * H) * sigma;

end
