function [mu, sigma] = recover_gaussian(sigma_points, w_m, w_c)
% This function computes the recovered Gaussian distribution (mu and sigma)
% given the sigma points (size: nx2n+1) and their weights w_m and w_c:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
% The weight vectors are each 1x2n+1 in size,
% where n is the dimensionality of the distribution.

% Try to vectorize your operations as much as possible
n = size(sigma_points,1);

% compute mu
mu = (w_m * sigma_points')'; % mu = nx1;

% compute sigma
chi_minus_mu = sigma_points .- mu; % chi_minus_mu = nx2n+1
sigma = zeros(n,n);

% TODO: This has to be vectorizable...
for i = 1:2*n+1
  sigma = sigma + w_c(i) * (chi_minus_mu(1:n,i) * chi_minus_mu(1:n,i)');
endfor

end
