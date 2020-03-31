% Basic RANSAC fitting
% Applies least squares fitting to the samples according to the RANSAC
% fitting algorithm.
%
% Parameters:
%   D_x: [nx1] column vector of the differences (deltas) between 
%   the predicted position and position from vision measurements 
%   D_t: [nx1] column vector of time deltas from the start of the window to 
%   each measurement
%   iter: number of iterations in the RANSAC algorithm
%   sigma_th: error threshold
%   n: number of elements to select from the array
%   P: the 
%
% Outputs:
%   x_off, v_off: linear model parameters
%
% Author: Samuel Carbone, scar7732@uni.sydney.edu.au
% Semester 1, 2020
%
% Notes:
% This is not quite the full ransac algorithm.

function [x_off, v_off] = prior_ransac(D_x, D_t, iter, sigma_th, n, P)
    % The minimum summed error for the iterations
    epsilon_min = [];
    index_min = [];
    
    % The final coefficients of the linear model
    x_off = 0;
    v_off = 0;
    
    for i = 1:(iter+1)
        % Select n random elements from the input data
        rand_index = randperm(length(D_t), n);
        D_t_sample = D_t(rand_index);
        D_x_sample = D_x(rand_index);
        
        % Apply LS to the samples
        X = [ones(length(D_t_sample), 1), D_t_sample];
        Y = D_x_sample;
        coeffs = linear_LS(X, Y, P);
        x_off_i = coeffs(1);
        v_off_i = coeffs(2);
        
        % Sum of squared errors for all of the samples, for this iteration
        epsilon_i = 0;
        
        for j = 1:length(D_t)
            % Calculate the 2-norm of the error between the estimate and
            % actual Dx this sample
            epsilon_j = (v_off_i* D_t(j) + x_off_i - D_x(j))^2;
            
            % Add to the total error for this iteration
            if epsilon_j > sigma_th^2
                epsilon_i = epsilon_i + sigma_th;
            else
                epsilon_i = epsilon_i + epsilon_j;
            end
            
        end
        
        % Check if this has the lowest of the sum of squared errors
        % compared to the other iterations
        if isempty(epsilon_min)
            epsilon_min = epsilon_i;
            index_min = i;
            x_off = x_off_i;
            v_off = v_off_i;
        
        elseif epsilon_i < epsilon_min
            epsilon_min = epsilon_i;
            index_min = i;
            x_off = x_off_i;
            v_off = v_off_i;
            
        end
        
    end 
    
end

% Calculate the linear least squares given, X, Y and penalty P
% Returns the coefficients of the linear model
function coeffs = linear_LS(X, Y, P)

    [~, x_cols] = size(X);

    if nargin < 3
        P = zeros(x_cols);
    end
    
    coeffs = (X' * X + P) \ (X' * Y); 

end