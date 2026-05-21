%% Main script

% 1D global localization scenario: the robot moves on a discrete grid.
N = 100; % number of cells [1][2] ... [N]

% --- GROUND TRUTH (Hidden from the filter) ---
true_initial_pos = 10; % x_0: True initial state: the robot is in cell 10
L_pos = 50;            % Landmark position

% --- INITIAL BELIEF - Filters internal state %
% Bel(x_0): Uniform prior, expresses maximum uncertainty about where the robot is
bel = ones(1, N) / N; 

% --- SIMULATION PARAMETERS --- %
steps = 20; 
u = 1; % Control input u_t: at each step the robot moves to the right by +1 unit

% 1) Average Robot
%sigma_motion = 2; % Motion noise 
%sigma_sensor = 1.4;   % Observation noise 

% 2) Kidnapped Robot
%sigma_motion = 8.0;
%sigma_sensor = 0.5;

% 3) Blind Robot
%sigma_motion = 0.1;
%sigma_sensor = 10.0;

% which steps to include in the final plot  
t_plot = [1,5,10,15,20];

figure;
true_pos = true_initial_pos;    
subplot_idx = 1; 
for t = 1:steps
    % ==========================================
    % SIMULATION OF PHYSICAL REALITY
    % ==========================================
    
    % 1. Real kinematics: x_t = x_{t-1} + u_t + N(0, sigma_motion^2)
    % Additive Gaussian noise simulates physical actuation errors.
    % rounding forces snapping to the grid
    actual_motion = u + round(randn() * sigma_motion);
    true_pos = true_pos + actual_motion;
    % Limit true_pos to the grid boundaries
    true_pos = max(1, min(N, true_pos));
    
    % 2. Real observation: z_t = |x_t - L| + N(0, sigma_sensor^2)
    % Additive Gaussian noise simulates sensor inaccuracy.
    % rounding to always get an integer reading snapped
    % to the grid
    z_dist = abs(true_pos - L_pos) + round(randn() * sigma_sensor);
    
    % ==========================================
    % BAYES FILTER
    % ==========================================
    
    % 1. PREDICTION
    % Computes the a priori belief \overline{bel}(x_t) for the current step.
    % Integrates the control command, assuming the filter uses the correct motion noise model.
    bel_bar = motion_update(bel, u, sigma_motion, N);
   
    % 2. CORRECTION
    % Computes the a posteriori belief bel(x_t).
    % Merges the prior of the predicted state with the likelihood of the new sensory reading
    bel = sensor_update_dist(bel_bar, z_dist, sigma_sensor, L_pos, N);
    
    % --- VISUALIZATION ---
    if (ismember(t,t_plot))
        subplot(numel(t_plot), 1, subplot_idx);
        plot(bel_bar, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Prediction'); hold on;
        plot(bel, 'b', 'LineWidth', 2, 'DisplayName', 'Posterior');
        xline(L_pos, 'k:', '--o', 'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
        xline(true_pos, 'Color', [0, 0.5, 0], 'LineWidth', 1.5, 'DisplayName', 'True state x_t');
        title(['Step ', num2str(t), ': Measured distance z_t = ', num2str(z_dist, '%.2f')]);
        grid on;
        if subplot_idx == 1, legend('Location', 'best'); end
        subplot_idx = subplot_idx + 1;
    end
        
end

%% motion_update: Computes the predicted belief (bel_bar), given only the control action u.
function bel_bar = motion_update(bel, u, sigma, N)
    
    bel_bar = zeros(1, N);
    
    % i represents the DESTINATION state (x_t) we are evaluating
    for i = 1:N 
        
        % j represents the SOURCE state (x_{t-1}) from which the robot could come
        for j = 1:N 
            
            % 1. KINEMATIC ERROR
            % If we started from 'j' and applied command 'u', 
            % our predicted destination is 'j + u'.
            % 'dist' is the physical deviation between the predicted destination and destination 'i'.
            dist = i - (j + u);
            
            % 2. STATE TRANSITION PROBABILITY (Motion model)
            % Computes the unnormalized p(x_t=i | x_{t-1}=j, u_t=u) using a Gaussian kernel.
            % - If dist == 0 (perfect motion), prob_move is 1.
            % - If dist > 0, the probability decays exponentially.
            % - A larger 'sigma' flattens the curve, increasing 
            % the probability of landing further away.
            prob_move = exp(-0.5 * (dist / sigma)^2);
            
            % 3. MARGINALIZATION (Law of total probability)
            % Multiplies the transition likelihood (prob_move) by the a priori probability 
            % of starting from that state (bel(j)). Accumulates this for all possible source states 'j'.
            bel_bar(i) = bel_bar(i) + prob_move * bel(j);
        end
    end
    
    % 4. NORMALIZATION
    % Ensures the resulting array is a valid probability mass function that sums exactly to 1.
    bel_bar = bel_bar / sum(bel_bar);
end

%% sensor_update_dist: Corrects the predicted belief (bel_bar) using the new measurement (z_dist)
function bel = sensor_update_dist(bel_bar, z_dist, sigma, L_pos, N)
    % SENSOR_UPDATE_DIST performs the correction phase of the discrete Bayes filter.
    % Computes the a posteriori belief (bel) by merging the predicted state (bel_bar) 
    % with the new sensor measurement (z_dist).
    
    % 1. VECTORIZATION OF THE STATE SPACE
    % Creates an array 'x' representing all possible robot positions [1, 2, ..., N].
    % This allows us to compute the likelihood for the entire grid simultaneously 
    % without using a for loop.
    x = 1:N;
    
    % 2. FORWARD OBSERVATION MODEL
    % Computes the theoretical and perfect distance from each possible cell 'x' 
    % to the known landmark position 'L_pos'.
    expected_dist = abs(x - L_pos);
    
    % 3. LIKELIHOOD FUNCTION: P(z_t | x_t)
    % Evaluates how much the actual sensor reading 'z_dist' corresponds to the 'expected_dist' 
    % for each cell, assuming Gaussian sensor noise.
    % - The error is the difference between the expected and measured distance.
    % - Cells where (expected_dist == z_dist) return the maximum likelihood (1.0).
    % - The '.^' operator applies the exponent element-wise to the entire vector.
    likelihood = exp(-0.5 * ((expected_dist - z_dist) / sigma).^2);
    
    % 4. CORRECTION
    % Applies Bayes rule: Posterior is proportional to Likelihood * Prior.
    % We multiply the new evidence (likelihood) by our existing prediction (bel_bar).
    bel = likelihood .* bel_bar;
    
    % 5. NORMALIZATION (eta)
    % Divides by the total sum to ensure the resulting array is a valid 
    % probability distribution that sums to 1. This corresponds to the 'eta' factor 
    % in the formal equations of the Bayes filter.
    bel = bel / sum(bel); 
end