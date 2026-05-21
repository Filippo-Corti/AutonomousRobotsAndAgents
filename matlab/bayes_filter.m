clc
clear
close all

% Create figure with 5 side-by-side plots
tiledlayout(1,4)

simulate_bayes_filter(100, 'D', 0.5, 0.0, 0.0);
simulate_bayes_filter(100, 'D', 0.5, 0.99, 0.5);
simulate_bayes_filter(100, 'H', 0.999, 0.6, 0.6);
simulate_bayes_filter(100, 'D', 0.5, 0.5, 0.5);


function simulate_bayes_filter(n, x_true, prior, p_zD_D, p_zH_H)

    % -- Binary Sensor Characterization (factory data) --
    p_zH_D = 1 - p_zD_D;        % false negative rate (FNR)
    p_zD_H = 1 - p_zH_H;        % false positive rate (FPR)

    % 1) Generate n measurements, given the true state (x_true)
    % and the above sensor parameters
    % We pre-allocate the measurements in the z vector
    z = zeros(1, n);

    for idx = 1:n
        r = rand(); % uniform in 0,1

        if x_true == 'D'
            if r <= p_zD_D
                z(idx) = 1;
            else
                z(idx) = 0;
            end
        end

        if x_true == 'H'
            if r <= p_zH_H
                z(idx) = 0;
            else
                z(idx) = 1;
            end
        end
    end

    fprintf("Generated %d zeros\n", sum(z == 0));
    fprintf("Generated %d ones\n", sum(z == 1));

    % 2) Run simulation (take measurements,
    % update belief of plant's state)

    % Preallocate beliefs for n+1 steps
    % (as we want to see belief progressing)
    belief_D = zeros(1, n+1);
    belief_H = zeros(1, n+1);

    % Set initial value
    belief_D(1) = prior;
    belief_H(1) = 1 - prior;

    for t = 2:n+1

        % A) Get new measurement
        zt = z(t-1);

        % B) Extract likelihood from sensor info
        if zt == 1
            LD = p_zD_D; % p(z=1 | D)
            LH = p_zD_H; % p(z=1 | H)
        else
            LD = p_zH_D; % p(z=0 | D)
            LH = p_zH_H; % p(z=0 | H)
        end

        % C) Compute new belief using
        % likelihood and prior belief
        unnormalized_D = LD * belief_D(t-1);
        unnormalized_H = LH * belief_H(t-1);

        normalizer = unnormalized_D + unnormalized_H;

        belief_D(t) = unnormalized_D / normalizer;
        belief_H(t) = unnormalized_H / normalizer;
    end

    % 3) Plot how the belief about the state being 'D'
    % changes as more measurements are taken into account
    nexttile

    plot(1:n+1, belief_D, 'b-*', 'LineWidth', 1.5);

    ylim([0 1.2]);
    yticks(0:0.1:1);

    xlabel('Time step');
    ylabel('Belief probability');

    title(sprintf( ...
        'True=%s | Prior=%.2f\nTPR=%.2f TNR=%.2f', ...
        x_true, prior, p_zD_D, p_zH_H));

    grid on;
end