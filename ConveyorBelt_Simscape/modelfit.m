% Load the saved Simulink.SimulationOutput from myData.mat
data = load('myData.mat');
if ~isfield(data, 'out')
    error('myData.mat must contain a variable named out. Found: %s', ...
        strjoin(fieldnames(data), ', '));
end
out = data.out;

% Logged signal: speed_mms is a timeseries with 2 columns
% (column 1 = setpoint/input, column 2 = measured speed/output)
simout = out.get('speed_mms');
if ~isa(simout, 'timeseries') || size(simout.Data, 2) ~= 2
    error('speed_mms must be a 2-column timeseries [setpoint, response].');
end

t = simout.Time(:);
u = simout.Data(:, 1);
y = simout.Data(:, 2);
Ts = median(diff(t));

% Uniformly sampled input/output dataset for tfest/compare
tt = iddata(y, u, Ts);
tt.TimeUnit = 'seconds';

%% ---- Dataset diagnostics ----
fprintf('\n================ DATASET ================\n');
fprintf('Samples            : %d\n', numel(t));
fprintf('Time span          : %.4f to %.4f s\n', t(1), t(end));
fprintf('Sample time Ts     : %.6g s (min dt %.6g, max dt %.6g)\n', Ts, min(diff(t)), max(diff(t)));
fprintf('Input  u  : min %.4g  max %.4g  mean %.4g  std %.4g  first %.4g  last %.4g\n', ...
    min(u), max(u), mean(u), std(u), u(1), u(end));
fprintf('Output y  : min %.4g  max %.4g  mean %.4g  std %.4g  first %.4g  last %.4g\n', ...
    min(y), max(y), mean(y), std(y), y(1), y(end));
fprintf('Steady-state gain estimate (y_end/u_end): %.4g\n', y(end)/u(end));
[~, iPk] = max(y);
fprintf('Peak output %.4g at t = %.4f s (overshoot vs final: %.2f %%)\n', ...
    y(iPk), t(iPk), 100*(y(iPk)-y(end))/max(abs(y(end)), eps));
yf = y(end);
i10 = find(y >= 0.1*yf, 1);
i90 = find(y >= 0.9*yf, 1);
if ~isempty(i10) && ~isempty(i90)
    fprintf('10-90%% rise time   : %.4f s\n', t(i90) - t(i10));
end
iSet = find(abs(y - yf) > 0.02*abs(yf), 1, 'last');
if ~isempty(iSet)
    fprintf('2%% settling time   : %.4f s\n', t(min(iSet+1, numel(t))));
end

%% ---- Estimate candidate models ----
orders = [2 3];
names  = {'sys2', 'sys3'};
models = cell(1, numel(orders));
for k = 1:numel(orders)
    fprintf('\n================ tfest order %d ================\n', orders(k));
    tic;
    models{k} = tfest(tt, orders(k));
    fprintf('Estimation time: %.2f s\n', toc);
end
sys2 = models{1};
sys3 = models{2};

%% ---- Per-model report ----
results = struct();
for k = 1:numel(models)
    G = models{k};
    nm = names{k};
    fprintf('\n################ MODEL %s (order %d) ################\n', nm, orders(k));

    % Transfer function
    fprintf('\n-- Transfer function --\n');
    disp(G);
    [num, den] = tfdata(G, 'v');
    fprintf('Numerator coefficients   (descending s): %s\n', mat2str(num, 6));
    fprintf('Denominator coefficients (descending s): %s\n', mat2str(den, 6));
    fprintf('Input delay: %g s   Output delay: %g s   IO delay: %s\n', ...
        G.InputDelay, G.OutputDelay, mat2str(G.IODelay));

    % Parameter uncertainty
    fprintf('\n-- Parameter values, std devs, and 95%% confidence --\n');
    try
        [p, dp] = getpvec(G);
        for j = 1:numel(p)
            fprintf('  p%-2d = %14.6g   +/- %-12.4g  (95%% CI [%.6g, %.6g])\n', ...
                j, p(j), dp(j), p(j)-1.96*dp(j), p(j)+1.96*dp(j));
        end
    catch ME
        fprintf('  (could not read parameter uncertainty: %s)\n', ME.message);
    end
    try
        fprintf('\n-- Parameter covariance matrix --\n');
        disp(getcov(G));
    catch ME
        fprintf('  (covariance unavailable: %s)\n', ME.message);
    end

    % Poles / zeros / dc gain
    fprintf('\n-- Poles --\n');
    disp(pole(G));
    fprintf('-- Zeros --\n');
    disp(zero(G));
    fprintf('-- DC gain: %.6g --\n', dcgain(G));
    fprintf('-- Stable: %d --\n', isstable(G));
    fprintf('-- Damping / natural frequency of poles --\n');
    damp(G);
    fprintf('\n-- Step info --\n');
    disp(stepinfo(G));
    fprintf('-- Bandwidth: %.6g rad/s --\n', bandwidth(G));

    % Fit metrics from System Identification Toolbox
    fprintf('\n-- Identification fit metrics --\n');
    rep = G.Report;
    disp(rep.Fit);
    fprintf('\n-- Full estimation report --\n');
    disp(rep);
    try, disp(rep.Termination); catch, end
    try, disp(rep.Method);      catch, end

    % Independent simulation metrics
    % The data starts away from equilibrium (u = 40 but y = 0), so the model
    % must be simulated with its estimated initial state, exactly as compare()
    % does. A zero-state lsim leaves a constant offset in the error.
    [yh, fitCmp, x0] = compare(tt, G);
    yhat = yh.OutputData;
    fprintf('\n-- Estimated initial state x0 used for simulation --\n');
    disp(x0(:).');
    yhat0 = lsim(G, u, t);   % zero-initial-state version, for reference only
    fprintf('Zero-initial-state RMSE (reference, includes offset): %.6g\n', ...
        sqrt(mean((y - yhat0).^2)));
    e = y - yhat;
    rmse     = sqrt(mean(e.^2));
    mae      = mean(abs(e));
    maxErr   = max(abs(e));
    fitNRMSE = 100 * (1 - norm(e) / norm(y - mean(y)));
    R2       = 1 - sum(e.^2) / sum((y - mean(y)).^2);
    cc       = corrcoef(y, yhat);
    fprintf('\n-- Simulation metrics (estimated initial state) --\n');
    fprintf('RMSE                 : %.6g\n', rmse);
    fprintf('MAE                  : %.6g\n', mae);
    fprintf('Max abs error        : %.6g\n', maxErr);
    fprintf('NRMSE fit %%          : %.4f\n', fitNRMSE);
    fprintf('R^2                  : %.6f\n', R2);
    fprintf('Correlation y/yhat   : %.6f\n', cc(1,2));
    fprintf('Final-value error    : %.6g (y_end %.6g, yhat_end %.6g)\n', e(end), y(end), yhat(end));
    fprintf('Residual mean / std  : %.6g / %.6g\n', mean(e), std(e));

    fprintf('compare() fit %%      : %.4f\n', fitCmp);

    % Pole/zero separation check (is the model effectively lower order?)
    pl = pole(G); zr = zero(G);
    [~, ord] = sort(abs(real(pl)), 'descend');
    fprintf('\n-- Pole/zero separation check --\n');
    fprintf('Poles sorted fastest to slowest: %s\n', mat2str(real(pl(ord)).', 5));
    if numel(pl) > 1
        ratios = abs(real(pl(ord(1:end-1)))) ./ abs(real(pl(ord(2:end))));
        fprintf('Speed ratios between consecutive poles: %s\n', mat2str(ratios.', 4));
    end
    for iz = 1:numel(zr)
        [dmin, ip] = min(abs(pl - zr(iz)));
        fprintf('Zero %.5g is closest to pole %.5g (distance %.3g, relative %.3g)\n', ...
            real(zr(iz)), real(pl(ip)), dmin, dmin / max(abs(pl(ip)), eps));
    end

    results.(nm) = struct('G', G, 'num', num, 'den', den, 'yhat', yhat, ...
        'rmse', rmse, 'mae', mae, 'maxErr', maxErr, 'fitNRMSE', fitNRMSE, ...
        'R2', R2, 'fitCompare', fitCmp);
end

%% ---- Side-by-side summary table ----
fprintf('\n================ SUMMARY ================\n');
summ = table(string(names(:)), orders(:), ...
    [results.sys2.rmse; results.sys3.rmse], ...
    [results.sys2.mae; results.sys3.mae], ...
    [results.sys2.maxErr; results.sys3.maxErr], ...
    [results.sys2.fitNRMSE; results.sys3.fitNRMSE], ...
    [results.sys2.R2; results.sys3.R2], ...
    [results.sys2.fitCompare; results.sys3.fitCompare], ...
    [dcgain(sys2); dcgain(sys3)], ...
    'VariableNames', {'Model','Order','RMSE','MAE','MaxErr','NRMSE_pct','R2','Compare_pct','DCgain'});
disp(summ);
[~, best] = max([results.sys2.fitCompare, results.sys3.fitCompare]);
fprintf('Best fit by compare(): %s\n', names{best});

%% ---- Chosen model ----
Ghat = models{best};
fprintf('\nGhat set to %s (order %d)\n', names{best}, orders(best));

%% ---- Plots (original two only) ----
compare(tt, sys2, sys3);

figure;
plot(t, y, 'LineWidth', 1.2);
hold on;
plot(t, results.(names{best}).yhat, '--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Output (units)');
legend('Simscape plant', 'Estimated transfer function', 'Location', 'best');

yhat = results.(names{best}).yhat;
rmse = results.(names{best}).rmse;
fitNRMSE = results.(names{best}).fitNRMSE;
