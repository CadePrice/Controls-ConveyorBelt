loggedNames = who(out);
if isempty(loggedNames)
    error("No scope data was logged. Enable 'Log data to workspace' in the Scope block.");
end

data = out.(loggedNames{1});   % or use the exact variable name from the Scope block

if isstruct(data) && isfield(data, "time") && isfield(data, "signals")
    t = data.time;
    y = data.signals(1).values;
elseif isa(data, "Simulink.SimulationData.Dataset")
    sig = data{1};
    t = sig.Values.Time;
    y = sig.Values.Data;
elseif isa(data, "timeseries")
    t = data.Time;
    y = data.Data;
else
    error("Unsupported logged data type: %s", class(data));
end

figure;
plot(t, y, "LineWidth", 1.5);
grid on;
xlabel("Time (s)");
ylabel("Speed (%)");
title("Setpoint vs. Response");