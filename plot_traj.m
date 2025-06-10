close all;
state = out.simout.y;
ctrl = out.simout.u;
t = out.simout.y.psi.Time;

fields_state = fieldnames(state);
fields_ctrl  = fieldnames(ctrl);

for i = 1:length(fields_state)
    var_name = fields_state{i};
    ts = state.(var_name);
    assignin('base', var_name, ts.Data);
end


for i = 1:length(fields_ctrl)
    var_name = fields_ctrl{i};
    ts = ctrl.(var_name);
    assignin('base', var_name, ts.Data);
end

figure;
plot(x, y);
hold on;grid on;
plot(x_t, y_t);

figure;
plot(t, v);
grid on;

figure;
plot(t, phi_c);

figure;
plot(t, a_c);
