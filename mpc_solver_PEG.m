function sfun_mpc_casadi(block)
% level-2 matlab s-function for mpc
persistent opti_opts opti Ts N oscillator_model P X0 U X

setup(block);

    function setup(block)
        block.NumInputPorts = 2; % 输入：y, y_ref
        block.NumOutputPorts = 1; % 输出：u

%         block.SetPreCompInpPortInfoToDynamic;
        % 输入端口1:y
        block.InputPort(1).Dimensions = 8; % 状态 x: [ψ; v; Xe; Xn]
        block.InputPort(1).DirectFeedthrough = true;
        % 输入端口2:y_ref
        block.InputPort(2).Dimensions = 2; % 参考位置: [Xe_ref; Xn_ref]
        block.InputPort(2).DirectFeedthrough = true;
        % 输出端口:u
%         block.SetPreCompInpPortInfoToDynamic;
        block.OutputPort(1).Dimensions = 2; % 控制输出: [a_c; phi_c]

        block.RegBlockMethod('Start', @Start);
        block.RegBlockMethod('Outputs', @Outputs);
    end

    function Start(~)
        % CasADi 初始化
        casadi_path = 'D:\casadi\casadi-3.7.0-windows64-matlab2018b';
        addpath(casadi_path);
        import casadi.*
        % 参数定义
        Ts = 0.02;N = 20;g = 9.81;

        % 预测模型
        x = SX.sym('x', 8); % [psi, v, x_east, x_north, ATA, xe_t, xn_t]
        u = SX.sym('u', 2); % [a_c, phi_c]

        psi = x(1); v = x(2); xe = x(3); xn = x(4); ata = x(5); xe_t = x(6); xn_t = x(7); psi_t = x(8);
        
        a_c = u(1); phi_c = u(2);

        v_ = v + Ts * a_c;
        psi_raw = psi + Ts * g * tan(phi_c) / max(v, 1);
        psi_ = atan2(sin(psi_raw), cos(psi_raw));

        xe_ = xe + Ts * v * sin(psi);
        xn_ = xn + Ts * v * cos(psi);
        
        psi_t_ = psi_t;
        xe_t_ = xe_t + Ts * 60 * sin(psi_t_);
        xn_t_ = xn_t + Ts * 60 * cos(psi_t_);
        
        dx = fmax(xe_t - xe_, 1e-2); dy = fmax(xn_t - xn_, 1e-2);
        cos_ata_raw = (sin(psi_)*dx + cos(psi_)*dy) / sqrt(dx^2 + dy^2);
        cos_ata = fmin(fmax(cos_ata_raw, -1 + 1e-6), 1 - 1e-6);
        ata_ = acos(cos_ata);

        x_ = [psi_; v_; xe_; xn_; ata_; xe_t_; xn_t_; psi_t_];

        oscillator_model = Function('oscillator_model', {x, u}, {x_});
        % CasADi 优化器
        opti_opts = casadi.Opti(); % 全局实例

        % 优化变量
        X = opti_opts.variable(8, N + 1); % 状态变量
        U = opti_opts.variable(2, N); % 控制变量
        P = opti_opts.parameter(1, 2); % 参考轨迹
        X0 = opti_opts.parameter(8, 1); % 初始状态
        

        % 代价函数
        J = 0;
        Q_pos = 50; Q_ata = 100; R1 = 0.01; R2 = 0.01;
        for k=1:N
            x_ = oscillator_model(X(:, k), U(:, k));
            opti_opts.subject_to(X(:, k+1) == x_);

            pos_error = (X(3:4, k+1) - P.').^2;
            ata_error = X(5, k+1)^2;

            J = J + Q_pos * sum(pos_error) + ...
                Q_ata * ata_error + ...
            R1 * U(1,k)^2 + R2 * U(2,k)^2;
        end

        % 添加约束和目标函数
        opti_opts.subject_to(X(:, 1) == X0);
        opti_opts.subject_to(60 <= X(2, :) <= 130);
        opti_opts.subject_to(-4 <= U(1, :) <= 4);
        opti_opts.subject_to(-30/57.3 <= U(2, :) <= 30/57.3);
        opti_opts.minimize(J);
        % 配置求解器
        opts = struct;
        opts.ipopt.print_level = 0;
        opts.ipopt.max_iter = 10000;
        opts.ipopt.print_level = 5;
        opts.ipopt.acceptable_tol = 1e-1;
        opts.ipopt.acceptable_iter = 200;
        opts.ipopt.tol = 1e-2;
        opts.print_time = 0;
        opti_opts.solver('ipopt', opts);
    end

    function Outputs(block)
        % 获取输入信号
        y = block.InputPort(1).Data;
        y_ref = block.InputPort(2).Data;

        % 更新参数
        opti_opts.set_value(P, y_ref);
        opti_opts.set_value(X0, y);

        % 求解优化问题
        try
        sol = opti_opts.solve();
        catch e
%         disp('Debugging values:');
%         disp('State variables X:');
%         disp(opti_opts.debug.value(X));
%         disp('Control inputs U:');
%         disp(opti_opts.debug.value(U));
        rethrow(e);
        end

        % 输出最优控制输入
        block.OutputPort(1).Data = sol.value(U(:, 1));
    end
end