%% robot_can_MPC_ESO_Seamless.m
% Điều khiển robot 2 bánh vi sai mượt mà LIỀN MẠCH 100% (Đã tối ưu tham số và nội suy)
clear; clc; close all;
%% ===================== 1. CẤU HÌNH CHẠY =====================
USE_HARDWARE = true;             % true: chạy xe thật; false: mô phỏng
COM_PORT = "COM7";
BAUD = 115200;
Ts = 0.02;                       % Chu kỳ gửi lệnh [s] (20ms)
T_ENTRY = 12.0;                  % Thời gian đi hết đoạn Bezier dẫn nhập [s]
NUM_LAPS = 1.0;                  % Dừng sau chính xác 1 vòng (360 độ)
MAX_RUN_TIME = 150.0;            
%% ===================== 2. THÔNG SỐ XE / DRIVER =====================
r = 0.095;                       
b = 0.528;                       
GEAR_RATIO = 20;                 
RPM_IS_MOTOR_SHAFT = true;
LEFT_RPM_SIGN  = 1;
RIGHT_RPM_SIGN = 1;
RPM_PER_CMD_L = 3.0;
RPM_PER_CMD_R = 3.0;
CMD_LIMIT = 2500;                
CMD_MIN_MOVE = 4;                
D_CMD_LIM_RUN = 10.0;            
%% ===================== 3. QUỸ ĐẠO LIÊN TỤC =====================
R = 1.50;                        
D = 2*R;                         
ORBIT_CX = 0.0; ORBIT_CY = 0.0;
DIR = 1;                         
wr = 0.10;                       
PHI_JOIN = deg2rad(60);
LOOKAHEAD_RAD = 0.0;             % Đã đặt về 0.0 để triệt tiêu lệch pha
INITIAL_HEADING_DEG = 22.5;
INITIAL_HEADING = deg2rad(INITIAL_HEADING_DEG);

P0 = [0, 0];
P3 = [ORBIT_CX + R*cos(PHI_JOIN), ORBIT_CY + R*sin(PHI_JOIN)];
theta0 = INITIAL_HEADING;
theta3 = PHI_JOIN + DIR*pi/2;    
dist_p0_p3 = norm(P3 - P0);
handle_len = dist_p0_p3 * 0.414;  
P1 = P0 + handle_len * [cos(theta0), sin(theta0)];
P2 = P3 - handle_len * [cos(theta3), sin(theta3)];
%% ===================== 4. CẤU HÌNH MPC VÀ ESO =====================
V_SCALE = 1.0;                   % Đã chuẩn hóa về 1.0
V_MAX   = 0.35;                  
V_MIN   = -0.15;                 
OM_MAX  = 1.20;                  
OM_MIN  = -1.20;

% Đồng bộ thông số Horizon
Np = 50; Nc = 25;                         
Q_diag = [250.0, 250.0, 10.0];   % Tăng cường độ bám
R_diag = [20.05, 1.01];           

% Cấu hình ESO
wo_x  = 2.0;                    
wo_y  = 2.0;                    
wo_th = 0.1;                    
Beta_x  = [2*wo_x;  wo_x^2];
Beta_y  = [2*wo_y;  wo_y^2];
Beta_th = [2*wo_th; wo_th^2];
MAX_FX  = 0.15;  
MAX_FY  = 0.15;  
MAX_FTH = 0.30;  
SEND_ERROR_TO_DUE = true; ERR_SEND_EVERY_N = 5;
%% ===================== 5. KẾT NỐI ARDUINO DUE =====================
due = [];
if USE_HARDWARE
    ports = serialportlist("available");
    if ~any(ports == COM_PORT)
        error("Không thấy %s. Kiểm tra kết nối Due.", COM_PORT);
    end
    due = serialport(COM_PORT, BAUD);
    configureTerminator(due, "LF"); flush(due); pause(2.0);
    cleanupObj = onCleanup(@() stopRobot(due)); 
    writeline(due, "EN"); pause(0.2); writeline(due, "PING");
else
    cleanupObj = onCleanup(@() []);
end
%% ===================== 6. VẼ QUỸ ĐẠO =====================
T_CIRCLE_TARGET = NUM_LAPS * 2 * pi / wr; 
Nref = 1400;
tRefPlot = linspace(0, T_ENTRY + T_CIRCLE_TARGET, Nref);
refPlot = zeros(Nref, 6);
for k = 1:Nref
    refPlot(k,:) = referenceMPC_Seamless(tRefPlot(k), R, wr, T_ENTRY, P0, P1, P2, P3, PHI_JOIN, DIR, LOOKAHEAD_RAD, ORBIT_CX, ORBIT_CY);
end
figure(1); clf; hold on; grid on; axis equal;
plot(refPlot(:,1), refPlot(:,2), 'k-', 'LineWidth', 2.5, 'DisplayName', 'Quỹ đạo tham chiếu');
plot(P0(1), P0(2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [1 0.7 0], 'DisplayName', 'Start P0');
plot(P3(1), P3(2), 's', 'MarkerSize', 8, 'MarkerFaceColor', [0.6 0.1 0.8], 'MarkerEdgeColor', 'k', 'DisplayName', 'Điểm chuyển tiếp P3');
hRobotPath = plot(0, 0, 'r--', 'LineWidth', 2.2, 'DisplayName', 'Thực tế xe đi');
hRobotNow  = plot(0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5, 'HandleVisibility', 'off');
hTargetNow = plot(0, 0, 'm*', 'MarkerSize', 10, 'LineWidth', 1.5, 'HandleVisibility', 'off');
xlabel('x (m)'); ylabel('y (m)'); title('Bám Quỹ Đạo Bằng MPC-ESO - Cực Đạt');
legend('Location', 'northeastoutside');
limPlot = R + 0.35; xlim([-limPlot limPlot]); ylim([-limPlot limPlot]);
%% ===================== 7. KHỞI TẠO VÀ VÒNG LẶP CHÍNH =====================
x = P0(1); y = P0(2); theta = INITIAL_HEADING;
cmdL_old = 0; cmdR_old = 0;
last_rpmL = 0; last_rpmR = 0;
haveFeedback = false; lastFeedbackTime = -Inf;
ex_f = 0; ey_f = 0; eth_f = 0; rerr_f = 0;
robotX = x; robotY = y; v_last = 0; om_last = 0; 
z_hat = [x; y; theta; 0; 0; 0];
tStart = tic; tPrev = 0; idx = 1;
maxN = floor(MAX_RUN_TIME/Ts) + 2000;
LOG = nan(maxN, 30);
qp_options = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4); % Tăng độ tinh cậy solver
isFinished = false;
try
    while idx <= maxN && ~isFinished
        loopStart = tic;
        t = toc(tStart); dt = t - tPrev;
        if dt <= 0 || dt > 0.20, dt = Ts; end
        tPrev = t;
        
        %% --- READ FEEDBACK ---
        if USE_HARDWARE
            [newFeedback, rpmL, rpmR] = readFeedbackRPM(due);
            if newFeedback
                last_rpmL = rpmL; last_rpmR = rpmR;
                haveFeedback = true; lastFeedbackTime = t;
            end
        end
        fbFresh = haveFeedback && ((t - lastFeedbackTime) < 0.60);
        
        %% --- ODOMETRY ---
        if USE_HARDWARE && fbFresh
            rpmL_motor = LEFT_RPM_SIGN * double(last_rpmL);
            rpmR_motor = RIGHT_RPM_SIGN * double(last_rpmR);
        else
            rpmL_motor = RPM_PER_CMD_L * double(cmdL_old);
            rpmR_motor = RPM_PER_CMD_R * double(cmdR_old);
        end
        
        rpmL_wheel = rpmL_motor / GEAR_RATIO;
        rpmR_wheel = rpmR_motor / GEAR_RATIO;
        wL_fb = rpmL_wheel * 2*pi/60; wR_fb = rpmR_wheel * 2*pi/60;
        v_fb = r*(wR_fb + wL_fb)/2; om_fb = r*(wR_fb - wL_fb)/b;
        
        x = x + v_fb*cos(theta)*dt;
        y = y + v_fb*sin(theta)*dt;
        theta = wrapToPiLocal(theta + om_fb*dt);
        
        %% --- CẬP NHẬT ESO ---
        Y_meas = [x; y; theta];
        z_hat = updateESO_3Axis(z_hat, Y_meas, v_last, om_last, dt, Beta_x, Beta_y, Beta_th, MAX_FX, MAX_FY, MAX_FTH);
        eso = z_hat(4:6);
        
        %% --- BÁM QUỸ ĐẠO BẰNG MPC-ESO ---
        if t >= (T_ENTRY + T_CIRCLE_TARGET)
            fprintf("\n[THÔNG BÁO] Robot hoàn thành 1 vòng tròn (360 độ). Dừng xe!\n");
            isFinished = true;
            cmdL = 0; cmdR = 0; vCmd = 0; omCmd = 0;
        else
            refSeq = zeros(Np, 5);
            for i = 1:Np
                t_future = t + (i-1)*dt;
                ref_i = referenceMPC_Seamless(t_future, R, wr, T_ENTRY, P0, P1, P2, P3, PHI_JOIN, DIR, LOOKAHEAD_RAD, ORBIT_CX, ORBIT_CY);
                refSeq(i, :) = [ref_i(1), ref_i(2), ref_i(3), V_SCALE*ref_i(4), ref_i(5)];
            end
            
            xr = refSeq(1,1); yr = refSeq(1,2); thetaRef = refSeq(1,3); 
            vRef = refSeq(1,4); omRef = refSeq(1,5);
            
            X_curr = [x; y; theta];
            [v_mpc, om_mpc] = solveMPC_ESO(X_curr, refSeq, eso, v_last, om_last, dt, Np, Nc, Q_diag, R_diag, V_MAX, V_MIN, OM_MAX, OM_MIN, qp_options);
            
            vCmd = v_mpc; omCmd = om_mpc;
            v_last = vCmd; om_last = omCmd;
            
            exw = xr - x; eyw = yr - y;
            ex_f =  cos(theta)*exw + sin(theta)*eyw;
            ey_f = -sin(theta)*exw + cos(theta)*eyw;
            eth_f = wrapToPiLocal(thetaRef - theta);
            rerr_f = (t > T_ENTRY) * (hypot(x - ORBIT_CX, y - ORBIT_CY) - R);
            
            [cmdL_des, cmdR_des] = bodyToCmd(vCmd, omCmd, r, b, GEAR_RATIO, RPM_PER_CMD_L, RPM_PER_CMD_R, CMD_LIMIT, CMD_MIN_MOVE);
            cmdL = sat(cmdL_des, cmdL_old - D_CMD_LIM_RUN, cmdL_old + D_CMD_LIM_RUN);
            cmdR = sat(cmdR_des, cmdR_old - D_CMD_LIM_RUN, cmdR_old + D_CMD_LIM_RUN);
            
            lapNow = max(0, wr*max(0, t - T_ENTRY)/(2*pi));
        end
        
        %% --- GỬI LỆNH & LOG ---
        cmdL_old = cmdL; cmdR_old = cmdR;
        if USE_HARDWARE
            writeline(due, sprintf("M,%d,%d", int16(round(cmdL)), int16(round(cmdR))));
            if SEND_ERROR_TO_DUE && mod(idx, ERR_SEND_EVERY_N) == 0
                sendErrorToDue(due, ex_f, ey_f, eth_f);
            end
        end
        
        LOG(idx,:) = [t 1 x y theta xr yr thetaRef ex_f ey_f eth_f rerr_f vRef omRef vCmd omCmd cmdL cmdR last_rpmL last_rpmR double(fbFresh) lapNow 0 0 vCmd omCmd eso(1) eso(2) eso(3) 0];
        robotX(end+1) = x; robotY(end+1) = y; %#ok<SAGROW>
        
        if mod(idx, 5) == 0
            set(hRobotPath, 'XData', robotX, 'YData', robotY);
            set(hRobotNow,  'XData', x, 'YData', y);
            set(hTargetNow, 'XData', xr, 'YData', yr);
            title(sprintf('MPC-ESO High Precision | t=%.1fs | ex=%.4fm ey=%.4fm', t, ex_f, ey_f));
            drawnow limitrate;
        end
        
        idx = idx + 1;
        pause(max(0, Ts - toc(loopStart)));
    end
catch ME
    fprintf(2, '\nDừng do lỗi: %s\n', ME.message);
end
if USE_HARDWARE, stopRobot(due); end
%% ===================== 8. BÁO CÁO SAI SỐ BÁM QUỸ ĐẠO =====================
validIdx = find(~isnan(LOG(:, 1)));
if ~isempty(validIdx)
    LOG = LOG(validIdx, :); 
    t_log     = LOG(:, 1);
    ex_list   = LOG(:, 9);   
    ey_list   = LOG(:, 10);  
    eth_list  = LOG(:, 11);  
    rerr_list = LOG(:, 12);  
    
    fprintf("\n================ BÁO CÁO SAI SỐ BÁM QUỸ ĐẠO ================\n");
    fprintf("Thời gian bám quỹ đạo: %.2f giây\n", t_log(end));
    fprintf("------------------------------------------------------------\n");
    fprintf("1. Sai số dọc ex (Body Frame):\n");
    fprintf("   - RMS: %8.4f m | Max: %8.4f m\n", rms(ex_list), max(abs(ex_list)));
    fprintf("2. Sai số ngang ey (Body Frame):\n");
    fprintf("   - RMS: %8.4f m | Max: %8.4f m\n", rms(ey_list), max(abs(ey_list)));
    fprintf("3. Sai số góc e_theta:\n");
    fprintf("   - RMS: %8.4f rad | Max: %8.4f rad\n", rms(eth_list), max(abs(eth_list)));
    fprintf("============================================================\n");
    
    figure(2); clf; hold on; grid on; axis equal;
    plot(refPlot(:,1), refPlot(:,2), 'k-', 'LineWidth', 2.5, 'DisplayName', 'Reference');
    plot(LOG(:,3), LOG(:,4), 'r--', 'LineWidth', 2.0, 'DisplayName', 'Differential-drive MPC');
    plot(LOG(1,3), LOG(1,4), 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.9 0.7 0], 'DisplayName', 'Start P0');
    plot(LOG(end,3), LOG(end,4), 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.5 0.1 0.6], 'DisplayName', 'End');
    xlabel('x (m)'); ylabel('y (m)'); title('MPC-ESO Seamless Tracking Result');
    legend('Location', 'southeast'); xlim([-1.7 1.7]); ylim([-1.7 1.7]);
    
    figure(3); clf; hold on; grid on;
    plot(t_log, ex_list, 'LineWidth', 1.5, 'DisplayName', 'e_{x}^{b} [m]');
    plot(t_log, ey_list, 'LineWidth', 1.5, 'DisplayName', 'e_{y}^{b} [m]');
    plot(t_log, eth_list, 'LineWidth', 1.5, 'DisplayName', 'e_{\theta} [rad]');
    plot(t_log, rerr_list, 'LineWidth', 1.5, 'DisplayName', 'r-R [m]');
    xlabel('t (s)'); ylabel('error'); title('Sai so MPC-ESO (Tuned)');
    legend('Location', 'northeast'); xlim([0 max(t_log)+2]);
end
%% ===================== HÀM QUỸ ĐẠO THAM CHIẾU =====================
function ref = referenceMPC_Seamless(tPath, R, wr, T_ENTRY, P0, P1, P2, P3, PHI_JOIN, DIR, LOOKAHEAD_RAD, cx, cy)
    v_circle = R * wr; 
    
    if tPath <= T_ENTRY
        u = min(max(tPath/T_ENTRY, 0), 1);
        [s, dsdt, d2sdt2] = smoothStepMatch(u, T_ENTRY, v_circle, P2, P3);
        
        P = bezierPoint(s, P0, P1, P2, P3);
        xr = P(1); yr = P(2);
        
        dP = bezierDeriv(s, P0, P1, P2, P3);
        ddP = bezierSecond(s, P0, P1, P2, P3);
        
        vel = dP * dsdt; 
        acc = ddP * (dsdt^2) + dP * d2sdt2;
        vRef = hypot(vel(1), vel(2));
        
        if vRef < 1e-4
            thetaRef = atan2(P1(2)-P0(2), P1(1)-P0(1)); 
            omRef = 0;
        else
            thetaRef = atan2(vel(2), vel(1));
            om_bez = (vel(1)*acc(2) - vel(2)*acc(1)) / (vRef^2 + 1e-6);
            blend = u^3;
            omRef = (1 - blend)*om_bez + blend*(DIR * wr);
        end
    else
        tC = tPath - T_ENTRY;
        phi = PHI_JOIN + DIR * wr * tC;
        phiCmd = phi + DIR * LOOKAHEAD_RAD;
        
        xr = cx + R * cos(phiCmd); 
        yr = cy + R * sin(phiCmd);
        thetaRef = wrapToPiLocal(phiCmd + DIR * pi/2);
        vRef = v_circle; 
        omRef = DIR * wr; 
    end
    ref = [xr, yr, thetaRef, vRef, omRef, tPath];
end

function [s, dsdt, d2sdt2] = smoothStepMatch(u, T, V_target, P2, P3)
    L3 = norm(3 * (P3 - P2)); 
    if L3 < 1e-6, L3 = 1; end
    
    D = V_target / L3; 
    K = D * T;         
    
    a = K - 2;
    b = 3 - K;
    
    u = min(max(u,0),1);
    s = a*u^3 + b*u^2;
    dsdu = 3*a*u^2 + 2*b*u;
    d2sdu2 = 6*a*u + 2*b;
    
    dsdt = dsdu / T;
    d2sdt2 = d2sdu2 / (T^2);
end
%% ===================== CÁC HÀM PHỤ HỖ TRỢ =====================
function z_next = updateESO_3Axis(z, Y, v, om, dt, Beta_x, Beta_y, Beta_th, max_fx, max_fy, max_fth)
    e_x  = Y(1) - z(1); e_y  = Y(2) - z(2); e_th = wrapToPiLocal(Y(3) - z(3));
    dx_hat  = v * cos(z(3)) + z(4) + Beta_x(1) * e_x; dfx_hat = Beta_x(2) * e_x;
    dy_hat  = v * sin(z(3)) + z(5) + Beta_y(1) * e_y; dfy_hat = Beta_y(2) * e_y;
    dth_hat  = om + z(6) + Beta_th(1) * e_th;         dfth_hat = Beta_th(2) * e_th;
    z_next = z + dt * [dx_hat; dy_hat; dth_hat; dfx_hat; dfy_hat; dfth_hat];
    z_next(3) = wrapToPiLocal(z_next(3));
    z_next(4) = sat(z_next(4), -max_fx,  max_fx);
    z_next(5) = sat(z_next(5), -max_fy,  max_fy);
    z_next(6) = sat(z_next(6), -max_fth, max_fth);
end
function [v_opt, om_opt] = solveMPC_ESO(X0, refSeq, eso, v_last, om_last, dt, Np, Nc, Q_diag, R_diag, V_MAX, V_MIN, OM_MAX, OM_MIN, options)
    nx = 3; nu = 2; Q = diag(Q_diag); R = diag(R_diag);
    S_x = zeros(nx * Np, nx); S_u = zeros(nx * Np, nu * Nc); S_d = zeros(nx * Np, nx);
    Q_bar = kron(eye(Np), Q); R_bar = kron(eye(Nc), R);
    A_cell = cell(Np, 1); B_cell = cell(Np, 1);
    for i = 1:Np
        th_r = refSeq(i, 3); vr = refSeq(i, 4);
        A_cell{i} = eye(3) + [0, 0, -vr*sin(th_r)*dt; 0, 0, vr*cos(th_r)*dt; 0, 0, 0];
        B_cell{i} = [cos(th_r)*dt, 0; sin(th_r)*dt, 0; 0, dt];
    end
    A_cum = eye(nx); S_d_cum = zeros(nx, nx);
    for i = 1:Np
        A_cum = A_cell{i} * A_cum; S_x((i-1)*nx+1 : i*nx, :) = A_cum;
        S_d_cum = S_d_cum + A_cell{i} * dt; S_d((i-1)*nx+1 : i*nx, :) = S_d_cum;
    end
    for i = 1:Np
        for j = 1:min(i, Nc)
            A_prod = eye(nx);
            for k = j+1:i, A_prod = A_cell{k} * A_prod; end
            S_u((i-1)*nx+1 : i*nx, (j-1)*nu+1 : j*nu) = A_prod * B_cell{j};
        end
    end
    e0 = X0 - refSeq(1, 1:3)'; e0(3) = wrapToPiLocal(e0(3));
    H = S_u' * Q_bar * S_u + R_bar;
    f = S_u' * Q_bar * (S_x * e0 + S_d * eso);
    u_ref = [refSeq(1, 4); refSeq(1, 5)];
    lb = repmat([V_MIN; OM_MIN] - u_ref, Nc, 1);
    ub = repmat([V_MAX; OM_MAX] - u_ref, Nc, 1);
    [dU_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, [], options);
    if exitflag == 1 && ~isempty(dU_opt)
        v_opt = u_ref(1) + dU_opt(1); om_opt = u_ref(2) + dU_opt(2);
    else
        v_opt = sat(u_ref(1), V_MIN, V_MAX); om_opt = sat(u_ref(2), OM_MIN, OM_MAX);
    end
    v_opt = sat(v_opt, V_MIN, V_MAX); om_opt = sat(om_opt, OM_MIN, OM_MAX);
end
function [cmdL, cmdR] = bodyToCmd(v, om, r, b, gear, rpmPerCmdL, rpmPerCmdR, cmdLimit, minCmd)
    wL = (v - b*om/2)/r; wR = (v + b*om/2)/r;
    rpmL_motor = (wL*60/(2*pi))*gear; rpmR_motor = (wR*60/(2*pi))*gear;
    cmdL = sat(applyMinCmd(rpmL_motor/rpmPerCmdL, minCmd), -cmdLimit, cmdLimit);
    cmdR = sat(applyMinCmd(rpmR_motor/rpmPerCmdR, minCmd), -cmdLimit, cmdLimit);
end
function [ok, rpmL, rpmR] = readFeedbackRPM(due)
    ok = false; rpmL = NaN; rpmR = NaN;
    while due.NumBytesAvailable > 0
        line = strtrim(readline(due));
        if startsWith(line, "FB,")
            parts = split(line, ",");
            if numel(parts) >= 3
                r1 = str2double(parts(2)); r2 = str2double(parts(3));
                if ~isnan(r1) && ~isnan(r2), rpmL = r1; rpmR = r2; ok = true; end
            end
        end
    end
end
function sendErrorToDue(due, ex_body, ey_body, eth_body)
    ex_mm = max(min(round(ex_body*1000), 32767), -32768);
    ey_mm = max(min(round(ey_body*1000), 32767), -32768);
    eth_mrad = max(min(round(eth_body*1000), 32767), -32768);
    writeline(due, sprintf("ERR,%d,%d,%d", int16(ex_mm), int16(ey_mm), int16(eth_mrad)));
end
function stopRobot(due)
    try
        for k = 1:120, writeline(due, "M,0,0"); pause(0.02); end
        writeline(due, "STOP");
    catch, end
end
function P = bezierPoint(s, P0, P1, P2, P3), s = min(max(s,0),1); P = (1-s)^3*P0 + 3*(1-s)^2*s*P1 + 3*(1-s)*s^2*P2 + s^3*P3; end
function D = bezierDeriv(s, P0, P1, P2, P3), s = min(max(s,0),1); D = 3*(1-s)^2*(P1-P0) + 6*(1-s)*s*(P2-P1) + 3*s^2*(P3-P2); end
function DD = bezierSecond(s, P0, P1, P2, P3), s = min(max(s,0),1); DD = 6*(1-s)*(P2 - 2*P1 + P0) + 6*s*(P3 - 2*P2 + P1); end
function y = sat(x, xmin, xmax), y = min(max(x, xmin), xmax); end
function y = applyMinCmd(x, minCmd)
    if x > 0 && x < minCmd, y = minCmd; elseif x < 0 && x > -minCmd, y = -minCmd; else, y = x; end
end
function a = wrapToPiLocal(a), a = mod(a + pi, 2*pi) - pi; end