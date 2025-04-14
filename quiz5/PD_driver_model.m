m = 1270 + 88.6 + 54.4;
Iz = 1536.7; % 偏航慣性矩 [kg*m^2] 
Caf = 46000 * 2; % 前輪側偏剛度 [N/rad] 
Car = 42000 * 2; % 後輪側偏剛度 [N/rad] 
  
l_tot = 2.91; % 軸距 [m] 
la = 1.015; % 前軸到質心距離 [m] 
lb = l_tot - la; % 質心到後軸距離 [m] 
 
u0 = 20; % 縱向速度 [m/s] 
  
A = [0, 1, u0, 0; 
    0, -(Caf+Car)/(m*u0), 0, (lb*Car - la*Caf)/(m*u0) - u0; 
    0, 0, 0, 1; 
    0, (lb*Car - la*Caf)/(Iz*u0), 0, -(la^2*Caf + lb^2*Car)/(Iz*u0)]; 
% B矩陣 
B = [0; 
    Caf / m; 
    0; 
    la * Caf / Iz]; 
  
% C矩陣 
C = eye(4); 
 
% D矩陣 
D = zeros(4,1); 
Kp=1;  
N=100;  
% 清空 SDI 過去資料（可選）
Simulink.sdi.clear;

% 多組 Kd 的值
Kd_values = [0.23, 0.33, 0.43];

% 顏色設定
colors = ['r', 'b', 'g'];  % 紅、藍、綠

for i = 1:length(Kd_values)
    Kd = Kd_values(i);  % 指定本次模擬的 Kd 值

    % 跑模擬
    simOut = sim("PD_driver_model");

    % 取得最新的 Run
    runIDs = Simulink.sdi.getAllRunIDs;
    run = Simulink.sdi.getRun(runIDs(end));

    % 修改 Run 名稱為對應的參數（可幫助整理）
    run.Name = ['Run for Kd = ', num2str(Kd)];

    % 設定信號顏色
    for j = 1:run.getNumSignals
        signal = run.getSignal(j);
        signal.setLineColor(colors(i));  % 設置顏色
    end
    
    % 在標題中顯示 Kd 值（可選）
    Simulink.sdi.addText('Simulation Result for Kd = ' + string(Kd), 'fontSize', 14);
end



