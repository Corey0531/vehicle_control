%%
% 第二次模擬產生結果 y2 和同樣的時間 t
load('LookAheadTime1.mat');  
X1 = X; Y1 = Y;

load('LookAheadTime2.mat');  
X2 = X; Y2 = Y;


% 畫圖比較
figure;
plot(X1, Y1, 'b-', 'DisplayName', 'LookAheadTime1');
hold on;
plot(X2, Y2, 'r--', 'DisplayName', 'LookAheadTime2');
xlabel('X 位置 [m]');
ylabel('Y 位置 [m]');
legend;
title('Comparison: LookAheadTime 1 vs. 2');
grid on;
