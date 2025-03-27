D = 0.7;%D改這
C = 2.45;
B =4;
E=0.95;

x = 0:0.01:1;
y = D *sin(C* atan(B*x-E*(B*x-atan(B*x))));


plot(x,y,'r','LineWidth',1);%顏色改第三個參數
xlabel('K');
ylabel('\mu','FontSize',14);
yticks(0:0.1:D);%y軸一格是0.1
title('\mu -Slip Curve');
grid on;
