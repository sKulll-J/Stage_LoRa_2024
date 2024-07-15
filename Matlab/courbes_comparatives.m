clear figure
close all
clear all
grid on

x = linspace(0,0.80, 100);

y10= -2.27*x+4.1;

y11 = -1.43*x+4.1;
y12 = (-1.43-0.90)*x+4.1;

y20 = -1.97*x+4.1;

y21 = -0.99*x+4.1;
y22 = (-0.99-0.90)*x+4.1;

y30 = -1.556507384*x+4.1;
y31 = -0.87*x+4.1;

figure 1
hold on
grid on
grid minor on
plot(x*24,y10, "b")
plot(x*24, y11, "b:")
%plot(x*24, y12, "b_.")

plot(x*24,y20, "r")
plot(x*24, y21, "r:")
%plot(x*24, y22, "r_.")

plot(x*24,y30, "g")
plot(x*24, y31, "g:")


axis([0 20 3.4 4.1 ])
legend("avec LCD 21s - Pratique", "avec LCD 21s - Theorique", "sans LCD 21s - Pratique", "sans LCD 21s - Theorique", "sans LCD 10min - Pratique", "sans LCD 10min - Theorique");
xlabel ("temps (h)");
ylabel ("tension de baterie (V)");
title ("Comparaison entre l'autonomie théorique et pratique pour plusieurs modes de fonctionements");
hold off

figure 2
plot([12, 17, 19.49], [-0.84, -0.90, -0.69]); %derniere valeur a déterminer par la mesure
grid on

figure 3
plot([12,17,19.49], [7.25,8.30 ,10.38])
grid on
