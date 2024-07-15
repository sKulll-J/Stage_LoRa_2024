I_sleep = [50,40,30,20,10,7,5,3,2,1];
V_alim = 5;
I_stand = 60;
T = [16,30,60,120,240,480,960,1920, 2040, 4096];
I_send = 90+I_stand;
T_stand = 4;
T_send = 0,6155;
T_sleep = T-T_stand-T_send

I_sleep = I_sleep';

I_moy = (I_sleep.*T_sleep+I_stand*T_stand+I_send*T_send)./T

P_moy = V_alim*(I_moy/1000);

auto = (3.7./P_moy)/60

z = peaks(25);

figure 1
surf(T, I_sleep, auto)

figure 2
contour(T, I_sleep, auto,16)
