clear all
close all
clear figure
clf
fid = csvread("10kp_exp.csv");
mx=max(fid(:,2));
mn=min(fid(:,2));

delta = 0
chirp_lenght = 67
chirp_cnt = 20

y = ones(size(fid)(1))(:, 1)*mx;
y2=1.-((mod(((1:length(fid))-delta),chirp_lenght) ==0)*2);



x=fid(:, 1);

y1 = fid(:,2);


y_pm = [y1(2:length(y1), 1);0];
y_pp = [0;y1(1:(length(y1)-1), 1)];

y_smth = (y_pp+y1+y_pm)/3;

y_pm = [y_smth(2:length(y_smth), 1);0];
y_pp = [0;y_smth(1:(length(y_smth)-1), 1)];
chg = 1-(((y_pp-y_smth).*(y_smth-y_pm))>0);
y=y.*chg;
size(fid)

fct = mx;
ceil = 900;
d_fid_s = abs(diff(y_smth));
d_fid = (d_fid_s>=ceil)*fct;
size(d_fid)

d_fid_cl = ((d_fid(2:end, 1)-d_fid(1:end-1, 1))>=1)*fct;

reg_ofset = 0.02386;
reg_delt = 0.0010239;
y_reg = (mod(x-reg_ofset, reg_delt)<4.5e-6)*fct;


figure 1
hold on
%stem(x,y,"r");
plot(x,fid(:,2), "b.-");

%stem(x(2:end, 1),d_fid, "g--");
%stem(x(2:end-1, 1),d_fid_cl, "r");
stem(x, y_reg, "g");

plot(x(2:end, 1),d_fid_s, "g--");

hold off




figure 2




y_b = (y1-mn)*180/(mx-mn);
y_b2 = (y-mn)*180/(mx-mn);

cor=((mod(((1:0.1:18)),1) ==0)*max(fid(:,1)))';

hold on
plot(y_b2,x,"b")
plot(cor,"g")
plot(y_b, x,"r")

hold off

chrp_sel=4;

while 1
  disp(y_reg(100)==0 & 0!=1)
  figure 3
    grid on
  cnt = 0;
  idx=1;
  while ((cnt!=chrp_sel))
    if (y_reg(idx) !=0)
      cnt++;
    end
    idx++;
  end

  cnt=cnt
  chrp_sel
  p_st = idx
  idx++
   while  (y_reg(idx)==0)
    idx++;
  end
  p_ed = idx

  xmax = fid(p_st, 1);
  xmin = fid(p_ed, 1);
  xd =  (xmax-xmin)/128
  x_axe = mod(fid(p_st:p_ed, 1),xd)
  %x_axe_x =
  hold on
  plot(fid(p_st:p_ed, 1),fid(p_st:p_ed, 2))
  stem(x_axe_x, x_axe, "r");
  hold off
  chrp_sel=input("numero de chirp :");

end

