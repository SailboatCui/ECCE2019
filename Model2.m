clear;
clc;


%% Circuit Parameters

% testing 1
lambda = 0.1;
r = 100;
l = 6.8e-6;
c = 1e-6;
Vin = 12;
Vout = 40;
Vref1 = Vout;
Vref2 = 41;
Toff = 200e-9;

m1 = Vin/l;
m2 = (Vout-Vin)/l;
Ton = Toff*m2/m1;
T = Ton + Toff;
Iavg = Vout^2/r/Vin;
Iref1 = Iavg + m2*Toff/2;


%% Controller Parameters
zz = 0.968;
kth = 1;

%% Controller
z=tf('z',1);
zz = 0.968;
kth = 1;
C = kth*(z-zz)/(z-1);

%%  Model 2 Step Response
alpha = Toff/r/c;
beta = Toff^2/2/l/c;
z1 = 1 - 2*T/r/c - (lambda^2+(1-lambda)^2)*beta;
g = lambda*Toff/c - (Vout/Vin)*(l/r/c)*(1-lambda*alpha-lambda^2*beta);
z0 = -((1-lambda*alpha-lambda^2*beta)*(Vout/Vin)*(l/r/c) + (1+(1-lambda)*alpha-2*T/c/r-lambda^2*beta)*(1-lambda)*Toff/c)/(lambda*Toff/c - (Vout/Vin)*(l/r/c)*(1-lambda*alpha-lambda^2*beta));
Lz1 = (g*(z-z0)/(z-z1)/z)
[y0,x0,t]=step(feedback(C*Lz1,1));
y2 = [y0];


%% Plot Comparsion
timeoffset = 1e-3; % timeoffset of the step
scfac = 1; % scfac : scale factor of the simulation data
M=100;

rangec = 90;
mp3 = stairs(1:rangec,y2(1:rangec)','linewidth',1.5);                 % Plot simulation data

%t = title({'Model Accuracy Comparision','\lambda = 0.1 (under 1V step up)'});
set(t,'FontSize',10,'FontName','Arial');

lg=legend([mp3],'Model II','Location','Best');
set(lg,'fontsize',8,'FontName','Arial');
%set(p1,'legend','Noise Floor');

grid on;
grid minor;
box off;
xlim([0,80]);
% ylim([0.25,5]);
set(gca,'FontSize',10,'FontName','Arial');

l1 = xlabel('Cycles (N)');
set(l1,'FontSize',10,'FontName','Arial');
l2 = ylabel('Voltage');
set(l2,'FontSize',10,'FontName','Arial');



% fLz2=feedback((kth*(z-zz)/(z-1)),g*(z-z0)/(z-z1)/z);
% tz = (z-0.9)/(z-0.3);
% N = log(1/9)/log(0.99);


% N1 = 10;
% N2 = 1e3;
% N3 = 1e3;
% 
% zz = linspace(0.95,z1,N1);
% kth = linspace(0,10,N2);
% ZM = -1*ones(N1,N2);
% 
% for i = 1:N1
%     for j = 1:N2
%         p = linspace(zz(i)*0.99,0.01,N3);
%         for m = 1:N3-1
%             if abs((kth(j)*(p(m)-zz(i))/(p(m)-1)*(g*(p(m)-z0)/(p(m)-z1)/p(m))+1))<= 1e-3 
%                 ZM(i,j) = p(m);
%             end
%         end
%     end
% end
% 
% [X,Y]= meshgrid(zz,kth);
%surf(X,Y,ZM');
% figure(1);
% rlocus(Lz1);
% 
% figure(2);
% [y0,x0,t]=step(fLz1);
% stairs(y0);

% figure(3);
% [y0,x0,t]=step(tz);
% plot(y0);
% y = [0;y0];