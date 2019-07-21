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

%%  Mak1 Step Response
A1 = [0,0;0,-1/r/c];
b1 = [1/l;0];
A2 = [0,-1/l;1/c,-1/r/c];
b2 = [1/l;0];

Xp = [Iref1;Vref1];
Phi = expm(A1*Ton)*expm(A2*Toff);
gamma = A1*expm(A1*Ton)*expm(A2*Toff)*Xp+expm(A1*Ton)*b1*Vin+A1*expm(A1*Ton)*inv(A2)*(expm(A2*Toff)-eye(2))*b2*Vin;
delta = eye(2);
td = Toff*lambda;
E = expm(A2*td);

Ts = -1;
sys = ss(Phi,gamma,delta(1,:),0,Ts);
Gitz = tf(sys);
sys = ss(Phi,gamma,delta(2,:),0,Ts);
Gvtz = tf(sys);

Gvis = tf(Gvtz.Numerator,Gitz.Numerator,Ts);
z = tf('z',Ts);

Gvi = E(2,1)*z^-1+E(2,2)*Gvis*z^-1;
[z0,g] = zero(Gvi);
z1 = pole(Gvi);

Lz1 = Gvi;
[y0,x0,t] = step(feedback(C*Lz1,1));
y3 = [y0];


%% Plot Comparsion
timeoffset = 1e-3; % timeoffset of the step
scfac = 1; % scfac : scale factor of the simulation data
M=100;

rangec = 90;
mp3 = stairs(1:rangec,y3(1:rangec)','linewidth',1.5);                 % Plot simulation data

%t = title({'Model Accuracy Comparision','\lambda = 0.1 (under 1V step up)'});
set(t,'FontSize',10,'FontName','Arial');

lg=legend([mp3],'Model III','Location','Best');
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