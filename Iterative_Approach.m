%Del Vee Comp Script
clear all, clc
del_v = 10000;
f_one = linspace(.3,.7,100);
f_two = ones(1,length(f_one))-f_one;

%% Top stage
%Raptor Vacuum Engine Performance
go = 9.80665;
ISP = 375; %assume vacuum performance
ve = ISP*go;
m_pay = 15000; %kg, aka 15 metric tons
m_init_upper = [];
m_final_upper = [];
m_init_lower = [];
m_final_lower = [];
mprop_upper = [];
mprop_lower = [];


for i = 1:length(f_two)

    [mprop_upper(i),~,m_final_upper,m_init_upper(i)] = mass_vals(f_two(i)*del_v,375,.1,15000);
    [mprop_lower(i),~,m_final_lower,m_init_lower(i)] = mass_vals(f_one(i)*del_v,350,.1,m_init_upper(i));
    total_mass(i) = m_init_lower(i)+m_init_upper(i);
end
[M,I] = min(total_mass);
figure
plot(f_two,total_mass)
hold
plot(f_two(I),M,'r*')
xlabel('Stage 2 fractional \Delta V')
ylabel('Total Launch Vehicle Mass (Kg)')
text(f_two(I),M,int2str(M),'VerticalAlignment','bottom')
text(f_two(I),M,num2str(f_two(I)),'VerticalAlignment','Top')    
hold
clc
fprintf('Upper fuel mass %.2f \n',mprop_upper(I))
fprintf('Lower fuel mass %.2f \n',mprop_lower(I))

%% Prop Calcs
upper_prop = mprop_upper(I);
lower_prop = mprop_lower(I);
m_init_upper = m_init_upper(I);
m_init_lower = m_init_lower(I);

rho_lox = 1141; %kg per m^3
rho_lch4 = 422.62; %kg per m^3 

stoch_ratio = 1.995; %grams of LO2 per grams of lch 4 


lower_ch4 = lower_prop/2.995;
lower_lox = stoch_ratio*lower_ch4;

upper_ch4 = upper_prop/2.995;
upper_lox = stoch_ratio*upper_ch4;

m_dot_SLR = 1700e3/(350*go); %thrust  divided by isp and gravity
m_dot_VR = 1900e3/(375*go); %Same as above, both ar Kg/s

%% ODE TIMEEEEE
pos_1 = [0];
vel_1 = [0];
tspan = 0:.1:310;

[T1,Y1,TE1,YE1,IE] = ode45(@rocket_man,tspan ,[0 0],odeset('events',@altEvent,'AbsTol',1e-12));

dY = zeros(length(T1),2);
for i = 1:length(T1)
    dY(i,:) = rocket_1st_stage(T1(i),Y1(i,:)); 
end

q = [];
drag = [];
for i = 1:length(T1)
        [rho,~,~,a,~] = std_atmosphere(Y1(i,1));
        g = 9.81;
        SA = (3.6576/2)^2*pi; %m^2 12 ft OD
        cd = .075;
        V = dY(i,1);
        q(i) = .5*rho*V^2; 
        drag(i) = q(i)*SA*cd;
end

for i = 1:length(T1);
   if i >=1851;
       dY(i,2) = dY(i,2)-go;
   else;
   end
end
figure
[hAx,hLine1,hLine2] = plotyy(T1,((dY(:,2)+go)./go),T1,Y1(:,2));
title('Acceleration and Velocity VS. Time');
ylabel(hAx(1),'Acceleration (g)') % left y-axis 
ylabel(hAx(2),'Velocity (m/s)') % right y-axis
xlabel('Time (seconds)')
% ylim([0 7]);
figure
plot(T1,q)
[q_max, ind] = max(q);
fprintf('Max Q is %.2f pascals',q_max)
fprintf(' at %.2f seconds \n',T1(ind))

