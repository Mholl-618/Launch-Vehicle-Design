%Del Vee Comp Script
clear all, clc
del_v = 11000;
f_one = linspace(.3,.6,100);
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

    [mprop_upper(i),~,m_final_upper,m_init_upper(i)] = mass_vals(f_two(i)*del_v,375,.07,15000);
    [mprop_lower(i),~,m_final_lower,m_init_lower(i)] = mass_vals(f_one(i)*del_v,350,.06,m_init_upper(i));
    masses = [m_init_lower(i),mprop_lower(i),m_init_upper(i),mprop_upper(i)];
%     tspan_lower = ((mprop_lower(i)*.99)/(495.2907320178509*7))+5;
%     tspan_upper = ((mprop_upper(i)*.98)/516.6562)+1;
%     t_cuts = [tspan_lower tspan_upper];
%     tspan = 0:.01:sum(t_cuts);
%     [T1{i},Y1{i}] = ode45(@(t,x)rocket_man(t,x,masses,t_cuts),tspan,[0 0],odeset('events',@altEvent));
%     cell_convert = Y1{i};
%     deltaVee_with_losses(i) = max(cell_convert(:,2));
end
[M,I] = min(m_init_lower);
% [hAx,hLine1,hLine2] = plotyy(f_two,m_init_lower,f_two,deltaVee_with_losses);
% title('Mass and Delta V VS. F two');
% ylabel(hAx(1),'Mass (kg)') % left y-axis 
% ylabel(hAx(2),'Delta V with Losses (m/s)') % right y-axis
% xlabel('F two')

% figure
% surf(f_two,m_init_lower,meshgrid(deltaVee_with_losses))

figure
plot(f_two,m_init_lower)
hold
plot(f_two(I),M,'r*')
xlabel('Stage 2 fractional \Delta V')
ylabel('Total Launch Vehicle Mass (Kg)')
title('Staging Optimization')
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
SA = (2.5908)^2*pi; %1 inch thick walls give us a safety factor of 
vol_low_ch4 = lower_ch4/rho_lch4; % 4.12 in compressive yield failure, no clue on buckling
vol_low_ox = lower_lox/rho_lox;
vol_up_ch4 = upper_ch4/rho_lch4;
vol_up_ox = upper_lox/rho_lox;
hght_lower = (vol_low_ch4+vol_low_ox)/SA*1.05;
hght_upper = (vol_up_ch4+vol_up_ox)/SA*1.05;
total_hght = hght_lower+hght_upper; 
tank_mass = pi*(1.8288^2-1.8034^2)*total_hght*2685; %2685 is the density of 2195 aluminum
dome_mass = 4/3*pi*(1.8288^3-1.8034^3)*2685*.5;
dome_per_stage = dome_mass*3;

%Lower Tanks
lower_tank_mass = pi*(1.8288^2-1.8034^2)*hght_lower*2685;
lower_engine_mass = 5*800; %each raptor is roughly 800 kg
lower_inert_mass = lower_tank_mass+lower_engine_mass+dome_per_stage;
%Upper Tanks
upper_tank_mass = pi*(1.8288^2-1.8034^2)*hght_upper*2685; 
upper_engine_mass = 1200; %vacuum raptor is heavy
upper_inert_mass = upper_tank_mass+upper_engine_mass+dome_per_stage;


%% ODE TIMEEEEE
pos_1 = [0];
vel_1 = [0];
tspan = 0:.1:460;
masses = [m_init_lower,lower_prop,(m_init_upper),(upper_prop)];

% for n = 1:length(T1)
%     dY = zeros(length(T1{n}),2);
%     
%     for i = 1:length(T1)
%         dY(i,:) = rocket_man(T1(i),Y1(i,:),masses); 
%     end
% end

 tspan_lower = ((lower_prop)/(495.2907320178509*7))+5;
    tspan_upper = ((masses(4))/516.6562)+1;
    t_cuts = [tspan_lower tspan_upper];
    tspan = 0:.01:sum(t_cuts);
    [T1,Y1] = ode45(@(t,x)rocket_man(t,x,masses,t_cuts),tspan,[0 0],odeset('events',@altEvent));
for i = 1:length(T1)
        dY(i,:) = rocket_man(T1(i),Y1(i,:),masses,t_cuts); 
end

q = [];
drag = [];
for i = 1:length(T1)
        [rho,~,~,a,~] = std_atmosphere(Y1(i,1));
        SA = (5.1816/2)^2*pi; %m^2 17 ft OD
        cd = .075;
        V = Y1(i,2);
        q(i) = .5*rho*(V^2); 
        drag(i) = q(i)*SA*cd;
end


figure
[hAx,hLine1,hLine2] = plotyy(T1,((dY(:,2)+go)./go),T1,Y1(:,2));
title('Acceleration and Velocity VS. T+');
ylabel(hAx(1),'Acceleration (g)') % left y-axis 
ylabel(hAx(2),'Velocity (m/s)') % right y-axis
xlabel('T+ (seconds)')
% ylim([0 7]);
figure
plot(T1,q)
title('Dynamic Pressure vs T+')
xlabel('T+ (seconds)')
ylabel('Q (pascals)')
[q_max, ind] = max(q);
fprintf('Delta V with atm losses is %.2f \n',max(dY(:,1)))
fprintf('Max Q is %.2f pascals',q_max)
fprintf(' at %.2f seconds \n',T1(ind))

