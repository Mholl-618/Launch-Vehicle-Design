%Del Vee Comp Script
clear all, clc
del_v = 10750;
f_one = .3;
f_two = ones(1,length(f_one))-f_one;

%% Top stage
%Raptor Vacuum Engine Performance
go = 9.80665;
ISP = 375; %assume vacuum performance
ve = ISP*go;
m_pay = 13000; %kg, aka 13 metric tons
m_init_upper = [];
m_final_upper = [];
m_init_lower = [];
m_final_lower = [];
mprop_upper = [];
mprop_lower = [];
x = [0 0 0 0];
del_t = .01;
t = 0:del_t:550;

for i = 1:length(f_two)

    [mprop_upper(i),~,m_final_upper,m_init_upper(i)] = mass_vals(f_two(i)*del_v,375,.08,13000);
    [mprop_lower(i),~,m_final_lower,m_init_lower(i)] = mass_vals(f_one(i)*del_v,350,.08,m_init_upper(i));
    masses = [m_init_lower(i) mprop_lower(i) m_init_upper(i) mprop_upper(i)];
    m1_fi = mprop_lower(i);

for m = 1:length(t)
    [t(m),dx(m,:),masses(m+1,:),fuel_loss(m),q_act(m),fpa] = throttling_launch(t(m),x(m,:),masses(m,:),del_t,mprop_lower(i));
    accel_x= dx(m,3);
    accel_y= dx(m,4);
    vel_x = x(m,3)+(accel_x*del_t);
    vel_y = x(m,4)+(accel_y*del_t);
    pos_x = x(m,1)+(vel_x*del_t);
    pos_y = x(m,2)+(vel_y*del_t);
    x(m+1,:) = [pos_x pos_y vel_x vel_y];
end 

    max_vels(i) = max(dx(:,1));
end

[M,I] = max(max_vels);
figure
plot(f_two,m_init_lower)
hold
plot(f_two(I),m_init_lower(I),'r*')
xlabel('Stage 2 fractional \Delta V')
ylabel('Total Launch Vehicle Mass (Kg)')
title('Staging Optimization')
text(f_two(I),M,int2str(M),'VerticalAlignment','bottom')
text(f_two(I),M,num2str(f_two(I)),'VerticalAlignment','Top')    
hold
clc
fprintf('Upper fuel mass %.2f \n',mprop_upper(I))
fprintf('Lower fuel mass %.2f \n',mprop_lower(I))
%%
figure 
plot(f_two,max_vels)
hold
plot(f_two(I),max_vels(I),'b*')
xlabel('Stage 2 fractional \Delta V')
ylabel('Delta V After Atmospheric Losses and Throttling')
title('Staging Optimization')
%% Prop Calcs
upper_prop = mprop_upper(I);
lower_prop = mprop_lower(I);
m_init_upper = m_init_upper(I);
m_init_lower = m_init_lower(I);

masses = [m_init_lower,lower_prop,m_init_upper,upper_prop];

rho_lox = 1141; %kg per m^3
rho_lch4 = 422.62; %kg per m^3 

stoch_ratio = 3.8; %grams of LO2 per grams of lch 4 


lower_ch4 = lower_prop/2.995;
lower_lox = stoch_ratio*lower_ch4;

upper_ch4 = upper_prop/2.995;
upper_lox = stoch_ratio*upper_ch4;

m_dot_SLR = 1700e3/(350*go); %thrust  divided by isp and gravity
m_dot_VR = 1900e3/(375*go); %Same as above, both ar Kg/s
SA = (1.80339999997968)^2*pi; %1 inch thick walls give us a safety factor of 
vol_low_ch4 = lower_ch4/rho_lch4; % 4.12 in compressive yield failure, no clue on buckling
vol_low_ox = lower_lox/rho_lox;
vol_up_ch4 = upper_ch4/rho_lch4;
vol_up_ox = upper_lox/rho_lox;
hght_lower = (vol_low_ch4+vol_low_ox)/SA*1.1;
hght_upper = (vol_up_ch4+vol_up_ox)/SA*1.1;
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
t = 0:.01:555;
masses = zeros(length(t),4);
masses = [m_init_lower,lower_prop,m_init_upper,upper_prop];
dx = zeros(length(t),4);
x = zeros(length(t),4);
x(1,:) = [0 0 0 0];
del_t = .01;

x = [0 0 0 0];

for i = 1:length(t)
    [t(i),dx(i,:),masses(i+1,:),fuel_loss(i),q_act(i),fpa(i)] = throttling_launch(t(i),x(i,:),masses(i,:),del_t,mprop_lower(I));
    accel_x= dx(i,3);
    accel_y= dx(i,4);
    vel_x = x(i,3)+(accel_x*del_t);
    vel_y = x(i,4)+(accel_y*del_t);
    pos_x = x(i,1)+(vel_x*del_t);
    pos_y = x(i,2)+(vel_y*del_t);
    x(i+1,:) = [pos_x pos_y vel_x vel_y];
end
%%
plot(t,dx(:,3))
figure
plot(t,dx(:,4))
figure
plot(x(:,1),x(:,2))
accel = sqrt(dx(:,3).^2+dx(:,4).^2);
figure 
plot(t,accel)


%%

figure
[hAx,hLine1,hLine2] = plotyy(t,dx(:,2)./go,t,dx(:,1)./go);
title('Acceleration and Velocity VS. T+');
ylabel(hAx(1),'Acceleration (g)') % left y-axis 
ylabel(hAx(2),'Velocity (m/s)') % right y-axis
xlabel('T+ (seconds)')
% ylim([0 7]);
figure
plot(t,q_act)
title('Dynamic Pressure vs T+')
xlabel('T+ (seconds)')
ylabel('Q (pascals)')
[q_max, ind] = max(q_act);
fprintf('Delta V with atm losses is %.2f \n',max(dx(:,1)))
fprintf('Max Q is %.2f pascals',q_max)
fprintf(' at %.2f seconds \n',t(ind))

