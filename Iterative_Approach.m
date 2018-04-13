%Del Vee Comp Script
clear all, clc
del_v = 9750;
f_one = .3:.02:.5;
f_two = ones(1,length(f_one))-f_one;

% Top stage
% Raptor Vacuum Engine Performance
go = 9.80665;
ISP = 375; %assume vacuum performance
ve = ISP*go;
m_pay = 13000; % kg, aka 13 metric tons
m_init_upper = [];
m_final_upper = [];
m_init_lower = [];
m_final_lower = [];
mprop_upper = [];
mprop_lower = [];
del_t = .05;
t = 0:del_t:1000;
x = zeros(length(t)+1,4);
x(1,:) = [0 0 0 0];


for i = 1:length(f_two)
    
    [mprop_upper(i),~,m_final_upper,m_init_upper(i)] = mass_vals(f_two(i)*del_v,375,.08,15000);
    [mprop_lower(i),~,m_final_lower,m_init_lower(i)] = mass_vals(f_one(i)*del_v,350,.08,m_init_upper(i));
    masses = [m_init_lower(i) mprop_lower(i) m_init_upper(i) mprop_upper(i)];
    m1_fi = mprop_lower(i);
    
end
%%
[M,I] = min(m_init_lower);
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





    



% % Prop Calcs
upper_prop = mprop_upper(I);
lower_prop = mprop_lower(I);
m_init_upper = m_init_upper(I);
m_init_lower = m_init_lower(I);

masses = [m_init_lower,lower_prop,m_init_upper,upper_prop];

rho_lox = 1141; %kg per m^3
rho_lch4 = 422.62; %kg per m^3 

stoch_ratio = 3.8; %grams of LO2 per grams of lch 4 


lower_ch4 = lower_prop/4.8;
lower_lox = stoch_ratio*lower_ch4;

upper_ch4 = upper_prop/4.8;
upper_lox = stoch_ratio*upper_ch4;

m_dot_SLR = 1700e3/(350*go); %thrust  divided by isp and gravity
m_dot_VR = 1900e3/(375*go); %Same as above, both ar Kg/s
SA = (1.8288)^2*pi; %1 inch thick walls give us a safety factor of 


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
del_t = .01;
t = 0:del_t:300;
 %first burn
    x = [0 0 422.1 0];
%     masses = [m_init_lower(I) mprop_lower(I) m_init_upper(I) mprop_upper(I)];
masses = [m_init_lower,lower_prop,m_init_upper,upper_prop];
for m = 1:length(t)
    [t(m),dx(m,:),masses(m+1,:),q_act(m),h_mag(m),~] = throttling_launch_FELO(t(m),x(m,:),masses(m,:),del_t,25);
    if m==1
        adot_x(m) = 0;
        adot_y(m) = 0;
        addot_x(m) = 0;
        addot_y(m) = 0;
    elseif m==2
        adot_x(m) = (dx(m,3)-dx((m-1),3))/del_t;
        adot_y(m) = (dx(m,4)-dx((m-1),4))/del_t;
        addot_x(m) = 0;
        addot_y(m) = 0;
    elseif m>2
        adot_x(m) = (dx(m,3)-dx((m-1),3))/del_t;
        adot_y(m) = (dx(m,4)-dx((m-1),4))/del_t;
        addot_x(m) = (adot_x(m)-adot_x(m-1))/del_t;
        addot_y(m) = (adot_y(m)-adot_y(m-1))/del_t;
    end
    
    accel_x= dx(m,3);
    accel_y= dx(m,4);
    vel_x = x(m,3)+(accel_x*del_t)+(.5*adot_x(m)*(del_t^2))+((1/6)*addot_x(m)*(del_t^3));
    vel_y = x(m,4)+(accel_y*del_t)+(.5*adot_y(m)*(del_t^2))+((1/6)*addot_y(m)*(del_t^3));
    pos_x = x(m,1)+(vel_x*del_t)+(.5*accel_x*(del_t^2))+((1/6)*adot_x(m)*(del_t^3))+((1/24)*addot_x(m)*(del_t^4));
    pos_y = x(m,2)+(vel_y*del_t)+(.5*accel_y*(del_t^2))+((1/6)*adot_y(m)*(del_t^3))+((1/24)*addot_y(m)*(del_t^4));
    x(m+1,:) = [pos_x pos_y vel_x vel_y];
    accel(m) = sqrt(accel_x^2+accel_y^2);
end
h_mag_pass = h_mag(end);
upper_coast_fuel = masses(end,4);
upper_coast = masses(end,3);
m = 1;

figure
plot(t,accel/go)
title('1st burn')
xlabel('T+ (seconds)')
ylabel('Acceleration (G)')
figure
plot(x(:,1),x(:,2))
title('Ascent Path')
xlabel('X (meters)')
ylabel('Y (meters)')

%2nd burn
vPerp = h_mag_pass/6.828;
x = [0 6828e3 vPerp 0];



masses(1,4) = upper_coast_fuel;
masses(1,3) = upper_coast;
accel = zeros(1,4);

t = 0:del_t:60;
for m = 1:length(t)
    [t(m),dx(m,:),masses(m+1,:),h_mag_end(m),~] = throttling_SECB(t(m),x(m,:),masses(m,:),del_t,60);
    if m==1
        adot_x(m) = 0;
        adot_y(m) = 0;
        addot_x(m) = 0;
        addot_y(m) = 0;
    elseif m==2
        adot_x(m) = (dx(m,3)-dx((m-1),3))/del_t;
        adot_y(m) = (dx(m,4)-dx((m-1),4))/del_t;
        addot_x(m) = 0;
        addot_y(m) = 0;
    elseif m>2
        adot_x(m) = (dx(m,3)-dx((m-1),3))/del_t;
        adot_y(m) = (dx(m,4)-dx((m-1),4))/del_t;
        addot_x(m) = (adot_x(m)-adot_x(m-1))/del_t;
        addot_y(m) = (adot_y(m)-adot_y(m-1))/del_t;
    end
    
    accel_x= dx(m,3);
    accel_y= dx(m,4);
    vel_x = x(m,3)+(accel_x*del_t)+(.5*adot_x(m)*(del_t^2))+((1/6)*addot_x(m)*(del_t^3));
    vel_y = x(m,4)+(accel_y*del_t)+(.5*adot_y(m)*(del_t^2))+((1/6)*addot_y(m)*(del_t^3));
    pos_x = x(m,1)+(vel_x*del_t)+(.5*accel_x*(del_t^2))+((1/6)*adot_x(m)*(del_t^3))+((1/24)*addot_x(m)*(del_t^4));
    pos_y = x(m,2)+(vel_y*del_t)+(.5*accel_y*(del_t^2))+((1/6)*adot_y(m)*(del_t^3))+((1/24)*addot_y(m)*(del_t^4));
    x(m+1,:) = [pos_x pos_y vel_x vel_y];
    accel(m) = sqrt(accel_x^2+accel_y^2);
end
h_mag_plot = h_mag_end(end);
figure
plot(t,accel)
title('2nd burn')




t = 0:del_t:300;
figure
plot(t,q_act)
title('Dynamic Pressure vs T+')
xlabel('T+ (seconds)')
ylabel('Q (pascals)')
[q_max, ind] = max(q_act);
fprintf('Max Q is %.2f pascals',q_max)
fprintf(' at %.2f seconds \n',t(ind))

