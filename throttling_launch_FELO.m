function [t,dx,masses,q,h_mag,var_pass] = throttling_launch_FELO(t,x,masses,del_t,hyp_perc)

m1 = masses(1);
m1_fuel = masses(2);
m2 = masses(3);
m2_fuel = masses(4);

[rho,~,a,~] = std_atmosphere(x(2));
g = (6.6742e-11)*(5973600000000000000000000/((6371000+x(2))^2));
SA = (1.8288)^2*pi; %m^2 12 ft OD
cd = .075;
V = sqrt(((x(3)-422.1)^2)+(x(4)^2));
q = .5*rho*V^2;
MACH = V/a;
%Ascent Path Guidance
turn_i = 8.5e3;%initiate turn
turn_alt = 112e3; %alt of 0 vertical speed
hyp_val = (hyp_perc/100)*10;
t_par = x(4)/g;
d_peak = x(2)+(x(4)*t_par)-(.5*g*t_par^2);
turn_ang_final = (pi/2)-deg2rad(0);


if x(2) < turn_i
    fpa = 1; %go straight up
    var_pass = 0;
else %grav turn profile
    var_pass = (tanh(hyp_val-(hyp_val*(((turn_alt+turn_i)-d_peak)/turn_alt))))*turn_ang_final; %final alt 
    fpa = cos(var_pass);
end

%Basic Orb calcs to see if we need to coast
if x(2) > 5e3
    r = [x(1)/1000 ((x(2)/1000)+6378) 0];
    v = [x(3)/1000 x(4)/1000 0];
    [semi_major,~,h_mag, ~, ~,~, e_mag, ~,~,~,~,~,~,~,~,~, ~,~,~] = orbital_param(r,v);
    
    peri = semi_major*(1-e_mag);
    appo = semi_major*(1+e_mag);
else
    appo = 1;
    h_mag = 1;
end

if x(2) < 80e3
    thrust_co = (((200e3)*(x(2)/80e3))+1700e3)/1700e3;
else
    thrust_co = (1900/1700);
end

%Actual Math Part
if appo < 6828;
    if m1_fuel > 10000 %1st Stage
        if q < 28e3 ;
            thrust = 1700e3*4*thrust_co;
            accel_rocket = (thrust-(q*SA*cd))/m1;
            if accel_rocket+(fpa*g) > 39.24
                accel_rocket = 4*9.81-(fpa*g);
                thrust = (m1*accel_rocket)+(fpa*g);
                fuel_loss = (thrust/(1700e3*4*thrust_co))*(495.3*4)*del_t;
            else
                fuel_loss = 495.3*4*del_t;
            end
            
        else
            thrust = (q*SA*cd)+(g*m1*fpa);
            accel_rocket = 0;
            fuel_loss = (thrust/(1700e3*4*thrust_co))*(495.3*4)*del_t;
        end
        m1 = m1 - fuel_loss;
        m1_fuel = m1_fuel - fuel_loss;
        masses(1) = m1;
        masses(2) = m1_fuel;
        thrust_x = sin(var_pass)*thrust;
        thrust_y = fpa*thrust;
        accel_x = thrust_x/m1;
        accel_y = thrust_y/m1;
        
        %1st stage accel calcs
        r = x(2) + 6378e3; %orbital radius from center of earth
        v_perp = x(3);
        a_cent = (v_perp^2)/r; %positive (up)
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3) = accel_x;
        dx(4) = accel_y+a_cent-g;
        
        
    elseif m1_fuel <10000%2nd Stage
        if m2_fuel > 3500
            thrust = 1900e3;
            accel_rocket = (thrust - (q*SA*cd))/m2;
            if (accel_rocket/9.81) + (fpa*g) > 4
                accel_rocket = 4*9.81-(fpa*g);
                thrust = (m2*accel_rocket)+(fpa*g);
                fuel_loss = (thrust/(1900e3))*(516.7)*del_t;
            else
                fuel_loss = 516.6*del_t;
            end
        else
            fuel_loss = 0;
            thrust = 0;
            accel_rocket = 0;
        end
        m2 = m2 - fuel_loss;
        m2_fuel = m2_fuel - fuel_loss;
        masses(3) = m2;
        masses(4) = m2_fuel;
        thrust_x = sin(var_pass)*thrust;
        thrust_y = fpa*thrust;
        accel_x = thrust_x/m2;
        accel_y = thrust_y/m2;
        
        %1st stage accel calcs
        if m2_fuel > 3500
            r = x(2) + 6378e3; %orbital radius from center of earth
            v_perp = x(3);
            a_cent = (v_perp^2)/r; %positive (up)
            dx(1) = x(3);
            dx(2) = x(4);
            dx(3) = accel_x;
            dx(4) = accel_y+a_cent-g;
        else
            dx(1) = x(3);
            dx(2) = x(4);
            dx(3) = 0;
            dx(4) = 0;
        end
    else
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3) = 0;
        dx(4) = 0;
    end
else
    dx(1) = x(3);
    dx(2) = x(4);
    dx(3) = 0;
    dx(4) = 0;
end