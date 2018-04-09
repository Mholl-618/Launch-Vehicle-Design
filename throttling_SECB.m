function [t,dx,masses,q,h_mag] = throttling_SECB(t,x,masses,del_t,hyp_perc)

fpa = 0;
var_pass = pi/2;
m2 = masses(3);
m2_fuel = masses(4);

    r = [x(1)/1000 ((x(2)/1000)+6378) 0];
    v = [x(3)/1000 x(4)/1000 0];
    [~,~,h_mag, ~, ~,~, ~, ~,~,~,~,~,~,~,~,~, ~,~,~] = orbital_param(r,v);
    

[rho,~,~,a,~] = std_atmosphere(x(2));
g = (6.6742e-11)*(5973600000000000000000000/((6371000+x(2))^2));
SA = (3.6576/2)^2*pi; %m^2 12 ft OD
cd = .075;
V = sqrt((x(3)^2)+(x(4)^2));
q = .5*rho*V^2;
MACH = V/a;
%Ascent Path Guidance

hyp_val = (hyp_perc/100)*5;


%Basic Orb calcs to see if we need to coast



%Actual Math Part

%2nd Stage
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

if m2_fuel > 3500
    r = x(2) + 6378e3; %orbital radius from center of earth
    v_perp = x(3);
    a_cent = (v_perp^2)/r; %positive (up)
    dx(1) = x(3);
    dx(2) = x(4);
    dx(3) = accel_x;
    dx(4) = accel_y+a_cent-(fpa*g);
else
    dx(1) = x(3);
    dx(2) = x(4);
    dx(3) = 0;
    dx(4) = 0;
end


  
end