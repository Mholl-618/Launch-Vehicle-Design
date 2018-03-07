function dx = rocket_man(t,x)
    dx = zeros(2,1);
    %Atmo stuff and basic conditions
    [rho,~,~,a,~] = std_atmosphere(x(1));
        g = 9.81;
        SA = (3.6576/2)^2*pi; %m^2 12 ft OD
        cd = .075;
        V = x(2);
        q = .5*rho*V^2;
        MACH = V/a;

        
        %Actual Math Part 
        if t<185 %1st Stage
            mass = (5.826244781795430e+05) - (495.2907320178509*5*t);
            m_prop_lower = (447604.9970389301 - (495.2907320178509*5*t));
            if m_prop_lower > 0
                Thrust = 1700e3*5; %Five raptors in KN
                Drag = cd*q*SA;
                accel_thrust = Thrust/mass;
                accel_drag = Drag/mass;
                
                dx(1) = x(2);
                dx(2) = (accel_thrust - accel_drag)-g;
            elseif t<185
                dx(1) = x(2);
                dx(2) = 0;
            else
            end
        else %2nd Stage
            mass = 85285.59258073178 - (516.6562*(t-185));
            m_prop_upper = 63257.03332265860 - (516.6562*(t-185));
            if m_prop_upper > 0 
                Thrust = 1900e3; %Vacuum raptors in KN
                accel_thrust = Thrust/mass;
                
                dx(1) = x(2);
                dx(2) = accel_thrust; %assume no drag
            else
                dx(1) = x(2);
                dx(2) = 0;
            end
        end
    
end