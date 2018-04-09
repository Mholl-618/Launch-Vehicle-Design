function dx = rocket_man(t,x,m,tc)
    dx = zeros(2,1);
    %Atmo stuff and basic conditions
    [rho,~,~,a,~] = std_atmosphere(x(1));
        g = (6.6742e-11)*(5973600000000000000000000/((6371000+x(1))^2));
        SA = (2.5908)^2*pi; %m^2 17 ft OD
        cd = .075;
        V = x(2);
        q = .5*rho*V^2;
        MACH = V/a;

        
        %Actual Math Part 
        if t<tc(1) %1st Stage
            mass = m(1) - (495.2907320178509*7*t);
            m_prop_lower = m(2) - (495.2907320178509*7*t);
            if m_prop_lower > .02*m(2)
                Thrust = 1700e3*7; %7 raptors in KN
                Drag = cd*q*SA;
                accel_thrust = Thrust/mass;
                accel_drag = Drag/mass;
                
                dx(1) = x(2);
                dx(2) = (accel_thrust - accel_drag)-g;
                
            elseif t<tc(1)
                dx(1) = x(2);
                dx(2) = 0;
            else
            end
        else %2nd Stage
            mass = m(3) - (516.6562*(t-tc(1)));
            m_prop_upper = m(4) - (516.6562*(t-tc(1)));
            Drag = cd*q*SA;
            if m_prop_upper > 0.03*m(4)
                Thrust = 1900e3; %Vacuum raptors in KN
                accel_thrust = Thrust/mass;
                accel_drag = Drag/mass;
                
                dx(1) = x(2);
                dx(2) = accel_thrust-accel_drag; %assume no drag
            else
                dx(1) = x(2);
                dx(2) = 0;
            end
        end
    
end