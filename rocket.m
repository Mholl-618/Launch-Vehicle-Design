function dx = rocket(t,x)
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
        mass = (667910.0707602748) - (495.2907320178509*5*t);
        m_prop_lower = (447604.9970389301 - (495.2907320178509*5*t));
        if m_prop_lower > 0
        Thrust = 1700e3*5; %Five raptors in KN
        Drag = cd*q*SA;
        accel_thrust = Thrust/mass;
        accel_drag = Drag/mass;
        
        dx(1) = x(2);
        dx(2) = (accel_thrust - accel_drag)-g;
        else 
        dx(1) = x(2);
        dx(2) = 0;
        end
    
end