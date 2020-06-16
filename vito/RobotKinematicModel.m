function xdot = RobotKinematicModel(x,u,params)
    
    L = 1;

    x_update = [u(1) * cos(x(3)) * cos(u(2));
                u(1) * sin(x(3)) * cos(u(2));
                u(1)/L * sin(u(2))];

    xdot = x_update;
end

