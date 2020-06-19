function xdot = ModelloCinematicoVeicolo(x,u,params)

    L=params.Vehicle_Length;
    x_update = [cos(x(3))*x(4);
                sin(x(3))*x(4);
                (tan(u(2))/L)*x(4);
                0.5*u(1)];

    xdot = x_update;
end