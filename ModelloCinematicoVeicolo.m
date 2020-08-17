function [xdot] = ModelloCinematicoVeicolo(x,u,params)

    %L=params.Vehicle_Length;
%     x_update = [cos(x(3))*x(4);
%                 sin(x(3))*x(4);
%                 (tan(u(2))/L)*x(4);
%                 0.5*u(1)];

%     x_update = [u(1) * cos(x(3)) * cos(u(2));
%                 u(1) * sin(x(3)) * cos(u(2));
%                 u(1)/L * sin(u(2))
%                 0.5*u(1)];

     L=1.5;

x_update = [u(1) * cos(x(3)) * cos(u(2));
u(1) * sin(x(3)) * cos(u(2));
u(1)/L * sin(u(2))];
xdot = x_update;
%      stato=x,y,teta ingressi=v,w


    
    
    
end