V = 20;
x0 = [0; 0; 0; V]; 
u0 = [0; 0];

%%
% Discretize the continuous-time model using the zero-order holder method
% in the |obstacleVehicleModelDT| function.
Ts = 0.01;
[Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x0,u0);
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
dsys.InputName = {'Throttle','Delta'};
dsys.StateName = {'X','Y','Theta','V'};
dsys.OutputName = dsys.StateName;