clear all
clc
%% 
scenario_finale
%% Modello del sistema
%x(1) - coordinata x
%x(2) - coordinata y
%x(3) - coordinata teta
%x(4) - vehicle speed

%variabili di attuazione:
%u(1) - throttle (accelerazione)
%u(2) - steering angle

nx = 4;
ny = 4;
nu = 2;
nlobj = nlmpc(nx,ny,nu);

Ts=scenario.SampleTime;
p=60;

%Vincoli di controllo
%Throttle:
nlobj.ManipulatedVariables(1).RateMin = -0.2*Ts;
nlobj.ManipulatedVariables(1).RateMax = 0.2*Ts;
%Steering angle:
nlobj.ManipulatedVariables(2).RateMin = -pi/30*Ts;
nlobj.ManipulatedVariables(2).RateMax = pi/30*Ts;

startPose=scenario.Actors(1,6).Position(1,:);
goalPose=[22.9,27.8,pi];

params=ObstaclePosition(scenario);







