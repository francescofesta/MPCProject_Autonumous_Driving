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
%startPose=[12 48.0137366185146 0];
goalPose=[22.9,27.8,pi];
%condizioni iniziali
x0=[startPose 0];
u0=[0 0];

params=ObstaclePosition(scenario);
params.Lane_rb_mat_ext=rb_mat_ext;
params.Lane_rb_mat_int=rb_mat_int;
params.Vehicle_Length=egoVehicle.Length;
nlobj.Model.NumberOfParameters = 1;

%% Modello di predizione

nlobj.Model.StateFcn = "ModelloCinematicoVeicolo";

nlobj.Ts = Ts;
nlobj.PredictionHorizon = 83;
nlobj.ControlHorizon = 83;
%% Funzione costo

nlobj.Optimization.CustomCostFcn = @(X,U,e,data,params) Ts*sum(U(1:p,1));
nlobj.Optimization.ReplaceStandardCost = true;
nlobj.Optimization.UseSuboptimalSolution = true;

%% Vincoli anti collisione e mantenimento carreggiata
if (size(params.pos,2)>1)
    nlobj.Optimization.CustomIneqConFcn = "CollisionAvoidanceFcn";
end

%% Pesi
nlobj.Weights.OutputVariables = [15, 10, 8, 10];
nlobj.Weights.ManipulatedVariablesRate = [5, 10];

%% Validazione

validateFcns(nlobj,x0,u0,[],{params});
%Problemi con collision avoidance su troppe colonne












