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

Ts=1;%scenario.SampleTime;
p=10;

%Vincoli di controllo
%Throttle:
% nlobj.ManipulatedVariables(1).RateMin = -0.2*Ts;
% nlobj.ManipulatedVariables(1).RateMax = 0.2*Ts;
%Steering angle:
% nlobj.ManipulatedVariables(2).RateMin = -pi/30*Ts;
% nlobj.ManipulatedVariables(2).RateMax = pi/30*Ts;
nlobj.ManipulatedVariables(1).Min = -10;
nlobj.ManipulatedVariables(1).Max = 10;

startPose=scenario.Actors(1,6).Position(1,:);
%startPose=[12 48.0137366185146 0];
goalPose=[40,48,pi,5];
%condizioni iniziali
x0=[startPose 0];
u0=[0 0];

params=ObstaclePosition(scenario);
params.Lane_rb_mat_ext=rb_mat_ext;
params.Lane_rb_mat_int=rb_mat_int;
params.Vehicle_Length=egoVehicle.Length;
nlobj.Model.NumberOfParameters = 1;
ostacoli.pos=params.pos;
ostacoli.dim=params.length/2;

%% Modello di predizione

nlobj.Model.StateFcn = "ModelloCinematicoVeicolo";

nlobj.Ts = Ts;
nlobj.PredictionHorizon = 15;
nlobj.ControlHorizon = 15;
%% Funzione costo

% nlobj.Optimization.CustomCostFcn = @(X,U,e,data,params) Ts*sum(U(1:p,1));
% nlobj.Optimization.ReplaceStandardCost = true;
nlobj.Optimization.UseSuboptimalSolution = true;

%% Vincoli anti collisione e mantenimento carreggiata
if (size(params.pos,2)>1)
    nlobj.Optimization.CustomIneqConFcn = "CollisionAvoidanceFcn";
end

%% Pesi
% nlobj.Weights.OutputVariables = [10, 10, 2, 2];
% nlobj.Weights.ManipulatedVariablesRate = [10, 5];

%% Validazione

validateFcns(nlobj,x0,u0,[],{params});
%Problemi con collision avoidance su troppe colonne

%% Pianificatore di traiettorie
% 
options = nlmpcmoveopt;
options.parameters = {params};
% Trova le prossime p mosse

tic;
[~,~,info] = nlmpcmove(nlobj,x0,u0,goalPose,[],options);
toc;  % tempo impiegato

% Stampa i risultati:

plotData_nostro(x0,u0,goalPose,Ts,info,nlobj, ostacoli, rb_mat_int, rb_mat_ext)











