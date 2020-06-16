clear;
clc;

%% Modello del sistema
%
% Viene rappresentato il modello di un biciclo
% Gli stati sono:
%
% x(1) - x posizione del robot
% x(2) - y posizione del robot
% x(3) - theta angolo del robot
%
% Assumo la perfetta osservabilità, quindi l'uscita y(k) = x(k).
%
% Inputs di controllo
%
% u(1) - robot v velocità
% u(2) - robot gamma angolo di sterzo

nx = 3;
ny = 3;
nu = 2;
nlobj = nlmpc(nx,ny,nu);

Ts = 0.05;   % Tempo di campionamento
p = 60;      % Orizzonte di predizione

% Limiti del controllo
uv_lb = -1.25;
uv_ub = 1.25;

ug_lb = -pi/4;
ug_ub = pi/4;

ruv_lb = -uv_ub*Ts;
ruv_ub = uv_ub*Ts;

rug_lb = -ug_ub*Ts;
rug_ub = ug_ub*Ts;


% Condizioni iniziali
x0 = [0;0;pi/4];
u0 = [0;0];
% ref
ref = [2 2 pi/4];
% Ostacolo trattato come parametro del sistema
ostacoli.pos = [1,1];
%ostacoli.pos = []; % se non commentato sto rigo => no ostacoli
ostacoli.dim = [0.1];

params = [ostacoli];
nlobj.Model.NumberOfParameters = 1;

%% Modello di predizione

nlobj.Model.StateFcn = "RobotKinematicModel";

nlobj.Ts = Ts;
nlobj.PredictionHorizon = 70;
nlobj.ControlHorizon = 70;

%% Funzione costo

% nlobj.Optimization.CustomCostFcn = "myCostFunction";
% nlobj.Optimization.ReplaceStandardCost = true;
% nlobj.Optimization.UseSuboptimalSolution = true;

%% Vincoli

% di controllo
nlobj.MV(1).Min = uv_lb;
nlobj.MV(1).Max = uv_ub;

nlobj.MV(2).Min = ug_lb;
nlobj.MV(2).Max = ug_ub;

% di variazione del controllo
% nlobj.MV(1).RateMin = ruv_lb;
% nlobj.MV(1).RateMax = ruv_ub;

% nlobj.MV(2).RateMin = rug_lb;
% nlobj.MV(2).RateMax = rug_ub;

% Vincoli anti collisione
if (length(ostacoli.pos)>1)
    nlobj.Optimization.CustomIneqConFcn = "CollisionAvoidanceConstraintFcn";
end

%% Pesi
nlobj.Weights.OutputVariables = [15, 10, 8];
%  nlobj.Weights.ManipulatedVariables = [0, 5];
%  nlobj.Weights.ManipulatedVariablesRate = [0, 10];

%% Validazione

validateFcns(nlobj,x0,u0,[],{params});

%% Pianificatore di traiettorie
% 
options = nlmpcmoveopt;
options.parameters = {params};
% Trova le prossime p mosse

tic;
[~,~,info] = nlmpcmove(nlobj,x0,u0,ref,[],options);
toc;  % tempo impiegato

% Stampa i risultati:
plotData(x0,u0,ref,Ts,info,nlobj, ostacoli)

%% Trajectory following: tutta la roba real-time
plotData(x0,u0,ref,Ts,info,nlobj, ostacoli)
% Useremo un controller non lineare per il tracking della traiettoria
nlobj_tracking = nlmpc(nx,ny,nu);
nlobj_tracking.Model.NumberOfParameters = nlobj.Model.NumberOfParameters;

% Stesso modello di prima
nlobj_tracking.Model.StateFcn = nlobj.Model.StateFcn;
% nlobj_tracking.Jacobian.StateFcn = nlobj.Jacobian.StateFcn; % da valutare
% se serve lo jacobiano

% Orizzonti di controllo e predizione brevi per una maggior velocità
% computazionale.
nlobj_tracking.Ts = Ts;
nlobj_tracking.PredictionHorizon = 10;
nlobj_tracking.ControlHorizon = 4;

% Useremo una funzione di costo standard, con un peso leggermente più alto
% sugli stati.
nlobj_tracking.Weights.ManipulatedVariablesRate = 0.2*ones(1,nu);
nlobj_tracking.Weights.OutputVariables = 5*ones(1,nx);

% Vincoli anti-collisione
if (length(ostacoli.pos)>1)
    nlobj_tracking.Optimization.CustomIneqConFcn = "CollisionAvoidanceConstraintFcn";
end
% Stessi vincoli di prima
% di controllo
nlobj_tracking.MV(1).Min = uv_lb;
nlobj_tracking.MV(1).Max = uv_ub;

nlobj_tracking.MV(2).Min = ug_lb;
nlobj_tracking.MV(2).Max = ug_ub;

% di variazione del controllo
% nlobj_tracking.MV(1).RateMin = ruv_lb;
% nlobj_tracking.MV(1).RateMax = ruv_ub;

% nlobj_tracking.MV(2).RateMin = rug_lb;
% nlobj_tracking.MV(2).RateMax = rug_ub;

% Validazione
validateFcns(nlobj_tracking,x0,u0,[],{params});

%% Finalmente, esegui la simulazione
Tsteps = 60;
xHistory = x0';
uHistory = [];
lastMV = zeros(nu,1);

% Usa la traiettoria calcolata dal planner come riferimento.
Xopt = info.Xopt;
Xref = [Xopt(2:p+1,:); repmat(Xopt(end,:),Tsteps-p,1)];

options = nlmpcmoveopt;
options.Parameters = {params};
grid on
for k = 1:Tsteps
    % Stampa la posizione attuale
    plot(xHistory(k,1),xHistory(k,2),'rx')
    % Ottieni le misure dal plant.
    % Qui dovreste mettere la retroazione. Io ho aggiunto del rumore per 
    % rendere la pianificazione più reale.
    yk = xHistory(k,:)' + randn*0.01;
    xk = yk;
    % Calcola le mosse di controllo.
    [uk,options,track_info] = nlmpcmove(nlobj_tracking,xk,lastMV,Xref(k:min(k+9,Tsteps),:),[],options);
    % Conserva le mosse di controllo e aggiorna l'ultimo MV per il prossimo
    % step.
    uHistory(k,:) = uk';
    lastMV = uk;
    % Aggiorna lo stato del plant reale per il prossimo step risolvendo
    % l'eq differenziale basata sullo stato corrente xk e l'input uk.
    ODEFUN = @(t,xk) RobotKinematicModel(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    % conserva le variabili di stato.
    xHistory(k+1,:) = YOUT(end,:);
    
    % stampa la traiettoria pianificata
    x1_plan = track_info.Xopt(:,1);
    x2_plan = track_info.Xopt(:,2);
    plot(x1_plan,x2_plan,'g-');
    
    % Pausa per l'animazione
     pause(0.01);
end

legend('Traiettoria pianificata','Partenza','Arrivo','Traiettoria attuale','pianificazione MPC','Location','southeast')

