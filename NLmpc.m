%scenario_finale
%% Modello del sistema
%x(1) - coordinata x
%x(2) - coordinata y
%x(3) - teta
%x(4) - steering angle

%variabili di attuazione:
%u(1) - linear speed
%u(2) - angular speed
tic
nx = 4;
ny = 4;
nu = 2;
nlobj = nlmpc(nx,ny,nu);

Ts=0.1;%scenario.SampleTime;
p=10;

%Vincoli di controllo
%Velocità:
% nlobj.ManipulatedVariables(1).RateMin = -0.2*Ts;
% nlobj.ManipulatedVariables(1).RateMax = 0.2*Ts;
%Velocità angolare:
% nlobj.ManipulatedVariables(2).RateMin = -30*Ts;
% nlobj.ManipulatedVariables(2).RateMax = 30*Ts;
%  nlobj.ManipulatedVariables(2).Min = -30;
%  nlobj.ManipulatedVariables(2).Max = 30;

% nlobj.OutputVariables(3).Min = -4;
% nlobj.OutputVariables(3).Max = 4;
%nlobj.OutputVariables(4).Min = -30;
%nlobj.OutputVariables(4).Max = 30;

startPose=scenario.Actors(1,6).Position(1,:);
%startPose=[12 48.0137366185146 0];
%goalPose=[55,30,pi,5];
%condizioni iniziali
%x=[startPose 0];
x=traiettoria_mat(1,2:5);
u=[0 0];

%  params=ObstaclePosition(scenario);
% % params.Lane_rb_mat_ext=rb_mat_ext;
% % params.Lane_rb_mat_int=rb_mat_int;
%  params.Vehicle_Length=egoVehicle.Length;
%  nlobj.Model.NumberOfParameters = 1;
%  ostacoli.pos=params.pos;
%  ostacoli.dim=params.length/2;

%% Modello di predizione

nlobj.Model.StateFcn = "ModelloCinematicoVeicolo";

nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;
%% Funzione costo

% nlobj.Optimization.CustomCostFcn = @(X,U,e,data,params) Ts*sum(U(1:p,1));
% nlobj.Optimization.ReplaceStandardCost = true;
% nlobj.Optimization.UseSuboptimalSolution = true;

%% Vincoli anti collisione e mantenimento carreggiata
% if (size(params.pos,2)>1)
%     nlobj.Optimization.CustomIneqConFcn = "CollisionAvoidanceFcn";
% end

%% Pesi
%  nlobj.Weights.OutputVariables = [5, 5, 1, 1];
%  nlobj.Weights.ManipulatedVariablesRate = [1, 5];

%% Validazione

%validateFcns(nlobj,x,u,[],{params});
validateFcns(nlobj,x,u,[]);
%Problemi con collision avoidance su troppe colonne

Duration=300;
xk=traiettoria_mat(1,2:5);
lastMV=u;

figure
xHistory = x;

for k=1:size(sim_time,1)
    % Stampa la posizione attuale
    plot(xHistory(k,1),xHistory(k,2),'rx')
    
    % Ottieni le misure dal plant.
    % Qui dovreste mettere la retroazione. Io ho aggiunto del rumore per 
    % rendere la pianificazione più reale.
    yk = xHistory(k,:)' + randn*0.01;
    xk = yk;
    
    yref=traiettoria_mat(k:min(k+9,Duration),2:5);
    
    [uk,~,info]=nlmpcmove(nlobj,xk,lastMV,yref,[]);
    % Conserva le mosse di controllo e aggiorna l'ultimo MV per il prossimo
    % step.
    uHistory(k,:) = uk';
    lastMV=uk;
    
    % Azionamento e feedback loop
    % Aggiorna lo stato del plant reale per il prossimo step risolvendo
    % l'eq differenziale basata sullo stato corrente xk e l'input uk.
    ODEFUN = @(t,xk) ModelloCinematicoVeicolo(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN, [0 Ts], xHistory(k,:)');
    odeset('RelTol', 1e-20, 'AbsTol', 1e-20);
    
    xHistory(k+1,:) = YOUT(end,:);
    
    % stampa la traiettoria pianificata
    x1_plan = info.Xopt(:,1);
    x2_plan = info.Xopt(:,2);
    plot(x1_plan,x2_plan,'g-');
    hold on
    
    % Pausa per l'animazione
     %pause(0.01);
    
   
    %

    
    
end
 hold on
    plot(rb_mat_int(:,1),rb_mat_int(:,2))
    plot(rb_mat_ext(:,1),rb_mat_ext(:,2))
    plot(traiettoria_mat(:,2),traiettoria_mat(:,3))
    
for i=1:1:size(ost_pos,1)
    rectangle('Position',[ost_pos(i,1)-ost_dim(1,1), ost_pos(i,2)-ost_dim(1,1), 2*ost_dim(1,1), 2*ost_dim(1,1)],'Curvature',[1,1],'FaceColor',[0.5,0.5,0.5]);
end

    

% for k = 1:(Duration/Ts)
%     % Set references for previewing
%     t = linspace(k*Ts, (k+p-1)*Ts,p);
%     yref = QuadrotorReferenceTrajectory(t);
%     % Compute the control moves with reference previewing.
%     xk = xHistory(k,:);
%     [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
%     uHistory(k+1,:) = uk';
%     lastMV = uk;
%     % Update states.
%     ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
%     [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
%     xHistory(k+1,:) = YOUT(end,:);
%     waitbar(k*Ts/Duration,hbar);
% end
toc




