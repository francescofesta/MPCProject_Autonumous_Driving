%scenario_finale
%% Modello del sistema
%x(1) - coordinata x
%x(2) - coordinata y
%x(3) - teta
%x(4) - steering angle

%variabili di attuazione:
%u(1) - linear speed
%u(2) - angular speed

nx = 4;
ny = 4;
nu = 2;
nlobj = nlmpc(nx,ny,nu);

Ts=0.01;%scenario.SampleTime;
p=10;

%Vincoli di controllo
%Throttle:
% nlobj.ManipulatedVariables(1).RateMin = -0.2*Ts;
% nlobj.ManipulatedVariables(1).RateMax = 0.2*Ts;
%Steering angle:
% nlobj.ManipulatedVariables(2).RateMin = -pi/30*Ts;
% nlobj.ManipulatedVariables(2).RateMax = pi/30*Ts;
%nlobj.ManipulatedVariables(1).Min = -10;
%nlobj.ManipulatedVariables(1).Max = 10;

startPose=scenario.Actors(1,6).Position(1,:);
%startPose=[12 48.0137366185146 0];
%goalPose=[55,30,pi,5];
%condizioni iniziali
%x=[startPose 0];
x=traiettoria_mat(1,2:5);
u=[0 0];

% params=ObstaclePosition(scenario);
% params.Lane_rb_mat_ext=rb_mat_ext;
% params.Lane_rb_mat_int=rb_mat_int;
% params.Vehicle_Length=egoVehicle.Length;
% nlobj.Model.NumberOfParameters = 1;
% ostacoli.pos=params.pos;
% ostacoli.dim=params.length/2;

%% Modello di predizione

nlobj.Model.StateFcn = "ModelloCinematicoVeicolo";

nlobj.Ts = Ts;
nlobj.PredictionHorizon = 20;
nlobj.ControlHorizon = 2;
%% Funzione costo

% nlobj.Optimization.CustomCostFcn = @(X,U,e,data,params) Ts*sum(U(1:p,1));
% nlobj.Optimization.ReplaceStandardCost = true;
nlobj.Optimization.UseSuboptimalSolution = true;

%% Vincoli anti collisione e mantenimento carreggiata
%if (size(params.pos,2)>1)
    %nlobj.Optimization.CustomIneqConFcn = "CollisionAvoidanceFcn";
%end

%% Pesi
% nlobj.Weights.OutputVariables = [10, 10, 2, 2];
% nlobj.Weights.ManipulatedVariablesRate = [10, 5];

%% Validazione

validateFcns(nlobj,x,u,[]);
%Problemi con collision avoidance su troppe colonne

Duration=10;
xk=traiettoria_mat(1,2:5);
lastMV=u;
figure
for k=1:size(sim_time)
    yref=traiettoria_mat(k,2:5);
    [uk,~,info(k)]=nlmpcmove(nlobj,xk,lastMV,yref,[]);
    xk=info(k).Xopt(2,:);
    lastMV=uk;
   
    hold on
    plot(info(k).Xopt(1,1), info(k).Xopt(1,2),'bo')

    
    
end
    plot(rb_mat_int(:,1),rb_mat_int(:,2))
    plot(rb_mat_ext(:,1),rb_mat_ext(:,2))
    plot(traiettoria_mat(:,2),traiettoria_mat(:,3))

    

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





