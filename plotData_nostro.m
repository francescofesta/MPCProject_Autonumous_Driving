function plotData_nostro(x0,u0,ref,Ts,info,nlobj, ostacoli, rb_mat_int, rb_mat_ext)
    ost = ostacoli;
    % Robot path
    close all;
    figure;
    hold on
    plot(info.Xopt(:,1), info.Xopt(:,2),'bo')
    plot(rb_mat_int(:,1),rb_mat_int(:,2))
    plot(rb_mat_ext(:,1),rb_mat_ext(:,2))
    
   
    grid on
    hold on
    scatter(x0(1),x0(2),'g','filled');
    scatter(ref(1),ref(2),'r','filled');
    if (length(ost.pos)>1)
       for i=1:1:size(ost.pos,1)
         rectangle('Position',[ost.pos(i,1)-ost.dim(1,1), ost.pos(i,2)-ost.dim(1,1), 2*ost.dim(1,1), 2*ost.dim(1,1)],'Curvature',[1,1],'FaceColor',[0.5,0.5,0.5]);
       end
    end
   
    
    
        
    
    % Rimuovi il commento se vuoi vedere i vettori di theta
    % quiver(info.Xopt(:,1),info.Xopt(:,2),0.1*cos(info.Xopt(:,3)),0.1*sin(info.Xopt(:,3)));
    axis equal

    xlabel('x position')
    ylabel('y position')

    % MVs
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(5,1,1);
    plot(0:Ts:nlobj.PredictionHorizon*Ts,info.MVopt(:,1));
    ylabel('v (m/s)')
    ylim([-1.5 1.5]);
    grid on;

    subplot(5,1,2);
    plot(0:Ts:nlobj.PredictionHorizon*Ts, rad2deg(info.MVopt(:,2)));
    ylabel('gamma (°)')
    grid on;
    ylim([-50 50]);

    % X monitor
    subplot(5,1,3);
    plot(0:Ts:nlobj.PredictionHorizon*Ts, info.Xopt(:,1));
    ylabel('x (m)')
    yline(ref(1),'-.r');
    grid on;

    subplot(5,1,4);
    plot(0:Ts:nlobj.PredictionHorizon*Ts, info.Xopt(:,2));
    ylabel('y (m)')
    yline(ref(2),'-.r');
    grid on;

    subplot(5,1,5);
    plot(0:Ts:nlobj.PredictionHorizon*Ts, rad2deg(info.Xopt(:,3)));
    ylabel('theta (°)')
    yline(rad2deg(ref(3)),'-.r');
    grid on;
end

