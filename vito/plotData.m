function plotData(x0,u0,ref,Ts,info,nlobj, ostacoli)
    ost = ostacoli;
    % Robot path
    close all;
    figure;
    plot(info.Xopt(:,1), info.Xopt(:,2),'bo')
    grid on
    hold on
    scatter(x0(1),x0(2),'g','filled');
    scatter(ref(1),ref(2),'r','filled');
    if (length(ost.pos)>1)
        rectangle('Position',[ost.pos(1)-ost.dim, ost.pos(2)-ost.dim, 2*ost.dim, 2*ost.dim],'Curvature',[1,1],'FaceColor',[0.5,0.5,0.5]);
    end
    % Rimuovi il commento se vuoi vedere i vettori di theta
    % quiver(info.Xopt(:,1),info.Xopt(:,2),0.1*cos(info.Xopt(:,3)),0.1*sin(info.Xopt(:,3)));
    axis equal

    xlabel('x position')
    ylabel('y position')

    % MVs
%     figure('units','normalized','outerposition',[0 0 1 1])
%     subplot(5,1,1);
%     plot(0:Ts:nlobj.PredictionHorizon*Ts,info.MVopt(:,1));
%     ylabel('v (m/s)')
%     ylim([-1.5 1.5]);
%     grid on;
% 
%     subplot(5,1,2);
%     plot(0:Ts:nlobj.PredictionHorizon*Ts, rad2deg(info.MVopt(:,2)));
%     ylabel('gamma (°)')
%     grid on;
%     ylim([-50 50]);
% 
%     % X monitor
%     subplot(5,1,3);
%     plot(0:Ts:nlobj.PredictionHorizon*Ts, info.Xopt(:,1));
%     ylabel('x (m)')
%     yline(ref(1),'-.r');
%     grid on;
% 
%     subplot(5,1,4);
%     plot(0:Ts:nlobj.PredictionHorizon*Ts, info.Xopt(:,2));
%     ylabel('y (m)')
%     yline(ref(2),'-.r');
%     grid on;
% 
%     subplot(5,1,5);
%     plot(0:Ts:nlobj.PredictionHorizon*Ts, rad2deg(info.Xopt(:,3)));
%     ylabel('theta (°)')
%     yline(rad2deg(ref(3)),'-.r');
%     grid on;
end

