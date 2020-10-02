%% Plot variabili ottimizzate
figure('Name','Velocità Lineare Ottimizzazta')
plot(sim_time(1:end-1),uHistory(1:end-1,1))
title('Velocità Lineare Ottimizzata')
xlabel('Tempo [s]')
ylabel('Velocità Lineare [m/s]')
grid on
grid minor
print('-dsvg', '-r300','vel_opt')

figure('Name','Accelerazione Lineare Ottimizzata')
plot(sim_time(1:end-2),diff(uHistory(1:end-1,1)))
title('Accelerazione Lineare Ottimizzata')
xlabel('Tempo [s]')
ylabel('Accelerazione Lineare [m/s^{2}]')
grid on
grid minor
print('-dsvg', '-r300','acc_opt')
%%
acc_lin_opt=diff(uHistory(1:end-1,1));
for i=1:1:size(acc_lin_opt)
    if acc_lin_opt(i)<0
        acc_lin_opt(i)=0;
    end
end
index_opt= trapz(acc_lin_opt);

%% Plot variabili non ottimizzate
vel_lin_no_opt=abs(vel_lin_no_opt);

figure('Name','Velocità Lineare non Ottimizzata')
plot(sim_time(1:end-1),vel_lin_no_opt(1:end-1))
title('Velocità Lineare non Ottimizzata')
xlabel('Tempo [s]')
ylabel('Velocità Lineare [m/s]')
grid on
grid minor
print('-dsvg', '-r300','vel_no_opt')

figure('Name','Accelerazione Lineare non Ottimizzata')
plot(sim_time(1:end-2),diff(vel_lin_no_opt(1:end-1)))
title('Accelerazione Lineare non Ottimizzata')
xlabel('Tempo [s]')
ylabel('Accelerazione Lineare [m/s^{2}]')
grid on 
grid minor
print('-dsvg', '-r300','acc_no_opt')
%%
acc_lin_no_opt=diff(vel_lin_no_opt(1:end-1));
for i=1:1:size(acc_lin_no_opt)
    if acc_lin_no_opt(i)<0
        acc_lin_no_opt(i)=0;
    end
end
index_no_opt= trapz(acc_lin_no_opt);
%% plot sovrapposti
figure('Name','Confronto Velocità')
plot(sim_time(1:end-1),vel_lin_no_opt(1:end-1))
hold on
plot(sim_time(1:end-1),uHistory(1:end-1,1))
legend('velocità non ottimizzata','velocità ottimizzata')
legend('boxon')
title('Confronto velocità')
xlabel('Tempo [s]')
ylabel('Velocità Lineare [m/s]')
grid on 
grid minor
print('-dsvg', '-r300','confronto_velocita')

figure('Name','Confronto Accelerazioni')
plot(sim_time(1:end-2),diff(vel_lin_no_opt(1:end-1)))
hold on
plot(sim_time(1:end-2),diff(uHistory(1:end-1,1)))
legend('accelerazione non ottimizzata','accelerazione ottimizzata')
legend('boxon')
title('Confronto accelerazioni')
xlabel('Tempo [s]')
ylabel('Accelerazione Lineare [m/s^{2}]')
grid on 
grid minor
print('-dsvg', '-r300','confronto_accelerazioni')