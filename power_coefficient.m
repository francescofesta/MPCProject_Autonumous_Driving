%% Calcolo potenza e potenza media
m=1500;
v=uHistory(1:end-1,1);
a=diff(uHistory(1:end-1,1));
% a=[a(1);a];
%v()=[];
v_av=mean(v);
a_av=mean(a);
p=m*v_av*a_av;
%%
acc_no_opt=diff(vel_lin_no_opt);
v_no_av=mean(vel_lin_no_opt);
a_no_av=mean(acc_no_opt);
p_no=m*v_no_av*a_no_av;
% p=m*(a.*v);
% 
% for i=1:1:size(p)
%     if p(i)<0
%         p(i)=0;
%     end
% end
% index_p=mean(p);