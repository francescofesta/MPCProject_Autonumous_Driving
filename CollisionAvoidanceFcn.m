function cineq = CollisionAvoidanceFcn(X,U,e,data,params)
 
  
% rb_mat_ext=params.Lane_rb_mat_ext;
% rb_mat_int=params.Lane_rb_mat_int;
p=data.PredictionHorizon;
% k=1;
% j=1;
% treshold_lane=0.2+(params.Vehicle_Length/2)*cos(X(2:p+1,3));
% for i=1:1:size(rb_mat_ext,1)
%     dist_ext=vecnorm(X(1,1:2)-rb_mat_ext(i,1:2));
%     dist_int=vecnorm(X(1,1:2)-rb_mat_int(i,1:2));
%     if dist_ext<10
%         rb_ext_new(k,:)=rb_mat_ext(i,1:2);
%         k=k+1;
%     end
%     if dist_int<10
%         rb_int_new(j,:)=rb_mat_int(i,1:2);
%         j=j+1;        
%     end   
% end
% vincolo_est=ones(10,1);
% if k>1
%  for i=1:1:size(rb_ext_new,1)
%     diff_ext(:,i)=vecnorm(X(2:p+1,1:2)-rb_ext_new(i,:),2,2);
%     vincolo_est(:,i)=diff_ext(:,i)-treshold_lane;
%  end
%  vincolo_est=reshape(vincolo_est,[],1);
% end
% vincolo_int=ones(10,1);
% if j>1
%  for i=1:1:size(rb_int_new,1)
%     diff_int(:,i)=vecnorm(X(2:p+1,1:2)-rb_int_new(i,:),2,2);
%     vincolo_int(:,i)=diff_int(:,i)-treshold_lane;
%  end
%  vincolo_int=reshape(vincolo_int,[],1);
% end

    

% treshold_obstacle=(params.length(1,1)/2)+((4.7)/2)*cos(X(2:p+1,3)); % inserire lunghezza veicolo da passare
treshold_obstacle=6.5;
    for i=1:1:(size(params.pos,1))
        argomento_vecnorm=[X(2:p+1,1)-params.pos(i,1)  X(2:p+1,2)-params.pos(i,2)];
        out_vec(i,:)=vecnorm(argomento_vecnorm);
        dist_obstacle(i)=out_vec(i,1)+out_vec(i,2);
        
        vincolo_ost(i)=dist_obstacle(i)-treshold_obstacle;
    end   
%  vincolo_ost=reshape(vincolo_ost,[],1);
 for i=1:1:5
    cineq= [-vincolo_ost(i)];
 end 
%             
%      cineq=[
%          -(vincolo_ost);
%             ];
%     r_safe = 2*params.Obstacle.length(1);
%    
%
%     X1 = X(2:p+1,1:2);
%     X2 = X(2:p+1,2:3);
%    
%     dist = vecnorm(X1' - X2');
%    
%     Lanenorm_ext=vecnorm((rb_mat_ext(2:p+1,1))'-(rb_mat_ext(2:p+1,2))');
%     Lanenorm_int=vecnorm((rb_mat_int(2:p+1,1))'-(rb_mat_int(2:p+1,2))');
%
%     cineq = [-(dist - r_safe);
%              dist-Lanenorm_ext;
%              Lanenorm_int-dist];
end