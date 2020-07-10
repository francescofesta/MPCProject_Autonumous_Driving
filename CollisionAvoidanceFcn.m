function cineq = CollisionAvoidanceFcn(X,U,e,data,params)
     p = data.PredictionHorizon;

     treshold_obstacle=(params.length(1,1)/2)+(params.Vehicle_Length/2)*cos(X(2:p+1,3));
      for i=1:1:size(params.pos,1)
        dist_obstacle(:,i)=vecnorm(X(2:p+1,1:2)-params.pos(i,:),2,2);
        vincolo_ost(:,i)=dist_obstacle(:,i)-treshold_obstacle;
      end
     
      vincolo_ost=reshape(vincolo_ost,[],1);
           cineq=-(vincolo_ost);
            
%      cineq=[
%          -(vincolo_int);
%          -(vincolo_est);
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