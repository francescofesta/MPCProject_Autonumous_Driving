function cineq = ObstacleAvoidanceFcn(X,U,e,data,params)
 
p=data.PredictionHorizon;
d=sqrt(X(1,1)^2+X(1,2)^2);
treshold_obstacle=3;
% ascissa=zeros(15,10);
  for i=1:1:(size(params.pos,1))
        
%         argomento_vecnorm=[X(2:p+1,1)-params.pos(i,1)  X(2:p+1,2)-params.pos(i,2)];
%         out_vec(i,:)=vecnorm(argomento_vecnorm');
% %       dist_obstacle(i)=out_vec(i,1)+out_vec(i,2);
%         vincolo_ost(i)=out_vec(i)-treshold_obstacle;
        for j=1:1:p
             mat_trasf(:,:,j)=[cos(X(j+1,3)) -sin(X(j+1,3)) 0 X(j+1,1);sin(X(j+1,3)) cos(X(j+1,3)) 0 X(j+1,2);0 0 1 0;0 0 0 1];

%       mat_trasf=[cos(X(1,3)) -sin(X(1,3)) 0 d*cos(X(1,3));sin(X(1,3)) cos(X(1,3)) 0 d*sin(X(1,3));0 0 1 0;0 0 0 1];
%         mat_trasf=[cos(X(1,3)) -sin(X(1,3)) 0 X(1,1);sin(X(1,3)) cos(X(1,3)) 0 X(1,2);0 0 1 0;0 0 0 1];
            posiz_ostacolo(:,j)=inv(mat_trasf(:,:,j))*[params.pos(i,1) params.pos(i,2) 0 1]';
            distanza_pitagora(i,j)=sqrt(posiz_ostacolo(1,j)^2+posiz_ostacolo(2,j)^2)-3;
%             if distanza_pitagora(i,j)<2 && i>5
%                 ascissa(i,j)=posiz_ostacolo(1,j);
%             end
%         dist_ost=vecnorm([X(1,1)-params.pos(i,1)  X(1,2)-params.pos(i,2)]);
        end
%         if posiz_ostacolo(2,1)<0
%             vincolo_carr=posiz_ostacolo(2,:);
%         
%         if posiz_ostacolo(1)>0 && dist_ost<5 
%             if params.pos(i,1)>X(1,1) && params.pos(i,2)<X(1,2)
%                 vincolo_carr(:,i)=-(X(2:p+1,2)-params.pos(i,2));
%                 disp('sono entrato nell if 1');
%             end          
%             if params.pos(i,1)>X(1,1) && params.pos(i,2)>X(1,2)
%                 vincolo_carr(:,i)=X(2:p+1,1)-params.pos(i,1);
%                 disp('sono entrato nell if 2');
%             end
%             if params.pos(i,1)<X(1,1) && params.pos(i,2)>X(1,2)
%                 vincolo_carr(:,i)=X(2:p+1,2)-params.pos(i,2);
%                 disp('sono entrato nell if 3');
%             end
%             if params.pos(i,1)<X(1,1) && params.pos(i,2)<X(1,2)
%                 vincolo_carr(:,i)=-(X(2:p+1,1)-params.pos(i,1));
%                 disp('sono entrato nell if 4');
%             end       
%         else vincolo_carr=zeros(p,1);
%         end
    
   end   
    
%   vincolo_carr=reshape(vincolo_carr,[],1);
distanza_pitagora=reshape(distanza_pitagora,[],1);
% ascissa=reshape(ascissa,[],1);
%   cineq= [-distanza_pitagora; ascissa];
  cineq= [-distanza_pitagora];%<=

end