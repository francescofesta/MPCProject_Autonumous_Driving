function ceq = PedestrianAvoidanceFcn(X,U,data,params)

p=data.PredictionHorizon;
d=sqrt(X(1,1)^2+X(1,2)^2);
treshold_obstacle=3;
speed=zeros(p,1);

for j=1:1:p
       mat_trasf(:,:,j)=[cos(X(j+1,3)) -sin(X(j+1,3)) 0 X(j+1,1);sin(X(j+1,3)) cos(X(j+1,3)) 0 X(j+1,2);0 0 1 0;0 0 0 1];
       posiz_ostacolo(:,j)=inv(mat_trasf(:,:,j))*[params.pos(6,1) params.pos(6,2) 0 1]';
       distanza_pitagora(j)=sqrt(posiz_ostacolo(1,j)^2+posiz_ostacolo(2,j)^2)-3;
       if distanza_pitagora(j)<=1.5
           speed(j)=U(j,1);
       end
end
% speed=reshape(speed,[],1);
ceq=[speed];
