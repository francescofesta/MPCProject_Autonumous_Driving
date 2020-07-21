function [A,Bmv]=myJacobian(x,u,params)


    L=params.Vehicle_Length;
    A=zeros(4,4);
    Bmv=zeros(4,2);
    
%     A=[0 0 cos(x(3))*u(1) 0;
%        0 0 sin(x(3))*u(1) 0;
%        0 0 0 tan(x(4))*(u(1)/L);
%                0 0 0 0];
%    B=[0 0;
%        0 0;
%        0 0;
%        0 1];
% 
%     
    A(1,3)=-sin(x(3))*u(1);
    A(2,3)=cos(x(3))*u(1);
    A(3,4)=(u(1)/L)/((cos(x(4))+eps)^2);
    
    Bmv(1,1)=cos(x(3));
    Bmv(2,1)=sin(x(3));
    Bmv(3,1)=sin(x(4))/((cos(x(4))+eps)*L);
    Bmv(4,2)=1;
    
    
    
    
    
end