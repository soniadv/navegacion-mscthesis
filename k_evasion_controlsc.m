 function [gammavec,velvec,edos]=k_evasion_controlsc(xin,xfin,angfin,punto)

global v gammav L dt cont d alpha beta
L=0.5; dt=0.1; cont=0;
tact=0; tvec=tact;
krho=1; kalpha=3; kbeta=-2;
kr2=krho/10; ka2=kalpha/10; kb2=kbeta/10;
edos=xin;
xactual=xin;
d=1;

%%
%punto=[-6 -1];
%d1=0.5; d2=0.8; kdv=1.5;kdg=1;
%d1=0.5; d2=1.3; kdv=1.15; kdg=1.1; ultimos valores oct14
%d1=0.4; d2=1.5; kdv=1.25; kdg=0.3; oct 23, cambiando radio test
d1=0.4; d2=1.5; kdv=1.25; kdg=0.5; 
%% Parametros filtro
q=0.001; Q=q*q*eye(3); %process noise covmat
r=0.01; R=r*r;            %measurement noise var

P=r*eye(3);
edo_pre=zeros(3,1);
salida(1)=xin(1)+r*randn;
pre_X(1)=xin(1); pre_Y(1)=xin(2); pre_T(1)=xin(3);

%% Ciclo
while d>=0.1
cont=cont+1;
%% Cambio de variable
dif=xactual-xfin;
[d, alpha, beta, direction]=varchange(dif);
beta=beta+angfin;

%% Control
[t2,ind] = rks4(@sistd,tact,tact+dt,0,1);
v=(krho*d+kr2*ind(2))*direction; %velocidad

if v>1;  v=1; end
if v<-1; v=-1;end
cur=xactual(1:2);
n=norm(cur-punto);
 if n<d2                             %evasion
     div=(d2-n)/(d2-d1);
     f=1-tanh(kdv*div);
%       if n<d1
%         f=0;
%     end
     v=f*v;
 else
     v=v;
 end
velvec(cont)=v;

[t3,ina] = rks4(@sista,tact,tact+dt,0,1);
[t4,inb] = rks4(@sistb,tact,tact+dt,0,1);

gammav=(kalpha*alpha+ka2*ina(2))+(kbeta*beta+kb2*inb(2)); %gamma
gammav=atan(gammav*direction/abs(v));
% if gammav>0.4; gammav=0.4; end
% if gammav<-0.4; gammav=-0.4;end
if n<d2
    dife=xactual-[punto 0];
    [d22, alpha2, beta2, direction2]=varchange(dife);
    div=(d2-n)/(d2-d1);
   if alpha2>0
      f=1+tanh(kdg*div);  %- para mover a izquierda
      gammav=gammav-f;  
   else
       f=1+tanh(kdg*div);  %+ para mover a derecha
      gammav=gammav+f;   
   end
else
    gammav=gammav;
end

if gammav>0.5; gammav=0.5; end
if gammav<-0.5; gammav=-0.5; end
gammavec(cont)=gammav; 
%% Ec de movimiento
x0=xactual';
[t,edo1] = rks4(@sist,tact,tact+dt,x0,1);

tact=tact+dt;
tvec=cat(1,tvec,tact);
edos=cat(1,edos,edo1(2,:));
%xactual=edo1(2,:);
xactualclean=edo1(2,:);
%% Agregar ruido
F=[1 0 -dt*v*sin(pre_T(cont)); 0 1 dt*v*cos(pre_T(cont));0 0 1];
H=[1 0 0];

edosnoise(cont+1,:)=xactualclean+q*randn(1,3);
salida(cont+1)=H*edosnoise(cont+1,:)'+r*randn;

%% Prediccion


mean_pre(1,cont)=dt*(v*cos(pre_T(cont)))+pre_X(cont);
mean_pre(2,cont)=dt*(v*sin(pre_T(cont)))+pre_Y(cont);
mean_pre(3,cont)=dt*(v*tan(gammav)/L)+pre_T(cont);
% mean_pre{k+1}= mean_pre{k+1}';
Y_pre(cont+1)=H*mean_pre(:,cont);
P=F*P*F'+Q;

%% Actualizacion
S=H*P*H'+R;
K(:,cont)=P*H'./S; %Kalman gain

%error(k+1)=x(k+1,1)-Y_pre(k+1);
in(cont+1)=(salida(cont+1)-Y_pre(cont+1));
mean_c(:,cont)=mean_pre(:,cont)+(K(:,cont)*in(cont+1));
pre_X(cont+1)=mean_c(1,cont); pre_Y(cont+1)=mean_c(2,cont); pre_T(cont+1)=mean_c(3,cont);

P_up=(eye(3)-K(:,cont)*H)*P; %updating covariance mat
P=P_up; %overwriting cov mat
xactual=mean_c(:,cont)';

end



figure(2)
plot(edos(:,1),edos(:,2),'k','LineWidth',1)
hold on
plot(punto(1),punto(2),'bo')
title('Trayectoria')
xlabel('X','fontsize',10);ylabel('Y','fontsize',14);
axis equal


function dx = sist(~,x)
%global v gammav L 
dx(1)=v*cos(x(3));
dx(2)=v*sin(x(3));
dx(3)=v*tan(gammav)/L;
dx=dx';
end

function dx = sistd(~,x)
%global d
dx(1)=d;
end

function dx = sista(~,x)
%global alpha
dx(1)=alpha;
end

function dx = sistb(~,x)
%global beta
dx(1)=beta;
end

function [d, alpha, beta, direction] = varchange(u)
%entra diferecia entre posicion actual y deseada, y orientacion actual 
x=u(1); y=u(2); theta=u(3); 
d=sqrt(x^2+y^2); %transformar en distancia
alpha=atan2(-y,-x)-theta;
beta=-theta-alpha;
if (alpha>=pi/2)||(alpha<= -pi/2) %si el angulo al que hay que dirigirse esta atras
 direction=-1; %move backwards
else
 direction=1;
end
if direction== -1
 alpha=-atan2(y,x)-theta;
end  
if alpha>=pi/2
    alpha=pi/2;
end
if alpha<=-pi/2
    alpha=-pi/2;
end
end

end