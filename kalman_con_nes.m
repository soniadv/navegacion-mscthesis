function [gammavec,velvec,edos,salida]=kalman_con_nes(xin,xfin,angfin)

global v gammav L dt contnes dnes alpha beta P
L=0.5; dt=0.1; contnes=0;
tact=0; tvec=tact;
krho=1; kalpha=3; kbeta=-2;
kr2=krho/10; ka2=kalpha/10; kb2=kbeta/10;
edos=xin;
xactual=xin;
dnes=1;

%% Parametros filtro
q=0.009; Q=q*q*eye(3); %process noise covmat
r=0.01; R=r*r;            %measurement noise var

%P=eye(3);
P=[6.1803854e-07,-9.7732446e-08,-2.7259084e-08;-9.7732453e-08,1.0028999,0.00058186345;-2.7259084e-08,0.00058186345,0.00016420466];
edo_pre=zeros(3,1);
salida(1)=xin(1)+r*randn;
pre_X(1)=xin(1); pre_Y(1)=xin(2); pre_T(1)=xin(3);



while dnes>=0.1
contnes=contnes+1;
%% Cambio de variable
dif=xactual-xfin;
[dnes, alpha, beta, direction]=varchange(dif);
beta=beta+angfin;

%% Control
[t2,ind] = rks4(@sistd,tact,tact+dt,0,1);
v=(krho*dnes+kr2*ind(2))*direction; %velocidad

if v>1;  v=1; end
if v<-1; v=-1;end
velvec(contnes)=v;

[t3,ina] = rks4(@sista,tact,tact+dt,0,1);
[t4,inb] = rks4(@sistb,tact,tact+dt,0,1);

gammav=(kalpha*alpha+ka2*ina(2))+(kbeta*beta+kb2*inb(2)); %gamma
gammav=atan(gammav*direction/abs(v));
if gammav>0.4; gammav=0.4; end
if gammav<-0.4; gammav=-0.4;end
gammavec(contnes)=gammav;  

%% Calcular x, y, theta
x0=xactual';
[t,edo1] = rks4(@sist,tact,tact+dt,x0,1);

tact=tact+dt;
tvec=cat(1,tvec,tact);
edos=cat(1,edos,edo1(2,:));
%xactual=edo1(2,:);
xactualclean=edo1(2,:);
%% Agregar ruido
  F=[1 0 -dt*v*sin(pre_T(contnes)); 0 1 dt*v*cos(pre_T(contnes));0 0 1]; 
    H=[1 0 0];
    
edosnoise(contnes+1,:)=xactualclean+q*randn(1,3);
salida(contnes+1)=H*edosnoise(contnes+1,:)'+r*randn;

  %% Prediccion
  
  
    mean_pre(1,contnes)=dt*(v*cos(pre_T(contnes)))+pre_X(contnes);
    mean_pre(2,contnes)=dt*(v*sin(pre_T(contnes)))+pre_Y(contnes);
    mean_pre(3,contnes)=dt*(v*tan(gammav)/L)+pre_T(contnes);
   % mean_pre{k+1}= mean_pre{k+1}';
    Y_pre(contnes+1)=H*mean_pre(:,contnes);
    P=F*P*F'+Q;
    
    %% Actualizacion
    S=H*P*H'+R;
    K(:,contnes)=P*H'./S; %Kalman gain

   %error(k+1)=x(k+1,1)-Y_pre(k+1);
   in(contnes+1)=(salida(contnes+1)-Y_pre(contnes+1));
   mean_c(:,contnes)=mean_pre(:,contnes)+(K(:,contnes)*in(contnes+1));
   pre_X(contnes+1)=mean_c(1,contnes); pre_Y(contnes+1)=mean_c(2,contnes); pre_T(contnes+1)=mean_c(3,contnes);
    
   P_up=(eye(3)-K(:,contnes)*H)*P; %updating covariance mat
   P=P_up; %overwriting cov mat
    
   
   xactual=mean_c(:,contnes)';
end

l=1;

% figure(2)
% plot(edos(:,1),edos(:,2),'k','LineWidth',1)
% title('Trayectoria')
% xlabel('X','fontsize',10);ylabel('Y','fontsize',14);
% axis equal
% hold on

function dx = sist(~,x)
%global v gammav L 
dx(1)=v*cos(x(3));
dx(2)=v*sin(x(3));
dx(3)=v*tan(gammav)/L;
dx=dx';
end

function dx = sistd(~,x)
%global d
dx(1)=dnes;
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