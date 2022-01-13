clear all
close all
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
  disp('Conectado');
  
  % enable the synchronous mode on the client:
  sim.simxSynchronous(clientID,true);
  
  % start the simulation:
  sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
  %% Handle
  
 d=0.325; % 2*d=distance between left and right wheels
 l=0.525; % l=distance between front and read wheels
 %load('cur40p.mat') %cargar trayectorias
  anguloderecha=-0.5;
  anguloizquierda=0.5;
  vderleft=atan(l/(-d+l/tan(anguloderecha)));
  vderright=atan(l/(d+l/tan(anguloderecha)));
  vizqleft=atan(l/(-d+l/tan(anguloizquierda)));
  vizqright=atan(l/(d+l/tan(anguloizquierda)));
   
  [returnCode,leftmotor]=sim.simxGetObjectHandle(clientID,'nakedCar_motorLeft',sim.simx_opmode_blocking);
  [returnCode,rightmotor]=sim.simxGetObjectHandle(clientID,'nakedCar_motorRight',sim.simx_opmode_blocking);
  
  [returnCode,rightsteer]=sim.simxGetObjectHandle(clientID,'nakedCar_steeringRight',sim.simx_opmode_blocking);
  [returnCode,leftsteer]=sim.simxGetObjectHandle(clientID,'nakedCar_steeringLeft',sim.simx_opmode_blocking);
  
  [returnCode,refcarr]=sim.simxGetObjectHandle(clientID,'Cylinder0',sim.simx_opmode_blocking);
  [returnCode,cilindro]=sim.simxGetObjectHandle(clientID,'Cylinder',sim.simx_opmode_blocking);
  
   
%   [returnCode,proxsensor]=sim.simxGetObjectHandle(clientID,'Proximity_sensor0',sim.simx_opmode_blocking);
  %streaming
  [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_streaming);
  [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_streaming);
 
    [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
%% Recta

%cond inicial
[returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
[returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
x0=[pos(1:2) Angles(3)];
xfin=[-2.9 -1.5 0];
angfin=[0];
punto=[-6 -1.3];
[gamma,vel,edos]=k_evasion_controlsc(x0,xfin,angfin,punto);
%[gamma,vel,edos]=k_evasion_nestedfun(x0,xfin,angfin,punto);
for i=1:length(gamma)
    %velocidad
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,6.3*vel(i),sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,6.3*vel(i),sim.simx_opmode_blocking);
    
    [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    tray1(i,:)=pos;
    
    %posicion deseada llantas
    posleft=atan(l/(-d+l/tan(gamma(i))));
    posright=atan(l/(d+l/tan(gamma(i))));
    %mover llantas
    [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,posleft,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,posright,sim.simx_opmode_blocking);
    sim.simxSynchronousTrigger(clientID);
end
[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);

[returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
[returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);

plot(tray1(:,1),tray1(:,2),'r')
%% vuelta en semaforo
[returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
[returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
x0=[pos(1:2) Angles(3)];
[gamma2,vel2,edos2]=pre_pathcontrol_wkal(x0,traj);
for i=1:length(gamma2)
        %velocidad
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,6.5*vel2(i),sim.simx_opmode_blocking);
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,6.5*vel2(i),sim.simx_opmode_blocking);   
   
    [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
   % ang2(i,:)=Angles;
    tray2(i,:)=pos;
    %posicion deseada llantas
    posleft=atan(l/(-d+l/tan(gamma2(i))));
    posright=atan(l/(d+l/tan(gamma2(i))));
    %mover llantas
    [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,posleft,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,posright,sim.simx_opmode_blocking);
    sim.simxSynchronousTrigger(clientID);
end
[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
plot(tray2(:,1),tray2(:,2),'r')
%% orientacion

[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,vizqleft,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,vizqright,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,2,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,2,sim.simx_opmode_blocking);
 [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
i=0;
while Angles(3)<1.5
    i=i+1;
    [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    trayc(i,:)=pos;
end

[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
%plot(trayc(:,1),trayc(:,2),'r')
%% recta mid-N
[returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
 [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
 x0=[pos(1:2) Angles(3)];
 xfin=[0.1 7.1 0]; angfin=[pi/2];
 punto=[0.65 3.5];
%[gamma4,vel4,edos4]=k_evasion_nestedfun(x0,xfin,angfin,punto);
[gamma4,vel4,edos4]=k_evasion_controlsc(x0,xfin,angfin,punto);

 for i=1:length(gamma4)
     %velocidad
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,6.5*vel4(i),sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,6.5*vel4(i),sim.simx_opmode_blocking);
     
     [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
     tray3(i,:)=pos;
     %posicion deseada llantas
     posleft=atan(l/(-d+l/tan(gamma4(i))));
     posright=atan(l/(d+l/tan(gamma4(i))));
     %mover llantas
     [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,posleft,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,posright,sim.simx_opmode_blocking);
     sim.simxSynchronousTrigger(clientID);
 end
   [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
plot(tray3(:,1),tray3(:,2),'r')
%% vuelta2
[returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
[returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
x0=[pos(1:2) Angles(3)];
load('cur240p.mat')
[gamma5,vel5,edos5]=pre_pathcontrol_wkal(x0,traj);
for i=1:length(gamma5)
    %velocidad
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,6.3*vel5(i),sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,6.3*vel5(i),sim.simx_opmode_blocking);
  
    [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    ang5(i,:)=Angles;
    tray4(i,:)=pos;
    %posicion deseada llantas
    posleft=atan(l/(-d+l/tan(gamma5(i))));
    posright=atan(l/(d+l/tan(gamma5(i))));
    %mover llantas
    [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,posleft,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,posright,sim.simx_opmode_blocking);
    sim.simxSynchronousTrigger(clientID);
end
[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
plot(tray4(:,1),tray4(:,2),'r')
%% orientacion

[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,vderleft,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,vderright,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,2,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,2,sim.simx_opmode_blocking);
 [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
i=0;
while Angles(3)>-0.1
    i=i+1;
    [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    trayc2(i,:)=pos;
end

[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
%% recta NE
[returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
 [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
 x0=[pos(1:2) Angles(3)];
  xfin=[6.8 10.5 0];
 angfin=[0];
 punto=[4 11];
%[gamma6,vel6,edos6]=k_evasion_nestedfun(x0,xfin,angfin,punto);
[gamma6,vel6,edos6]=k_evasion_controlsc(x0,xfin,angfin,punto);

 for i=1:length(gamma6)
     %velocidad
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,6.5*vel6(i),sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,6.5*vel6(i),sim.simx_opmode_blocking);
     
     [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
     tray6(i,:)=pos;
     %posicion deseada llantas
     posleft=atan(l/(-d+l/tan(gamma6(i))));
     posright=atan(l/(d+l/tan(gamma6(i))));
     %mover llantas
     [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,posleft,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,posright,sim.simx_opmode_blocking);
     sim.simxSynchronousTrigger(clientID);
 end
   [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
 plot(tray6(:,1),tray6(:,2),'r')
 %% vuelta NE
[returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
[returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
x0=[pos(1:2) Angles(3)];

[gamma7,vel7,edos7]=pre_pathcontrol_wkal(x0,traj3);
for i=1:length(gamma7)
    %velocidad
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,6.7*vel7(i),sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,6.7*vel7(i),sim.simx_opmode_blocking);
  
    [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    ang7(i,:)=Angles;
    tray7(i,:)=pos;
    %posicion deseada llantas
    posleft=atan(l/(-d+l/tan(gamma7(i))));
    posright=atan(l/(d+l/tan(gamma7(i))));
    %mover llantas
    [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,posleft,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,posright,sim.simx_opmode_blocking);
    sim.simxSynchronousTrigger(clientID);
end
[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
 plot(tray7(:,1),tray7(:,2),'r')
 %% orientacion

[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,vderleft,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,vderright,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,2,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,2,sim.simx_opmode_blocking);
 [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
i=0;
while Angles(3)>-1.5
    i=i+1;
    [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    trayc3(i,:)=pos;
end

[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
%% recta NE->E
[returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
 [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
 x0=[pos(1:2) Angles(3)];
 xfin=[10 4.6 0]; angfin=[-pi/2];
 punto=[9.3 7];
%[gamma8,vel8,edos8]=k_evasion_nestedfun(x0,xfin,angfin,punto);
[gamma8,vel8,edos8]=k_evasion_controlsc(x0,xfin,angfin,punto);
 for i=1:length(gamma8)
     %velocidad
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,6.5*vel8(i),sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,6.5*vel8(i),sim.simx_opmode_blocking);
     
     [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
     tray8(i,:)=pos;
     %posicion deseada llantas
     posleft=atan(l/(-d+l/tan(gamma8(i))));
     posright=atan(l/(d+l/tan(gamma8(i))));
     %mover llantas
     [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,posleft,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,posright,sim.simx_opmode_blocking);
     sim.simxSynchronousTrigger(clientID);
 end
   [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
 plot(tray8(:,1),tray8(:,2),'r')
 %% vuelta
[returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
[returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
x0=[pos(1:2) Angles(3)];

[gamma9,vel9,edos9]=pre_pathcontrol_wkal(x0,traj4);
for i=1:length(gamma9)
    %velocidad
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,6.7*vel9(i),sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,6.7*vel9(i),sim.simx_opmode_blocking);
    
     [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    tray9(i,:)=pos;
    %posicion deseada llantas
    posleft=atan(l/(-d+l/tan(gamma9(i))));
    posright=atan(l/(d+l/tan(gamma9(i))));
    %mover llantas
    [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,posleft,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,posright,sim.simx_opmode_blocking);
    sim.simxSynchronousTrigger(clientID);
end
[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
 plot(tray9(:,1),tray9(:,2),'r')
 %% orientacion

[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,vderleft,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,vderright,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,2,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,2,sim.simx_opmode_blocking);
 [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
i=0;
while Angles(3)<0
    i=i+1;
    [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
    trayc4(i,:)=pos;
end

[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
%% recta E->Cen
[returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
 [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
 x0=[pos(1:2) Angles(3)];
 xfin=[2 2.3 0];
 angfin=[3];
 punto=[6 0.5];
%[gamma10,vel10,edos10]=k_evasion_nestedfun(x0,xfin,angfin,punto);
[gamma10,vel10,edos10]=k_evasion_controlsc(x0,xfin,angfin,punto);

 for i=1:length(gamma10)
     %velocidad
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,6.5*vel10(i),sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,6.5*vel10(i),sim.simx_opmode_blocking);
     
     [returnCode,pos]=sim.simxGetObjectPosition(clientID,refcarr,cilindro,sim.simx_opmode_buffer);
     tray10(i,:)=pos;
     %posicion deseada llantas
     posleft=atan(l/(-d+l/tan(gamma10(i))));
     posright=atan(l/(d+l/tan(gamma10(i))));
     %mover llantas
     [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,posleft,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,posright,sim.simx_opmode_blocking);
     sim.simxSynchronousTrigger(clientID);
 end
   [returnCode]=sim.simxSetJointTargetPosition(clientID,leftsteer,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetPosition(clientID,rightsteer,0,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
   plot(tray10(:,1),tray10(:,2),'r') 


%% PARAR
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);

% stop the simulation:
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

% Now close the connection to CoppeliaSim:
sim.simxFinish(clientID);

else
    disp('Failed connecting to remote API server');
    

end


sim.delete(); % call the destructor!
disp('Program ended');


