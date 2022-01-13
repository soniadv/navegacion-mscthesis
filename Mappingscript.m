clear all
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
  d=0.755;% 2*d=distance between left and right wheels
  l=2.5772; % l=distance between front and read wheels
  anguloderecha=-pi/3;                
  anguloizquierda=pi/3;
  derposleft=atan(l/(-d+l/tan(anguloderecha)));
  derposright=atan(l/(d+l/tan(anguloderecha)));
    izqposleft=atan(l/(-d+l/tan(anguloizquierda)));
    izqposright=atan(l/(d+l/tan(anguloizquierda)));
  
   [returnCode,depth]=sim.simxGetObjectHandle(clientID,'kinect_depth',sim.simx_opmode_blocking);
   [returnCode,rgb]=sim.simxGetObjectHandle(clientID,'kinect_rgb',sim.simx_opmode_blocking);
   [returnCode,leftmotor]=sim.simxGetObjectHandle(clientID,'nakedCar_motorLeft',sim.simx_opmode_blocking);
   [returnCode,rightmotor]=sim.simxGetObjectHandle(clientID,'nakedCar_motorRight',sim.simx_opmode_blocking);
   
   [returnCode,rightsteer]=sim.simxGetObjectHandle(clientID,'nakedCar_steeringRight',sim.simx_opmode_blocking);
   [returnCode,leftsteer]=sim.simxGetObjectHandle(clientID,'nakedCar_steeringLeft',sim.simx_opmode_blocking);
   
    [returnCode,camara]=sim.simxGetObjectHandle(clientID,'Cylinder0',sim.simx_opmode_blocking);
    [returnCode,cilindro]=sim.simxGetObjectHandle(clientID,'Cylinder',sim.simx_opmode_blocking);

  %% View
[returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,rgb,0,sim.simx_opmode_streaming);
[returnCode,resolution,buffer]=sim.simxGetVisionSensorDepthBuffer2(clientID,depth,sim.simx_opmode_streaming);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,3,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,3,sim.simx_opmode_blocking);


[returnCode,pos]=sim.simxGetObjectPosition(clientID,camara,cilindro,sim.simx_opmode_streaming);
[returnCode,Angles]=sim.simxGetObjectOrientation(clientID,camara,cilindro,sim.simx_opmode_streaming);

%tomar posicion y fotos al iniciar
[returnCode,pos]=sim.simxGetObjectPosition(clientID,camara,cilindro,sim.simx_opmode_buffer);
[returnCode,Angles]=sim.simxGetObjectOrientation(clientID,camara,cilindro,sim.simx_opmode_buffer);
[returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,rgb,0,sim.simx_opmode_buffer);
[returnCode,resolution,buffer]=sim.simxGetVisionSensorDepthBuffer2(clientID,depth,sim.simx_opmode_buffer);
posiciones(1,:)=pos;
angulos(1,:)=Angles;
imagenes(:,:,:,1)=image;
profundidad(:,:,1)=buffer;
count=1;
while 1 %posiciones(count,1)<=-0.88 || posiciones(count,1)==0
%    [returnCode,pos]=sim.simxGetObjectPosition(clientID,camara,cilindro,sim.simx_opmode_buffer);
%    [returnCode,Angles]=sim.simxGetObjectOrientation(clientID,camara,cilindro,sim.simx_opmode_buffer);
   [returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,rgb,0,sim.simx_opmode_buffer);
   [returnCode,resolution,buffer]=sim.simxGetVisionSensorDepthBuffer2(clientID,depth,sim.simx_opmode_buffer);
   count=count+1;
   imagenes(:,:,:,count)=image;
   profundidad(:,:,count)=buffer;
%    posiciones(count,:)=pos;
%    angulos(count,:)=Angles;
   pause(0.022);
   %45 por seg
    sim.simxSynchronousTrigger(clientID);
end


%% PARAR
[returnCode]=sim.simxSetJointTargetVelocity(clientID,leftmotor,0,sim.simx_opmode_blocking);
[returnCode]=sim.simxSetJointTargetVelocity(clientID,rightmotor,0,sim.simx_opmode_blocking);
end

imagenes=uint8(imagenes);

sim.delete(); % call the destructor!
disp('Program ended');