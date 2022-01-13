clear all
close all

%% Recta
p1=9; %puntos
x1=7; x2=7.9; %coordenadas x inicial y final
y1=10.5; y2=10.5; %%coordenadas x inicial y final

  traj(:,1)=linspace(x1,x2,p1);
  traj(:,2)=linspace(y1,y2,p1);

%% Semicírculo
p2=30; %num de puntos
xin=[8 10.5];%coordenada inicial - no acabar y empezar en el mismo punto 
rad=2;
   freq=linspace(pi/2,0,p2); x=0; y=-rad;
   
for i=p1+1:p1+length(freq)
    traj(i,1)=rad*cos(freq(i-p1))+xin(1)+x;
    traj(i,2)=rad*sin(freq(i-p1))+xin(2)+y;
end


% p3=3;
% x1=0.5; x2=0.5;
% y1=0.5; y2=1.5;
% 
%   traj(p1+p2+1:p1+p2+p3,1)=linspace(x1,x2,p3);
%   traj(p1+p2+1:p1+p2+p3,2)=linspace(y1,y2,p3);

% 
% p4=10;
% xin=[0.5 -5.5];
%  freq=linspace(-pi/2,0,p4); x=0; y=rad;
%  
%  for i=p1+p2+p3+1:p1+p2+p3+length(freq)
%     traj(i,1)=rad*cos(freq(i-p1-p2-p3))+xin(1)+x;
%     traj(i,2)=rad*sin(freq(i-p1-p2-p3))+xin(2)+y;
%  end
% 
% p5=20;
% x1=2; x2=2;
% y1=-4; y2=0;
% 
%   traj(p1+p2+p3+p4+1:p1+p2+p3+p4+p5,1)=linspace(x1,x2,p5);
%   traj(p1+p2+p3+p4+1:p1+p2+p3+p4+p5,2)=linspace(y1,y2,p5);


 
  
  
 
figure(1)
plot(traj(:,1),traj(:,2),'b-o')
axis equal
%% 
% time=[0:0.1:73];
% robot_traj=[  10 10;
%   10 60;
%   80 80;
%   50 10 ];
% 
% gl=interp1( linspace(0, 73, size(robot_traj,1)), robot_traj, time);
% figure(1)
%  plot(gl(:,1),gl(:,2),'b:o')