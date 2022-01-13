clear all
Spill = importdata('MapPointsSave.txt');

Spill=-Spill;

X = -Spill(:,3);
Y = Spill(:,1);
Z = Spill(:,2);

plot3(X,Y,Z,'bo','MarkerSize',1)
hold on
xlabel('x');
ylabel('y');
zlabel('z');
axis equal