clear all

f = @(x) 0.9*x + 2;
x = 0:1:10;
plot(x,f(x))

xr = rand(10,1)*2+4;

hold on
yr = f(xr) + (rand(length(xr),1)-0.5)/1;
P =  [xr yr];
P = [P; 4 10];
P = [P; 6 4];
P = [P; 2 2];

plot(P(:,1),P(:,2),'ro');

R = pca(P);
plot([5 5+R(1,1)], [f(5) f(5)+R(2,1)],'b','linewidth',3);

%%

% Estimate the leading component
C = grassmann_average(P, 2); % the second input is the number of component to estimate

plot([5 5+C(1,1)], [f(5) f(5)+C(2,1)],'g','linewidth',3);

%%


hold off
axis equal

% dlmwrite('../utils/samples.txt',P, ' ');