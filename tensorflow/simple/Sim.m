clear all

x = -5:0.1:10;
y = 0:0.1:15;

c = 'rgb';

[X,Y] = meshgrid(x,y);

% Z = @(x,y) -(x.^2 + y.^2)/10 + 2;
Z = @(x,y) branin(x,y);

%%

p1 = gen_rand(1);
p1(3) = Z(p1(1),p1(2));

P = (rand(10,2)-0.5)*2;
P(:,1) = P(:,1)+p1(1); P(:,2) = P(:,2)+p1(2);
P(:,3) = Z(P(:,1),P(:,2));

[R,~,e] = pca(P);
pmean = mean(P)';
%% New point in plane and reconstruction
z = rand(2,100)*10 - 5;
pz = R(:,1:2) * z + pmean;
pz = pz';


%%
figure(1)
mesh(X,Y,Z(X,Y));
hold on
plot3(p1(1),p1(2),p1(3)*1.01,'ko','markerfacecolor','k');
plot3(P(:,1),P(:,2),P(:,3)*1.01,'ko','markerfacecolor','c');
plot3(pz(:,1),pz(:,2),pz(:,3),'ko','markerfacecolor','y');


for i = 1:3
    quiver3(p1(1),p1(2),p1(3),R(1,i),R(2,i),R(3,i),2,'color',c(i),'linewidth',5);
end
hold off
axis equal
% xlim([-10 0]);
% ylim([-10 0]);


function [y] = branin(x1,x2)


t = 1 / (8*pi);
s = 10;
r = 6;
c = 5/pi;
b = 5.1 / (4*pi^2);
a = 1;

term1 = a * (x2 - b*x1.^2 + c*x1 - r).^2;
term2 = s*(1-t)*cos(x1);

y = term1 + term2 + s;

y = y/10;

end

function p = gen_rand(n)
p(:,1) = rand(n,1)*15 - 5;
p(:,2) = rand(n,1)*15 - 0;
end
