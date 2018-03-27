x = -10:0.1:10;
y = x;

c = 'rgb';

[X,Y] = meshgrid(x,y);

Z = @(x,y) -(x.^2 + y.^2)/10 + 2;

%%

p1 = rand(2,1)*20 - 10; 
p1(3) = Z(p1(1),p1(2));

P = (rand(10,2)-0.5)*4;
P(:,1) = P(:,1)+p1(1); P(:,2) = P(:,2)+p1(2);
P(:,3) = Z(P(:,1),P(:,2));

[R,~,e] = pca(P);
pmean = mean(P)';
%% New point in plane and reconstruction
z = rand(2,1000)*30 - 15;
pz = R(:,1:2) * z + pmean;
pz = pz';

%% Project random point to new plane

% pn = [-10; -5; 5];
pn = rand(3,1)*30 - 15;
zn = (pn-pmean)' * R(:,1:3);

pnz = R(:,1:3) * zn' + pmean;
pnz = pnz';
pn = pn';
%%
figure(1)
mesh(X,Y,Z(X,Y));
hold on
plot3(p1(1),p1(2),p1(3)*1.01,'ko','markerfacecolor','k');
plot3(P(:,1),P(:,2),P(:,3)*1.01,'ko','markerfacecolor','c');
plot3(pz(:,1),pz(:,2),pz(:,3),'ko','markerfacecolor','y');
plot3(pn(:,1),pn(:,2),pn(:,3),'ko','markerfacecolor','k');
plot3(pnz(:,1),pnz(:,2),pnz(:,3)*1.05,'ko','markerfacecolor','m','markersize',15);

for i = 1:3
    quiver3(p1(1),p1(2),p1(3),R(1,i),R(2,i),R(3,i),2,'color',c(i),'linewidth',5);
end
hold off
axis equal
% xlim([-10 0]);
% ylim([-10 0]);