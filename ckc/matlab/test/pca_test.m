x = -4:.2:8;
y = x;
r = -.1;
f = @(x,y) r*x.^2+r*y.^2;

[X,Y] = meshgrid(x,y);
Z = f(X,Y);

%% rand points

N = 20;
pr = rand(N,2) * 4 + rand(1,1)*12-4;

pr = [pr f(pr(:,1),pr(:,2))];

mpr = mean(pr);

%% pca
% pro = pr - repmat(mpr,N,1);

[coeff,score,latent] = pca(pr);

% C = cov(pro);
% [U,V] = eig(C);
% 
% v = diag(V);
% [v, iv]=sort(v,'descend');
% coeff2 = U(:,iv);

%%
ps2d = rand(1,3) * 2 - 1;
% ps2d = ps2d * 5;

%%
L = repmat(latent', 3, 1);

M = coeff.*L;
ps2dl = ps2d;

% M = coeff(:,1:2);
% ps2dl = ps2d(1:2);

ps = M * ps2dl' + mpr';

[norm(ps2dl) norm(ps)]

%
figure(1)
clf
mesh(X,Y,Z);
hold on
plot3(pr(:,1),pr(:,2),pr(:,3),'ok','markerfacecolor','k');
quiver3(mpr(1),mpr(2),mpr(3),coeff(1,1),coeff(2,1),coeff(3,1),'r','linewidth',3);
quiver3(mpr(1),mpr(2),mpr(3),coeff(1,2),coeff(2,2),coeff(3,2),'g','linewidth',3);
quiver3(mpr(1),mpr(2),mpr(3),coeff(1,3),coeff(2,3),coeff(3,3),'b','linewidth',3);
plot3(ps(1),ps(2),ps(3),'pr','markerfacecolor','r','markersize',14);
hold off
axis equal
grid on