clear all

Q = load('samplesTangent_short.db');
N = min(1e5, size(Q,1));
Q = Q(1:N,:);
P = Q(:,1:3);
E = Q(:,end);

%%
ix = randi(N);
p1 = P(ix,:);
disp(['ix: ' num2str(ix)]);

% NN = P(knnsearch(P,p1,'K',k),:);
idx = rangesearch(P,p1,1);
NN = P(idx{1},:);
% NN(:,3)=0;

% [R,~,e] = pca(NN);
[R,e] = Pca(NN');
pmean = mean(NN)';

e = e / norm(e);

%%
Nm = 1e4;
c = 'rgb';

figure(1)
scatter3(P(1:Nm,1),P(1:Nm,2),P(1:Nm,3),20,P(1:Nm,3),'.');
hold on
plot3(p1(1),p1(2),p1(3)*1.01,'ko','markerfacecolor','k');
plot3(NN(:,1),NN(:,2),NN(:,3)*1.01,'ko','markerfacecolor','c');
for i = 1:3
    quiver3(p1(1),p1(2),p1(3),R(1,i),R(2,i),R(3,i),2,'color',c(i),'linewidth',5);
end
hold off
axis equal
grid on
colormap(jet)
% xlim([-10 0]);
% ylim([-10 0]);

figure(2)
scatter3(P(1:Nm,1),P(1:Nm,2),E(1:Nm),20,E(1:Nm),'.');
% colormap(jet)

%%

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
