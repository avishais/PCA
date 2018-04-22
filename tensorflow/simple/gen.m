clear all

Z = @(x,y) branin(x,y);

c = 'rgb';

N = 1e6;
P = gen_rand(N);
P(:,3) = Z(P(:,1),P(:,2));

k = 100;
%% Log data
% Q = zeros(N, 3+9+1);
% for i = 1:N
%     clc
%     disp(i/N*100);
%     p = P(i,:);
%     
%     %     NN = P(knnsearch(P,p,'K',k),:);
%     idx = rangesearch(P,p,1);
%     NN = P(idx{1},:);
%     
%     [R,e] = Pca(NN');
%     e = e / norm(e);
% 
%     Q(i,:) = [p, R(1:end), e(3)];
%     
%     % Log
%     if mod(i, 5e3)==0
%         dlmwrite('samples.db',Q,' ');
%     end
% end

% dlmwrite('samples.db',P,' ');

%%
Nm = 1e4;

figure(1)
scatter3(Q(1:Nm,1),Q(1:Nm,2),Q(1:Nm,3),'.');
hold on
quiver3(Q(1:Nm,1),Q(1:Nm,2),Q(1:Nm,3),Q(1:Nm,10),Q(1:Nm,11),Q(1:Nm,12),2,'color','b','linewidth',1);
hold off
axis equal
grid on




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
