clear all

R = load('net.netxt');
n = R(1)+1;
R(1) = [];

%%
% Weights
for i = 1:n
    r = R(1);
    w = R(2);
    R(1:2)=[];
    
    Wt = R(1:r*w);
    R(1:r*w) = [];
    
    W{i} = reshape(Wt, [w, r])';
end

% Biases
for i = 1:n
    w = R(1);
    R(1)=[];
    
    bt = R(1:w);
    R(1:w) = [];
    
    b{i} = bt;
end

x_max = R(1:numel(R)/2); x_min = R(numel(R)/2+1:numel(R));

%% Test net
Nm = 1e2;
X = load('samplesTangent_short.db');
Xi_nz = normz(X(1:Nm,1:3), x_max(1:3), x_min(1:3));
Xi = X(1:Nm,1:3);
ro1 = X(1:Nm,4:6);
ro2 = X(1:Nm,7:9);
Xo_net = denormz(Net(Xi_nz, W, b), x_max([4:9, 13]), x_min([4:9, 13]));
r1 = Xo_net(:,1:3);
r2 = Xo_net(:,4:6);

Eo = X(:,13);
E = Xo_net(:,7);

%%

figure(1)
scatter3(Xi(:,1),Xi(:,2),Xi(:,3));
hold on
quiver3(Xi(:,1),Xi(:,2),Xi(:,3),ro1(:,1),ro1(:,2),ro1(:,3),2,'color','b','linewidth',1);
quiver3(Xi(:,1),Xi(:,2),Xi(:,3),r1(:,1),r1(:,2),r1(:,3),2,'color','r','linewidth',1);
% quiver3(Xi(:,1),Xi(:,2),Xi(:,3),ro2(:,1),ro2(:,2),ro2(:,3),2,'color','b','linewidth',1);
% quiver3(Xi(:,1),Xi(:,2),Xi(:,3),r2(:,1),r2(:,2),r2(:,3),2,'color','r','linewidth',1);
hold off
axis equal
grid on
colormap(jet)
% xlim([-10 0]);
% ylim([-10 0]);

%%
figure(2)
scatter3(Xi(1:Nm,1),Xi(1:Nm,2),Eo(1:Nm),20,Eo(1:Nm),'.');
hold on
scatter3(Xi(1:Nm,1),Xi(1:Nm,2),E(1:Nm),20,E(1:Nm),'x');
k = randi(Nm, 1000, 1);
plot3(Xi(k,1),Xi(k,2),Eo(k),'ko','markerfacecolor','b');
plot3(Xi(k,1),Xi(k,2),E(k),'ko','markerfacecolor','r');
hold off

% colormap(jet)

%%

% x_in = normz(Xi(1,:), x_max(1:3), x_min(1:3));
% x_out = normz(Xo(1,:), x_max([4:9 13]), x_min([4:9 13]));
% x_out_net = Net(x_in, W, b);

%%
function x_out = Net(x_in, W, b)

T = x_in;%normz(x_in, x_max, x_min);
for i = 1:numel(W)
    T = T*W{i} + b{i};
    
    % T = sigmoid(T);
    T = tanh(T);
end

x_out = T;

end

function x = normz(x, x_max, x_min)

n = size(x,1);
x = (x-repmat(x_min,n,1))./repmat(x_max-x_min,n,1);

end

function x = denormz(x, x_max, x_min)

n = size(x,1);
x = x.*repmat(x_max-x_min,n,1) + repmat(x_min,n,1);

end

%%

function y = sigmoid(x,c,a)

narginchk(1,3) 
if nargin<3
    a = 1; 
else
    assert(isscalar(a)==1,'a must be a scalar.') 
end

if nargin<2
    c = 0; 
else
    assert(isscalar(c)==1,'c must be a scalar.') 
end

y = 1./(1 + exp(-a.*(x-c)));

end

function y = elu(x)

alpha1 = 1;
y = x.*(x > 0) + alpha1*(exp(x) - 1).*(x <= 0);

end
