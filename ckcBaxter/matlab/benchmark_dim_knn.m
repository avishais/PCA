
% In all trials, d = 2.8
% last updated: 12/12/17

clear all
clc

d = 1.;


%%
planners = {'CBiRRT','RRT'};
plannerType = planners{2};
switch plannerType
    case 'CBiRRT'
        D = load('Benchmark_CBiRRT_dimpca_knn.txt');
        fprintf('Failures: \t%.1f\n', 100-sum(D(:,2))/size(D,1)*100);
        D = D(D(:,2)==1,:);
        
        Dwo = load('Benchmark_CBiRRT_wo.txt');
        Dwo = Dwo(Dwo(:,1)==1,:);
        t_wo = mean(Dwo(:,3));
        clear Dwo
    case 'RRT'
        D = load('Benchmark_RRT_dimpca_knn_2.txt');
        fprintf('Failures: \t%.1f\n', 100-sum(D(:,3))/size(D,1)*100);
        D = D(D(:,3)==1,:);
        
        Dwo = load('Benchmark_RRT_wo.txt');
        Dwo = Dwo(Dwo(:,1)==1,:);
        t_wo = mean(Dwo(:,3));
        clear Dwo
end

%%
disp(['Results for ' plannerType ':']);

%%
knn = sort(unique(D(:,2)));
dim = sort(unique(D(:,1)));

[K, DM] = meshgrid(knn, dim);

T = zeros(size(K));
for i = 1:length(knn)
    for j = 1:length(dim)
        kk = K(j,i);
        dd = DM(j,i);
        M = D(D(:,1)==dd & D(:,2)==kk, 1:end);
        T(j,i) = mean(M(:,5));
    end
end

[tmin, im] = min(T(1:end));

%%
h = figure(1);
clf
surf(K, DM, T);
% hold on
% plot(xlim, t_wo*[1 1], ':k','linewidth',2);
% hold off
zlabel('mean runtime [msec]');
xlabel('k-nearest neighbors');
ylabel('number of eigenvectors for reconstruction');%
% legend('w/ PCA','w/o PCA');

%%
K1 = K(1:end);
DM1 = DM(1:end);
fprintf('k = %.1f and dim = %.1f provides the minimum runtime of: %.2f sec.\n', K1(im) , DM1(im), tmin);
fprintf('Best speed-up: %.2f.\n', t_wo/tmin);
clear K1 DM1
