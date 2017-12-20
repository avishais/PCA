
% In all trials, d = 2.8
% last updated: 12/12/17

clear all
clc

d = 2.8;

%%
Dwo = load('Benchmark_RRT_envI_wo.txt'); 
Dwo = Dwo(Dwo(:,1)==1,:); 
t_wo = mean(Dwo(:,3));
clear Dwo

%%
planners = {'BiRRT','RRT'};
plannerType = planners{2};
switch plannerType
    case 'BiRRT'
        D = load('Benchmark_CBiRRT_envI_knn.txt');
        fprintf('Failures: \t%.1f\n', 100-sum(D(:,2))/size(D,1)*100);
        D = D(D(:,2)==1,:);
    case 'RRT'
        %D = load('Benchmark_RRT_envI_knn.txt'); 
        %D = load('Benchmark_RRT_envI_minTree.txt'); 
        %D = load('Benchmark_RRT_envI_nnR.txt'); 
        D = load('Benchmark_RRT_envI_dimpca.txt'); 
        fprintf('Failures: \t%.1f\n', 100-sum(D(:,2))/size(D,1)*100);
        D = D(D(:,2)==1,:); 
end

%%
disp(['Results for ' plannerType ':']);

%%
%%

r = sort(unique(D(:,1)));
for i = 1:length(r)
    M = D(D(:,1)==r(i), 1:end);
    t(i) = mean(M(:,4));
    t_ste(i) = std(M(:,4))/sqrt(size(M,1));
end

[tmin, im] = min(t);

%%
h = figure(1);
clf
errorbar(r,t,t_ste,'-k','linewidth',2);
hold on
plot(xlim, t_wo*[1 1], ':k','linewidth',2);
hold off
ylabel('mean runtime [msec]');
xlabel('k-nearest neighbors');
legend('w/ PCA','w/o PCA');

%%
fprintf('%d results for each k.\n', floor(size(D,1)/length(r)));
fprintf('k = %.1f provides the minimum runtime of: %.2f.\n', r(im), tmin);
fprintf('Best speed-up: %.2f.\n', t_wo/min(t));

