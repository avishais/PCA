
% In all trials, d = 2.8
% last updated: 12/12/17

clear all
clc

d = 2.8;
%%
planners = {'BiRRT','RRT'};
plannerType = planners{2};
switch plannerType
    case 'BiRRT'
        D1 = load('Benchmark_BiRRT_PCS_3poles_rB.txt');
    case 'RRT'
        %D = load('Benchmark_RRT_envI_knn.txt'); 
        D = load('Benchmark_RRT_envI_minTree.txt'); 
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
    t(i) = mean(M(:,4))*1e3;
    t_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
end

[tmin, im] = min(t);

%%
h = figure(1);
clf
errorbar(r,t,t_ste,'-k','linewidth',2);
ylabel('mean runtime [msec]');
xlabel('k-nearest neighbors');

%%
fprintf('%d results for each k.\n', floor(size(D,1)/length(r)));
fprintf('k = %d provides the minimum runtime of: %.2f.\n', r(im), tmin);

