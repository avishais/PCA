% Benchmark_RRT_envI_w_rB.txt - Vanilla use of PCA
% Benchmark_RRT_envI_w_rB_2.txt - Included the NN in the PCA computation

% last updated: 12/11/17

clear all
clc

%%
planners = {'BiRRT','RRT'};
plannerType = planners{2};
switch plannerType
    case 'BiRRT'
        D = load('Benchmark_CBiRRT_rB.txt');
    case 'RRT'
        D = load('Benchmark_RRT_wo_rB.txt');
        fprintf('Failures: \t%.1f %% \n', 100-sum(D(:,2))/size(D,1)*100);
        D = D(D(:,2)==1,:);
        D = D(D(:,1)<=2.4, :);
end

%%
disp(['Results for ' plannerType ':']);

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
ylabel('mean runtime [msec]');
xlabel('max. local-connection distance');
% xlim([0 6]);
% xlim([min(rd) max(rd)]);

