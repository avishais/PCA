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
        D{1} = load('Benchmark_RRT_wo_rB.txt');
        D{2} = load('Benchmark_RRT_wo_rB.txt');
        fprintf('Failures: \t%.1f %% \n', 100-sum(D{1}(:,2))/size(D{1},1)*100);
        D{1} = D{1}(D{1}(:,2)==1,:);
        D{2} = D{2}(D{2}(:,2)==1,:);
end

%%
disp(['Results for ' plannerType ':']);

%%

for j = 1:size(D,2)
    
    r{j} = sort(unique(D{j}(:,1)));
    for i = 1:length(r{j})
        M = D{j}(D{j}(:,1)==r{j}(i), 1:end);
        t{j}(i) = mean(M(:,4));
        t_ste{j}(i) = std(M(:,4))/sqrt(size(M,1));
    end
    
    [tmin(j), im(j)] = min(t{j});
end

%%
h = figure(1);
clf
errorbar(r{1},t{1},t_ste{1},'-k','linewidth',2);
hold on
errorbar(r{2},t{2},t_ste{2},'--k','linewidth',2);
hold off
ylabel('mean runtime [msec]');

xlabel('max. local-connection distance');
% xlim([0 6]);
% xlim([min(rd) max(rd)]);

