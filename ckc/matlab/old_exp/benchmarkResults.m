% Benchmark_RRT_envI_wo.txt - Planning without PCA - for comparison
% Benchmark_RRT_envI_w.txt - with PCA (including the nearest neighbor in the PCA) knn=20
% Benchmark_RRT_envI_w_2.txt - with PCA (not including the nearest neighbor in the PCA) knn=20
% Benchmark_RRT_envI_w_3.txt - with PCA (including the nearest neighbor in the PCA) knn=tree size
% Benchmark_RRT_envI_w_4.txt - with PCA (including the nearest neighbor in the PCA) knn=90
% Benchmark_RRT_envI_w_5.txt- with the minimum number of nodes in tree to operate PCA set to 1 (previously was 6)
% *Benchmark_RRT_envI_w_6.txt- with the minimum number of nodes in tree to operate PCA set to 3 (previously was 6)
% Benchmark_RRT_envI_w_7.txt- nearestR with radius 2.6

% (*) marks best so far
% In all trials, d = 2.8
% last updated: 12/11/17

clear all
clc

d = 2.8;
%%
planners = {'BiRRT','RRT','LazyRRT'};
plannerType = planners{2};
switch plannerType
    case 'BiRRT'
        D{1} = load('Benchmark_CBiRRT_envI_w.txt'); 
        D{2} = load('Benchmark_CBiRRT_envI_wo.txt');
        D{1} = [d*ones(size(D{1},1),1) D{1}];
        D{2} = [d*ones(size(D{2},1),1) D{2}];
        D{1} = D{1}(D{1}(:,2)==1,:); 
        D{2} = D{2}(D{2}(:,2)==1,:); 
    case 'RRT'
        D{1} = load('Benchmark_RRT_envI_w_6.txt'); 
        D{2} = load('Benchmark_RRT_envI_wo.txt'); 
        fprintf('Failures: \t%.1f, %.1f \n', 100-sum(D{1}(:,1))/size(D{1},1)*100, 100-sum(D{2}(:,1))/size(D{2},1)*100);
        D{1} = [d*ones(size(D{1},1),1) D{1}];
        D{2} = [d*ones(size(D{2},1),1) D{2}];
        D{1} = D{1}(D{1}(:,2)==1,:); 
        D{2} = D{2}(D{2}(:,2)==1,:); 
    case 'LazyRRT'
        D{1} = load('Benchmark_LazyRRT_envI_w.txt'); 
        D{2} = load('Benchmark_LazyRRT_envI_wo.txt'); 
        fprintf('Failures: \t%.1f, %.1f \n', 100-sum(D{1}(:,1))/size(D{1},1)*100, 100-sum(D{2}(:,1))/size(D{2},1)*100);
        D{1} = [d*ones(size(D{1},1),1) D{1}];
        D{2} = [d*ones(size(D{2},1),1) D{2}];
        D{1} = D{1}(D{1}(:,2)==1,:); 
        D{2} = D{2}(D{2}(:,2)==1,:); 
end

%%
disp(['Results for ' plannerType ':']);

%%

for k = 1:size(D,2)
    
    suc = D{k}(:,2)==1;
    F(:,k) = mean(D{k});
    
    tmin(k) = F(4,k);
end

% clc
disp('-----------------------------------');
fprintf('         \t\tw/\tw/o\n');
fprintf('Queries: \t\t%d\t%d\n', size(D{1},1), size(D{2},1));
fprintf('d =      \t\t%.1f\t%.1f\n', F(1,1), F(1,2));
fprintf('Avg. time (for d): \t%.2f\t%.2f \t(msec)\n', F(4,1)*1e3, F(4,2)*1e3);
fprintf('Min. time (for d): \t%.2f\t%.2f \t(msec)\n', min(D{1}(:,4))*1e3, min(D{2}(:,4))*1e3);
fprintf('Nodes in path:     \t%.1f\t%.1f\n', F(10,1), F(10,2));
fprintf('Nodes in trees:    \t%.1f\t%.1f\n', F(11,1), F(11,2));
disp('------------ Sampling -------------');
fprintf('Sampling time:    \t%.2f\t%.2f \t(msec)\n', F(15,1)*1e3, F(15,2)*1e3);
fprintf('Sampling count: \t%.1f\t%.1f \t(%%)\n', F(16,1)+F(17,1), F(16,2)+F(17,2));
fprintf('Sampling success: \t%.2f\t%.2f \t(%%)\n', 100*F(16,1)/(F(16,1)+F(17,1)), 100*F(16,2)/(F(16,2)+F(17,2)));
disp('------------ loc.-con. ------------');
fprintf('Loc.-con. time:    \t%.2f\t%.2f \t(msec)\n', F(12,1)*1e3, F(12,2)*1e3);
fprintf('Loc.-con. count:   \t%.2f\t%.2f \t\n', F(13,1), F(13,2));
fprintf('Loc.-con. success: \t%.2f\t%.2f \t(%%)\n', 100*F(14,1)/F(13,1), 100*F(14,2)/F(13,2));
disp('----------- Projection ------------');
fprintf('Proj. time:        \t%.2f\t%.2f \t(msec)\n', F(6,1)*1e3, F(6,2)*1e3);
fprintf('Proj. count:       \t%.2f\t%.2f \t\n', F(5,1), F(5,2));
disp('--------------- PCA ---------------');
fprintf('PCA time:          \t%.2f\t%.2f \t(msec)\n', F(19,1)*1e3, F(19,2)*1e3);
fprintf('PCA count:       \t%.2f\t%.2f \t\n', F(18,1), F(18,2));

%%
disp(' ');
fprintf('Speed-up (at d = %.1f):      %.2f\n', d, tmin(2)/tmin(1));

%%
%%
%%
% PCS
td = D{1}(:,4);
maxT = max(td);
T1 = linspace(0,maxT,100);
T1 = T1(2:end);
for i = 1:length(T1)
    sd = td < T1(i);
    md(i) = mean(td(sd));
    Md(i) = 1-sum(sd)/length(td);
end
%%
% GD
tg = D{2}(:,4);
maxT = max(tg);
T2 = linspace(0,maxT,100);
T2 = T2(2:end);
for i = 1:length(T2)
    sg = tg < T2(i);
    mg(i) = mean(tg(sg));
    Mg(i) = 1-sum(sg)/length(tg);
end

%%
h = figure(2);
clf
plot(T1,Md*100,'-k','linewidth',2);
hold on
plot(T2,Mg*100,'--k','linewidth',2);
hold off
xlabel('maximum runtime (sec)');
ylabel('failure rate (%)');
legend('wPCA','woPCA');
xlim([0 8]);%max([T1 T2])]);
% title(plannerType);
set(gca,'fontsize',13);
% set(h, 'Position', [100, 100, 800, 400]);
% print PCS_NR_runtime.eps -depsc -r200