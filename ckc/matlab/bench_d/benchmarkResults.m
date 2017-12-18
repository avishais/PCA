% Benchmark_RRT_envI_w_rB.txt - Vanilla use of PCA
% Benchmark_RRT_envI_w_rB_2.txt - Included the NN in the PCA computation

% last updated: 12/11/17

clear all
clc

d = 2.8;
%%
planners = {'BiRRT','RRT'};
plannerType = planners{2};
switch plannerType
    case 'BiRRT'
        D1 = load('Benchmark_BiRRT_PCS_3poles_rB.txt');
        D2 = load('Benchmark_BiRRT_GD_3poles_rB.txt');
    case 'RRT'
        D{1} = load('Benchmark_RRT_envI_w_rB_2.txt'); 
        D{2} = load('Benchmark_RRT_envI_wo_rB.txt'); 
        fprintf('Failures: \t%.1f, %.1f \n', 100-sum(D{1}(:,2))/size(D{1},1)*100, 100-sum(D{2}(:,2))/size(D{2},1)*100);
        D{1} = D{1}(D{1}(:,2)==1,:); 
        D{2} = D{2}(D{2}(:,2)==1,:); 
end

%%
disp(['Results for ' plannerType ':']);

%%

for k = 1:size(D,2)
    
    r{k} = sort(unique(D{k}(:,1)));
    for i = 1:length(r{k})
        M = D{k}(D{k}(:,1)==r{k}(i), 1:end);
        t{k}(i) = mean(M(:,4))*1e3;
        t_ste{k}(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
    end
    
    [tmin(k), im(k)] = min(t{k});
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
legend('w/','w/o');
% xlim([0 6]);
% xlim([min(rd) max(rd)]);

%%

for k = 1:size(D,2)
    
    sS = d;%r{k}(im(k));
    
    D{k} = D{k}(D{k}(:,1)==sS, 1:end);
    suc = D{k}(:,2)==1;
    F(:,k) = mean(D{k});
    
    tmin(k) = F(4,k);
end

clc
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
fprintf('Avg. speed-up (for all d):  %.2f\n', mean(t{2}'./t{1}'));

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
T2 = linspace(0,maxT,1000);
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
xlim([0 1.2]);%max([T1 T2])]);
% title(plannerType);
set(gca,'fontsize',13);
% set(h, 'Position', [100, 100, 800, 400]);
% print PCS_NR_runtime.eps -depsc -r200