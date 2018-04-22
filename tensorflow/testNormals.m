Q = load('/home/avishai/Documents/workspace/ml_planner/data/abb_samples_noJL.db');
Q = Q(1:1e5,:);

N = size(Q,1);
%%
ix = randi(N);
p1 = Q(ix,:);
disp(['ix: ' num2str(ix)]);

idx = rangesearch(Q,p1,3);
NN = Q(idx{1},:);

% [R,~,e] = pca(NN);
[R,e] = Pca(NN');
pmean = mean(NN)';

e = e / norm(e);