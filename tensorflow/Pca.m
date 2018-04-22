function [eigvector,eigvalue] = Pca(data)

[r,c]=size(data);

% Compute the mean of the data matrix "The mean of each row" (Equation (10))
m=mean(data')';
% Subtract the mean from each image (Equation (11))
d=data-repmat(m,1,c);

% Compute the covariance matrix (co) (Equation (11))
co=(1/c-1)*d*d';

% Compute the eigen values and eigen vectors of the covariance matrix
[eigvector,eigvl]=eig(co);

% Sort the eigen vectors according to the eigen values
eigvalue = diag(eigvl);
[~, index] = sort(-eigvalue);
index = flipud(index);
eigvalue = -eigvalue(index);
eigvector = eigvector(:, index);

% % EigenvectorPer is the percentage of the selected eigenvectors
% EigenvectorPer=50;
% PCASpace=eigvector(:,1:EigenvectorPer*size(eigvector,2)/100);

end

