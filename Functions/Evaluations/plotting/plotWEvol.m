%%%%%

W_diag = createArray(size(W,1), size(W,3));
W_var = W_diag;
for t=1:size(W,3)
    W_diag(:,t)=diag(W(:,:,t));
    W_var(:,t) =sqrt(W_diag(:,t));
end

figure;
plot(ekfResult.elapsedTime, W_var');

%%%%%%
%P evol
P_diag = createArray(size(P,1), size(P,3));
P_var = P_diag;
for t=1:size(P,3)
    P_diag(:,t)=diag(P(:,:,t));
    P_var(:,t) =sqrt(P_diag(:,t));
end

figure;
plot(ekfResult.elapsedTime, P_var');
