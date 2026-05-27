function x = reservoir_update(W, Win, LR, input, seq, pr)
% run reservoir with internal weights W and input weights Win in input,
% which is given as column sequence. Return last state (column vector)

x = zeros(size(W,1),seq);

for i = 1:seq
    act = W(:,:,pr)*x(:,i) + Win(:,:,pr)*input(:,i);
    x(:,i) = (1-LR)*x(:,i) + LR*tanh(act);
end