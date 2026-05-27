function x = reservoir_update_cesn(W, Win, LR, input, seq)
% run reservoir with internal weights W and input weights Win in input,
% which is given as column sequence. Return last state (column vector)

x = zeros(size(W,1),seq);

for i = 1:seq
    act = W*x(:,i) + Win*input(:,i);
    x(:,i) = (1-LR)*x(:,i) + LR*tanh(act);
end