function [score] = EvalFunc_TGR(target)
%EvalFunc_TGR | Evaluation Functions Tuned to the Golden Ratio
%   黄金比で調律した評価関数
%   output[score]：0~100
%   input[target]：0~
if target>=0
    score = 100 .* (((1+sqrt(5))/2).^target) .* exp(-target);
else
    disp('error:The input variable is below zero. The input must be a positive real number(absolute value).')
    beep
    return
end

%% 確認
% target = 0:0.1:10;
% score = 100 .* (((1+sqrt(5))/2).^target) .* exp(-target);
% figure
% grid on
% hold on
% plot(target,score,'.b')
% 
% figure
% grid on
% hold on
% plot(target,score,'.b')
% xscale log

end