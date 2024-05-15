function analyzeResults(info,ref,timeVal)
% Analyze parking results from mpc.

fprintf('Summary of results:\n')

%%
% mpc solver exit flag
flag = info.ExitFlag;
fprintf('3) Optimization exit flag = %d (Successful when positive)\n', flag);
fprintf('4) Elapsed time (s) for nlmpcmove = %.4f\n', timeVal);

%%
% final position of ego
e1 = info.Xopt(end,1)-ref(1);
e2 = info.Xopt(end,2)-ref(2);
e3 = rad2deg(info.Xopt(end,3)-ref(3));
fprintf('5) Final states error in x (m), y (m) and theta (deg):  %2.4f, %2.4f, %2.4f\n',e1,e2,e3);

%%
% final controller values
vFinal = info.MVopt(end,1);
deltaFinal = rad2deg(info.MVopt(end,2));
fprintf('6) Final control inputs speed (m/s) and steering angle (deg): %2.4f, %2.4f\n', vFinal,deltaFinal);

end