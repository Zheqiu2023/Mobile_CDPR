function ref_traj = minimum_snap_simple(waypts, T)
%% condition
v0 = [0,0,0,0,0,0];
a0 = [0,0,0,0,0,0];
v1 = [0,0,0,0,0,0];
a1 = [0,0,0,0,0,0];
ts = arrangeT(waypts,T);
n_order = 5;

%% trajectory plan
polys_x = minimum_snap_single_axis_simple(waypts(1,:),ts,n_order,v0(1),a0(1),v1(1),a1(1));
polys_y = minimum_snap_single_axis_simple(waypts(2,:),ts,n_order,v0(2),a0(2),v1(2),a1(2));
polys_z = minimum_snap_single_axis_simple(waypts(3,:),ts,n_order,v0(3),a0(3),v1(3),a1(3));
polys_alpha = minimum_snap_single_axis_simple(waypts(4,:),ts,n_order,v0(4),a0(4),v1(4),a1(4));
polys_beta = minimum_snap_single_axis_simple(waypts(5,:),ts,n_order,v0(5),a0(5),v1(5),a1(5));
polys_gamma = minimum_snap_single_axis_simple(waypts(6,:),ts,n_order,v0(6),a0(6),v1(6),a1(6));

%% result show
figure(1)
plot3(waypts(1,:),waypts(2,:),waypts(3,:),'*r');hold on;
plot3(waypts(1,:),waypts(2,:),waypts(3,:),'--b');
% plot(waypts(1,:),waypts(2,:),'*r');hold on;
% plot(waypts(1,:),waypts(2,:),'b--');
title('minimum snap trajectory');
color = ['grc'];
for i=1:size(polys_x,2)
    tt = ts(i):0.05:ts(i+1);
    xx = polys_vals(polys_x,ts,tt,0);
    yy = polys_vals(polys_y,ts,tt,0);
    zz = polys_vals(polys_z,ts,tt,0);
    alpha = polys_vals(polys_alpha,ts,tt,0);
    beta = polys_vals(polys_beta,ts,tt,0);
    gamma = polys_vals(polys_gamma,ts,tt,0);
    plot3(xx,yy,zz,color(mod(i,3)+1));
%     plot(xx,yy,color(mod(i,3)+1));
end

figure(2)
tt = 0:0.05:T;
xx = polys_vals(polys_x,ts,tt,0);
yy = polys_vals(polys_y,ts,tt,0);
zz = polys_vals(polys_z,ts,tt,0);
alpha = polys_vals(polys_alpha,ts,tt,0);
beta = polys_vals(polys_beta,ts,tt,0);
gamma = polys_vals(polys_gamma,ts,tt,0);

vxx = polys_vals(polys_x,ts,tt,1);
vyy = polys_vals(polys_y,ts,tt,1);
vzz = polys_vals(polys_z,ts,tt,1);
walpha = polys_vals(polys_alpha,ts,tt,1);
wbeta = polys_vals(polys_beta,ts,tt,1);
wgamma = polys_vals(polys_gamma,ts,tt,1);

axx = polys_vals(polys_x,ts,tt,2);
ayy = polys_vals(polys_y,ts,tt,2);
azz = polys_vals(polys_z,ts,tt,2);
aalpha = polys_vals(polys_alpha,ts,tt,2);
abeta = polys_vals(polys_beta,ts,tt,2);
agamma = polys_vals(polys_gamma,ts,tt,2);

subplot(3,6,1),plot(tt,xx);title('x position');
subplot(3,6,2),plot(tt,yy);title('y position');
subplot(3,6,3),plot(tt,zz);title('z position');
subplot(3,6,4),plot(tt,alpha);title('alpha angle');
subplot(3,6,5),plot(tt,beta);title('beta angle');
subplot(3,6,6),plot(tt,gamma);title('gamma angle');
subplot(3,6,7),plot(tt,vxx);title('x velocity');
subplot(3,6,8),plot(tt,vyy);title('y velocity');
subplot(3,6,9),plot(tt,vzz);title('z velocity');
subplot(3,6,10),plot(tt,walpha);title('alpha velocity');
subplot(3,6,11),plot(tt,wbeta);title('beta velocity');
subplot(3,6,12),plot(tt,wgamma);title('gamma velocity');
subplot(3,6,13),plot(tt,axx);title('x acceleration');
subplot(3,6,14),plot(tt,ayy);title('y acceleration');
subplot(3,6,15),plot(tt,azz);title('z acceleration');
subplot(3,6,16),plot(tt,aalpha);title('alpha acceleration');
subplot(3,6,17),plot(tt,abeta);title('beta acceleration');
subplot(3,6,18),plot(tt,agamma);title('gamma acceleration');

ref_traj = [xx;yy;zz;alpha;beta;gamma;  vxx;vyy;vzz;walpha;wbeta;wgamma;    axx;ayy;azz;aalpha;abeta;agamma];

end

function polys = minimum_snap_single_axis_simple(waypts,ts,n_order,v0,a0,ve,ae)
p0 = waypts(1);
pe = waypts(end);

n_poly = length(waypts)-1;
n_coef = n_order+1;

% compute Q
Q_all = [];
for i=1:n_poly
    Q_all = blkdiag(Q_all,computeQ(n_order,3,ts(i),ts(i+1)));
end
b_all = zeros(size(Q_all,1),1);

Aeq = zeros(4*n_poly+2,n_coef*n_poly);
beq = zeros(4*n_poly+2,1);

% start/terminal pva constraints  (6 equations)
Aeq(1:3,1:n_coef) = [calc_tvec(ts(1),n_order,0);
                     calc_tvec(ts(1),n_order,1);
                     calc_tvec(ts(1),n_order,2)];
Aeq(4:6,n_coef*(n_poly-1)+1:n_coef*n_poly) = ...
                    [calc_tvec(ts(end),n_order,0);
                     calc_tvec(ts(end),n_order,1);
                     calc_tvec(ts(end),n_order,2)];
beq(1:6,1) = [p0,v0,a0,pe,ve,ae]';

% mid p constraints    (n_ploy-1 equations)
neq = 6;
for i=1:n_poly-1
    neq=neq+1;
    Aeq(neq,n_coef*i+1:n_coef*(i+1)) = calc_tvec(ts(i+1),n_order,0);
    beq(neq) = waypts(i+1);
end

% continuous constraints  ((n_poly-1)*3 equations)
for i=1:n_poly-1
    tvec_p = calc_tvec(ts(i+1),n_order,0);
    tvec_v = calc_tvec(ts(i+1),n_order,1);
    tvec_a = calc_tvec(ts(i+1),n_order,2);
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_p,-tvec_p];
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_v,-tvec_v];
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_a,-tvec_a];
end

Aieq = [];
bieq = [];

p = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq);

polys = reshape(p,n_coef,n_poly);

end
