function CdprPlot(ep_traj, car_traj, params, t_vec, alpha)
%% plot initial poses
f = figure(1);
xlabel('x');
ylabel('y');
% 车身起点及终点
plot(car_traj(1,1), car_traj(2,1),'rx');hold on;
plot(car_traj(1,end), car_traj(2,end),'rx');hold on;
% 末端平台起点及终点
plot(ep_traj(1,1), ep_traj(2,1),'gx');hold on;
plot(ep_traj(1,end), ep_traj(2,end),'gx');

%% set bounds
xbound = [-7 7];
ybound = [-7 7];
xlim(xbound);
ylim(ybound);

%% plot
f.Name = ['Automated Parking Animation'];
% plot the optimal XY trajectory
plot(car_traj(1,:),car_traj(2,:),'b.');hold on;
plot(ep_traj(1,:),ep_traj(2,:),'b.');
% animate car moves
animateCdpr(gca, t_vec, ep_traj, car_traj, alpha, params);

end

function animateCdpr(ax, tOut, ep_traj, car_traj, alpha, params)
% Playback CDPR motions
tSample = tOut;
xSample = car_traj(1,:);
ySample = car_traj(2,:);
thSample = car_traj(3,:);

W1 = params.W;
L1 = params.L;
T0Cdpr = [1 0 L1/2;
    0 1 0;
    0 0 1];
% T0Cdpr = [1 0 0;
%           0 1 0;
%           0 0 1];
cdprPoints = transformPoints(getRectangleVertices(L1, W1), T0Cdpr);

W2 = params.Wwheel;
L2 = params.Lwheel;
wheelPoints = getRectangleVertices(L2, W2);

hold on;
hCdpr = plot(ax, cdprPoints(:,1), cdprPoints(:,2), 'color', [1 0.5 0], 'LineWidth', 1, 'Visible', 'off');
hWheelFL = fill(ax, wheelPoints(:,1), wheelPoints(:,2), 'k', 'Visible', 'off');
hWheelFR = fill(ax, wheelPoints(:,1), wheelPoints(:,2), 'k', 'Visible', 'off');
hWheelRL = fill(ax, wheelPoints(:,1), wheelPoints(:,2), 'k', 'Visible', 'off');
hWheelRR = fill(ax, wheelPoints(:,1), wheelPoints(:,2), 'k', 'Visible', 'off');
hPoint = plot(ax, 0, 0, 'r.', 'Visible', 'off');
hPoint2 = plot(ax, 0, 0, 'r.', 'Visible', 'off');
hPoint3 = plot(ax, 0, 0, 'g.', 'Visible', 'off');

for i = 1:length(tSample)
    x = xSample(i);
    y = ySample(i);
    th = thSample(i);
%     alphaFL = alpha(i, 1);
%     alphaFR = alpha(i, 2);
    alphaFL = 0;
    alphaFR = 0;

    % Draw CDPR
    TCdpr = [cos(th) -sin(th) x;
             sin(th) cos(th) y;
             0 0 1];
    pts = transformPoints(cdprPoints, TCdpr);
    hCdpr.XData = pts(:,1);
    hCdpr.YData = pts(:,2);

    % Draw CDPR wheels
    TWheelFL = [cos(alphaFL) -sin(alphaFL) 0;
                sin(alphaFL) cos(alphaFL) 0;
                0 0 1];
    TWheelFR = [cos(alphaFR) -sin(alphaFR) 0;
                sin(alphaFR) cos(alphaFR) 0;
                0 0 1];

%     TOffsetFL = [1 0 L1/2; 0 1 W1/2; 0 0 1];
%     TOffsetFR = [1 0 L1/2; 0 1 -W1/2; 0 0 1];
%     TOffsetRL = [1 0 -L1/2; 0 1 W1/2; 0 0 1];
%     TOffsetRR = [1 0 -L1/2; 0 1 -W1/2; 0 0 1];
    TOffsetFL = [1 0 L1; 0 1 W1/2; 0 0 1];
    TOffsetFR = [1 0 L1; 0 1 -W1/2; 0 0 1];
    TOffsetRL = [1 0 0; 0 1 W1/2; 0 0 1];
    TOffsetRR = [1 0 0; 0 1 -W1/2; 0 0 1];

    pts = transformPoints(wheelPoints, TCdpr*TOffsetFL*TWheelFL);
    hWheelFL.XData = pts(:,1);
    hWheelFL.YData = pts(:,2);

    pts = transformPoints(wheelPoints, TCdpr*TOffsetFR*TWheelFR);
    hWheelFR.XData = pts(:,1);
    hWheelFR.YData = pts(:,2);

    pts = transformPoints(wheelPoints, TCdpr*TOffsetRL);
    hWheelRL.XData = pts(:,1);
    hWheelRL.YData = pts(:,2);

    pts = transformPoints(wheelPoints, TCdpr*TOffsetRR);
    hWheelRR.XData = pts(:,1);
    hWheelRR.YData = pts(:,2);

    % Draw midpoint of rear axle
    hPoint.XData = x;
    hPoint.YData = y;

    % Draw center point
    pts = transformPoints([L1/2 0], TCdpr);
    hPoint2.XData = pts(:,1);
    hPoint2.YData = pts(:,2);

    % Draw end-platform
    hPoint3.XData = ep_traj(1,i);
    hPoint3.YData = ep_traj(2,i);

    hCdpr.Visible = 'on';
    hWheelFL.Visible = 'on';
    hWheelFR.Visible = 'on';
    hWheelRL.Visible = 'on';
    hWheelRR.Visible = 'on';
    hPoint.Visible = 'on';
    hPoint2.Visible = 'on';
    hPoint3.Visible = 'on';
    
    % pause for animation
    drawnow;
    pause(0.1)
end
end

function points = getRectangleVertices(L, W)
points = [L/2, W/2;
    L/2, -W/2;
    -L/2, -W/2;
    -L/2, W/2;
    L/2, W/2];
end

function newPoints = transformPoints(points, T)
newPoints = T*[points';ones(1, size(points,1))];
newPoints = newPoints';
newPoints = newPoints(:, 1:2);
end