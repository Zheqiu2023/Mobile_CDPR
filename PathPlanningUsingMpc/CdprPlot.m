function CdprPlot(enable_obs, initialPose, targetPose, params, info, XY0)
% Animate automated parking of a CDPR system.

% plot initial poses
f = figure('NumberTitle','off');
xlabel('x');
ylabel('y');
plot(initialPose(1), initialPose(2),'rx');
hold on;
plot(targetPose(1), targetPose(2),'gx');
% set bounds
xbound = [-7 7];
ybound = [-7 7];
xlim(xbound);
ylim(ybound);
% plot obstacles
if enable_obs
    boxes = struct('Width',{0.5},'Height',{0.5},...
        'Pos',{[0;0]},'Theta',{0});
    for j = 1:length(boxes)
        obsStruct = boxes(j);
        obs = collisionBox(obsStruct.Width,obsStruct.Height,0);
        obs.Pose(1:3,1:3) = eul2rotm([obsStruct.Theta 0 0]);
        obs.Pose(1:2,4) = obsStruct.Pos;
        show(obs)
    end
end

if nargin<=5
    f.Name = 'Parking Lot with Initial and Target Positions';
    animateCdpr(gca, 0, initialPose', params);
else
    f.Name = ['Automated Parking Animation (x0 = ' num2str(initialPose(1)) ', y0 = ' num2str(initialPose(2)) ')'];
    % obtain optimal trajectory designed by MPC
    xTrackHistory = info.Xopt;
    uTrackHistory = info.MVopt;
    tTrackHistory = info.Topt;
    % plot the initial guess of XY trajectory
    plot(XY0(:,1),XY0(:,2),'c.')
    % plot the optimal XY trajectory by MPC
    plot(xTrackHistory(:,1),xTrackHistory(:,2),'bo')
    % animate track trailer moves
    animateCdpr(gca, tTrackHistory, xTrackHistory, params);
    %
    if enable_obs
        legend('','','','Initial Guess','Optimal Path');
        saveas(gcf, strcat("obs", string(datetime, 'MM-dd-HH-mm'), '].fig'));
    else
        legend('','','Initial Guess','Optimal Path');
        saveas(gcf, strcat("no_obs", string(datetime, 'MM-dd-HH-mm'), '].fig'));
    end
    % plot optimal MV and states
    figure('NumberTitle','off','Name','Optimal Trajectory of MVs and States');
    subplot(4,2,1)
    plot(tTrackHistory(1:end-1),uTrackHistory((1:end-1),1),'g.');title('$a(m/s^2)$', 'interpreter', 'latex');grid on;
    subplot(4,2,2)
    plot(tTrackHistory(1:end-1),uTrackHistory((1:end-1),2),'g.');title('w (rad/s)');grid on;
    subplot(4,2,3)
    plot(tTrackHistory,xTrackHistory(:,1),'g.');title('x (m)');grid on;
    subplot(4,2,4)
    plot(tTrackHistory,xTrackHistory(:,2),'g.');title('y (m)');grid on;
    subplot(4,2,5)
    plot(tTrackHistory,xTrackHistory(:,3),'g.');title('\theta (rad)');grid on;
    subplot(4,2,6)
    plot(tTrackHistory,xTrackHistory(:,4),'g.');title('v (m/s)');grid on;
    subplot(4,2,7)
    plot(tTrackHistory,xTrackHistory(:,5),'g.');title('\alpha (rad)');grid on;

    if enable_obs
        saveas(gcf, strcat("obs", string(datetime, 'MM-dd-HH-mm'), '.fig'));
    else
        saveas(gcf, strcat("no_obs", string(datetime, 'MM-dd-HH-mm'), '.fig'));
    end
end


function animateCdpr(ax, tOut, qOut, params)
% Playback CDPR motions

tSample = tOut;
xSample = qOut(:,1);
ySample = qOut(:,2);
thSample = qOut(:,3);
alphaSample = qOut(:,5);

W1 = params.W;
L1 = params.L;
T0Cdpr = [1 0 L1/2;
    0 1 0;
    0 0 1];
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

for i = 1:length(tSample)

    x = xSample(i);
    y = ySample(i);
    th = thSample(i);
    alpha = alphaSample(i);

    % Draw CDPR
    TCdpr = [cos(th) -sin(th) x;
        sin(th) cos(th) y;
        0 0 1];
    pts = transformPoints(cdprPoints, TCdpr);
    hCdpr.XData = pts(:,1);
    hCdpr.YData = pts(:,2);

    % Draw CDPR wheels
    TWheel = [cos(alpha) -sin(alpha) 0;
        sin(alpha) cos(alpha)  0;
        0 0 1];
    TOffsetFL = [1 0 L1; 0 1 W1/2; 0 0 1];
    TOffsetFR = [1 0 L1; 0 1 -W1/2; 0 0 1];
    TOffsetRL = [1 0 0; 0 1 W1/2; 0 0 1];
    TOffsetRR = [1 0 0; 0 1 -W1/2; 0 0 1];
    pts = transformPoints(wheelPoints, TCdpr*TOffsetFL*TWheel);
    hWheelFL.XData = pts(:,1);
    hWheelFL.YData = pts(:,2);
    pts = transformPoints(wheelPoints, TCdpr*TOffsetFR*TWheel);
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

    hCdpr.Visible = 'on';
    hWheelFL.Visible = 'on';
    hWheelFR.Visible = 'on';
    hWheelRL.Visible = 'on';
    hWheelRR.Visible = 'on';
    hPoint.Visible = 'on';

    % pause for animation
    %drawnow;
    pause(0.1)

end

function points = getRectangleVertices(L, W)
%getRectangleVertices L - along x, W - along y
points = [L/2, W/2;
    L/2, -W/2;
    -L/2, -W/2;
    -L/2, W/2;
    L/2, W/2];

function newPoints = transformPoints(points, T)
%TRANSFORMPOINTS
newPoints = T*[points';ones(1, size(points,1))];
newPoints = newPoints';
newPoints = newPoints(:, 1:2);

