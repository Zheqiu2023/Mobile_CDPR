fig = gcf;
axesHandles = flipud(findall(fig, 'Type', 'axes'));
data = struct;

for i = 1:length(axesHandles)
    lineHandles = findall(axesHandles(i), 'Type', 'line');
    for j = 1:length(lineHandles)
        data(i).line(j).XData = get(lineHandles(j), 'XData');
        data(i).line(j).YData = get(lineHandles(j), 'YData');
        data(i).line(j).LineStyle = get(lineHandles(j), 'LineStyle');
        data(i).line(j).Color = get(lineHandles(j), 'Color');
        data(i).line(j).LineWidth = get(lineHandles(j), 'LineWidth');
    end
end

% 重绘图形
figure;
set(gcf,'color','white');
for i = 1:length(data)
    subplot(2, 3, i);
    hold on;
    plot(data(i).line(2).XData, data(i).line(2).YData, 'b--', 'LineWidth',1.2);    
    plot(data(i).line(3).XData, data(i).line(3).YData, 'r-', 'LineWidth',1.2);
    
    if i <= 3
        ylabel('位置(m)', 'FontSize', 12);
        xlabel('时间(s)', 'FontSize', 12);xlim([0 270]);
        yyaxis right; % 设置右侧纵坐标轴
        set(gca, 'YColor', [0.7, 0.5, 0.3]);
        plot(data(i).line(1).XData, data(i).line(1).YData, '-.', 'LineWidth',1.2,'Color',[0.7, 0.5, 0.3]);
        ylabel('误差(m)');
        legend('期望位置','实际位置','位置误差','FontSize',10,'EdgeColor','none','Location','northeast');
    else
        ylabel('角度(°)', 'FontSize', 12);
        xlabel('时间(s)', 'FontSize', 12);xlim([0 270]);
        yyaxis right; % 设置右侧纵坐标轴
        set(gca, 'YColor', [0.7, 0.5, 0.3]);
        plot(data(i).line(1).XData, data(i).line(1).YData, 'y-.', 'LineWidth',1.2,'Color',[0.7, 0.5, 0.3]);
        ylabel('误差(°)');
        legend('期望角度','实际角度','角度误差','FontSize',10,'EdgeColor','none','Location','northeast');
    end

    hold off;
    set(gca, 'Box', 'on');
    if i == 1
        title('x轴位置', 'FontSize', 12);
    elseif i == 2
        title('y轴位置', 'FontSize', 12);
    elseif i == 3
        title('z轴位置', 'FontSize', 12);
    elseif i == 4
        title('x轴姿态', 'FontSize', 12);
    elseif i == 5
        title('y轴姿态', 'FontSize', 12);
    elseif i == 6
        title('z轴姿态', 'FontSize', 12);
    end
end

figure;
set(gcf,'color','white');
plot3(data(1).line(3).YData, data(2).line(3).YData, data(3).line(3).YData, 'r-','LineWidth',2);hold on;
plot3(data(1).line(2).YData, data(2).line(2).YData, data(3).line(2).YData, 'b--','LineWidth',2);
xlabel('X(m)', 'FontSize', 13);
ylabel('Y(m)', 'FontSize', 13);
zlabel('Z(m)', 'FontSize', 13);
axis([-0.5 0.5 -0.5 0.5 0 0.7]);
legend('期望轨迹','实际轨迹','FontSize',13,'EdgeColor','none','Location','northeast');
