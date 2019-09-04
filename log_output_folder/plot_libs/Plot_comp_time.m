close all
clear
clc

fig = figure('pos',[10 10 1900 1000]); 
addpath('export_fig');
data = load('../log_output_0.txt'); hold on; 
axis([0 12.5 2000 14500])

ros_time     = data(:,1) - data(1,1);
solve_time   = data(:,5);

color_blue = [36, 143, 149];
color_lblue = [142, 199, 240];
color_green = [91, 189, 91];
color_lgreen = [199, 240, 199];
color_red = [242,70,65];
color_lred = [240, 191, 191];

%% Computational Time Drawing

x2 = [ros_time(1:50)', fliplr(ros_time(1:50)')];
inBetween = [solve_time(1:50)'- 1200, fliplr(solve_time(1:50)' + 1200)];
fill(x2, inBetween, color_lblue/256, 'EdgeColor','none');
plot(ros_time(1:50), solve_time(1:50), 'color', color_blue/256, "LineWidth", 2);



x2 = [ros_time(51:551)', fliplr(ros_time(51:551)')];
inBetween = [solve_time(51:551)'- 900, fliplr(solve_time(51:551)' + 900)];
fill(x2, inBetween, color_lgreen/256, 'EdgeColor','none');
plot(ros_time(51:551), solve_time(51:551), 'color', color_green/256, "LineWidth", 2);


x2 = [ros_time(552:565)', fliplr(ros_time(552:565)')];
inBetween = [solve_time(552:565)'- 4000, fliplr(solve_time(552:565)' + 4000)];
fill(x2, inBetween, color_lred/256, 'EdgeColor','none');
plot(ros_time(552:565), solve_time(552:565), 'color', color_red/256, "LineWidth", 2);

x2 = [ros_time(566:759)', fliplr(ros_time(566:759)')];
inBetween = [solve_time(566:759)'- 900, fliplr(solve_time(566:759)' + 900)];
fill(x2, inBetween, color_lgreen/256, 'EdgeColor','none');
plot(ros_time(566:759), solve_time(566:759), 'color', color_green/256, "LineWidth", 2);

grid minor;
xlabel('time [s]');
ylabel('Computational Time[\mus]');
set(gca,'YDir','normal')
set(gca,'FontSize',20);
set(gca,'LineWidth',1);
set(gca,'MinorGridLineStyle', '-');
export_fig(['compTime','.pdf'], '-pdf');

% x2 = [ros_time(760:770)', fliplr(ros_time(760:770)')];
% inBetween = [solve_time(760:770)'- 1000, fliplr(solve_time(760:770)' + 1000)];
% fill(x2, inBetween, color_lred/256, 'EdgeColor','none');
% plot(ros_time(760:770), solve_time(760:770), 'color', color_red/256, "LineWidth", 1);
% 
% x2 = [ros_time(771:910)', fliplr(ros_time(771:910)')];
% inBetween = [solve_time(771:910)'- 400, fliplr(solve_time(771:910)' + 400)];
% fill(x2, inBetween, color_lblue/256, 'EdgeColor','none');
% plot(ros_time(771:910), solve_time(771:910), 'color', color_blue/256, "LineWidth", 1);
% 
% x2 = [ros_time(911:end)', fliplr(ros_time(911:end)')];
% inBetween = [solve_time(911:end)'-600, fliplr(solve_time(911:end)' + 600)];
% fill(x2, inBetween, color_lgreen/256, 'EdgeColor','none');
% plot(ros_time(911:end), solve_time(911:end), 'color', color_green/256, "LineWidth", 1);
