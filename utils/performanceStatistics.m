%% Statistical performance analysis of MWMP

% Approaches = unconstrained, 
%              ws_unconstrained, 
%              constrained, 
%              ws_constrained,
%              stepped, 
%              mwmp

% Data
iterations = [12 6 32 23 26 6;
              18 12 20 19 18 12;
              14 20 29 27 45 54;
              10 9 90 0 42 58;
              12 12 69 80 12 12;
              7 10 43 7 28 18;
              32 28 0 0 37 28;
              39 28 0 0 0 0;
              39 36 0 0 0 59;
              30 26 0 0 30 26;
              0 0 0 0 0 0;
              7 6 75 59 7 6;
              25 13 61 43 64 52;
              11 8 9 7 11 8;
              48 20 78 70 111 76;
              5 5 0 0 15 5;
              6 5 0 0 0 5;%
              24 9 0 0 24 9;
              20 23 0 0 20 23;
              30 10 0 0 0 10;
              8 7 7 8 8 7];

time = [0.293521 0.152425 0.751435 0.596241 0.635314 0.154372;
        0.450745 0.309013 0.525398 0.501833 0.450264 0.307576;
        0.348141 0.504908 0.649391 0.625485 1.037530 1.253210;
        0.256582 0.233430 2.103410 0        1.013880 1.371170;
        0.303844 0.307956 1.607130 1.879060 0.300781 0.305766;
        0.806066 0.713462 0        0        0.946451 0.726067;
        0.178544 0.255735 0.938731 0.167552 0.652667 0.453369;
        0.983308 0.721036 0        0        0        0       ;
        0.994529 0.931128 0        0        0        1.586230;
        0.753288 0.661194 0        0        0.755974 0.667319;
        0        0        0        0        0        0       ;
        0.175674 0.153063 1.812210 1.279720 0.179709 0.153124;
        0.651720 0.329424 1.373920 0.962130 1.505300 1.275700;
        0.279012 0.209236 0.233518 0.166664 0.282387 0.211360;
        1.221590 0.512584 1.733200 1.583290 2.625480 1.766140;
        0.135724 0.141560 0        0        0.384090 0.137630;
        0.152332 0.130601 0        0        0        0.137479;%
        0.619451 0.236606 0        0        0.612531 0.240569;
        0.507683 0.592410 0        0        0.508233 0.592691;
        0.785698 0.265709 0        0        0        0.267795;
        0.201426 0.179384 0.161725 0.187838 0.201429 0.182009];

success = [1 1 1 1 1 1;
           1 1 1 1 1 1;
           1 1 1 1 1 1;
           1 1 1 0 1 1;
           1 1 1 1 1 1;
           1 1 1 1 1 1;
           1 1 0 0 1 1;
           1 1 0 0 0 0;
           1 1 0 0 0 1;
           1 1 0 0 1 1;
           0 0 0 0 0 0;
           1 1 1 1 1 1;
           1 1 1 1 1 1;
           1 1 1 1 1 1;
           1 1 1 1 1 1;
           1 1 0 0 1 1;
           1 1 0 0 0 1;%
           1 1 0 0 1 1;
           1 1 0 0 1 1;
           1 1 0 0 0 1;
           1 1 1 1 1 1];

feasibility = [0 1 1 1 1 1;
               1 1 1 1 1 1;
               0 0 1 1 1 1;
               0 0 1 0 1 1;
               1 1 1 1 1 1;
               0 0 1 1 1 1;
               0 1 0 0 1 1;
               0 0 0 0 0 0;
               0 0 0 0 0 1;
               1 1 0 0 1 1;
               0 0 0 0 0 0;
               1 1 1 1 1 1;
               0 0 1 1 1 1;
               1 1 1 1 1 1;
               0 0 1 1 1 1;
               0 1 0 0 1 1;
               0 1 0 0 0 1;%
               1 1 0 0 1 1;
               1 1 0 0 1 1;
               0 0 0 0 0 1;
               1 1 1 1 1 1];

torque = [9.6503 9.6501 9.6505 9.6502 9.6510 9.6501;
          9.6512 9.6510 9.6512 9.6511 9.6512 9.6510;
          9.6519 9.6514 9.6518 9.6514 9.6516 9.6515;
          9.6516 9.6517 9.6530 999    9.6528 9.6518;
          9.6507 9.6510 9.6522 9.6524 9.6507 9.6510;
          9.6512 9.6513 9.6531 9.6534 9.6534 9.6533;
          9.6529 9.6537 999    999    9.6546 9.6537;
          9.6474 9.6490 999    999    999    999   ;
          9.6563 9.6568 999    999    999    9.6569;
          9.6516 9.6523 999    999    9.6516 9.6523;
          999    999    999    999    999    999   ;
          9.6506 9.6519 9.6514 9.6525 9.6506 9.6519;
          9.6505 9.6502 9.6507 9.6504 9.6513 9.6502;
          9.6516 9.6517 9.6524 9.6530 9.6516 9.6517;
          9.6499 9.6501 9.6509 9.6506 9.6511 9.6515;
          9.6521 9.6523 999    999    9.6522 9.6523;
          9.6530 9.6530 999    999    999    9.6530;%
          9.6487 9.6487 999    999    9.6487 9.6487;
          9.6511 9.6509 999    999    9.6511 9.6509;
          9.6552 9.6505 999    999    999    9.6505;
          9.6510 9.6511 9.6529 9.6528 9.6510 9.6511] - 9.6;

% Results
number_success = [0 0 0 0 0 0];
number_feasible = [0 0 0 0 0 0];
iterations_sum = [0 0 0 0 0 0];
time_sum = [0 0 0 0 0 0];
quality = [0 0 0 0 0 0];

for i = 1:size(success,1)
    for j = 1:size(success,2)
        if(success(i,j))
            number_success(j) = number_success(j)+1;
            iterations_sum(j) = iterations_sum(j)+iterations(i,j);
            time_sum(j) = time_sum(j)+time(i,j);
            if(feasibility(i,j))
                number_feasible(j) = number_feasible(j)+1;
                quality(j) = quality(j) + ...
                    (1-(torque(i,j) - min(torque(i,:)))./min(torque(i,:)));
            end
        end
    end
end

disp(['Number of registered samples: ', num2str(size(success,1))])

avg_iterations = iterations_sum./number_success;

disp(['Average number iterations: ', num2str(avg_iterations,6)])
avg_iterations(4) = avg_iterations(4)+0.01;

iterations_std_dev = zeros(size(success,2),1);
for i = 1:size(success,1)
    for j = 1:size(success,2)
        if(success(i,j))
            iterations_std_dev(j) = iterations_std_dev(j) + (iterations(i,j) - avg_iterations(j))^2;
        end
    end
end

iterations_std_dev = sqrt(iterations_std_dev/(size(success,1)-1));

disp(['Standard deviation number iterations: ', num2str(iterations_std_dev.',6)])

avg_time = time_sum./number_success;

disp(['Average time spent:        ', num2str(avg_time,6)])

avg_success = [sum(success(:,1)) sum(success(:,2)) sum(success(:,3))...
               sum(success(:,4)) sum(success(:,5)) sum(success(:,6))]...
               /size(success,1);

disp(['Average success rate:      ', num2str(avg_success,6)])

avg_feasibility = [sum(feasibility(:,1)) sum(feasibility(:,2)) sum(feasibility(:,3))...
                   sum(feasibility(:,4)) sum(feasibility(:,5)) sum(feasibility(:,6))]...
                   /size(feasibility,1);

disp(['Average feasibility rate:  ', num2str(avg_feasibility,6)])

avg_quality = quality./number_feasible;

disp(['Average quality rate:      ', num2str(avg_quality,6)])


%% Plot feasability graph
h = figure('Name', 'Feasability graph', 'DefaultAxesFontSize', 25);
black = [0 0 0];
set(h,'defaultAxesColorOrder',[black; black]);

% Bars for the success rates
xStep = 1;
x = [xStep 2*xStep 3*xStep 4*xStep 5*xStep 6*xStep];
y = [avg_feasibility.' (avg_success.' - avg_feasibility.')].*100;
b = bar(x, y, 0.1, 'stacked', 'XOffset', 20);
b(1).FaceColor = [0/255 0/255 200/255];
b(2).FaceColor = [235/255 10/255 10/255];

% Figure final details
legend('Feasibility ratio', 'Success ratio', ...
       'interpreter', 'latex','fontsize',25)
xticklabels(["USLQ", "PPWS+USLQ", "CSLQ", "PPWS+CSLQ", "USLQ+CSLQ", "MWMP"])
ylabel('Percentage (\%)', 'interpreter', 'latex', 'Color', 'black', 'fontsize', 30)
% xlabel('Stages', 'interpreter', 'latex','fontsize',30)
grid
axis([0 7 0 100])

% Save the figure
% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'feasibility_graph','-dpdf','-r0')

%% Plot iterations graph
h2 = figure('Name', 'Iterations graph', 'DefaultAxesFontSize', 25);
black = [0 0 0];
set(h2,'defaultAxesColorOrder',[black; black]);

% Error bars for number of iterations
xStep = 1;
x = [xStep 2*xStep 3*xStep 4*xStep 5*xStep 6*xStep];
eb = errorbar(x,avg_iterations,iterations_std_dev,'s','MarkerSize',20,...
         'MarkerFaceColor',[255/255 165/255 0],...
         'MarkerEdgeColor',[0/255 0/255 0], 'Color', [0/255 0/255 0],...
         'CapSize',25, 'LineWidth', 3);
hold on;

% Figure final details
ylabel('Number of iterations', 'interpreter', 'latex', 'fontsize', 30)
% xlabel('Stages', 'interpreter', 'latex','fontsize',30)
grid
axis([0.0 7 0 70])
xticks(x)
xticklabels(["USLQ", "PPWS+USLQ", "CSLQ", "PPWS+CSLQ", "USLQ+CSLQ", "MWMP"])

yyaxis right
axis([0.0 7 0 1750])
ylabel('Time (ms)', 'interpreter', 'latex', 'fontsize', 30)

% Save the figure
% set(h2,'Units','Inches');
% pos = get(h2,'Position');
% set(h2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h2,'iterations_graph','-dpdf','-r0')


