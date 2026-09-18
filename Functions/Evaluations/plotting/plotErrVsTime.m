function plotErrVsTime(figObj, t1, trajErr1, name1, t2, trajErr2, name2, t3, trajErr3, name3)

        figure(figObj);

        nexttile;
        plot(t1, trajErr1.AbsoluteError(:,2), 'LineWidth',1.5, 'DisplayName', name1);
        if exist('trajErr2', 'var')
            hold on;
            plot(t2, trajErr2.AbsoluteError(:,2), 'LineWidth',1.5, 'DisplayName', name2);
        end
        if exist('trajErr3', 'var')
            hold on;
            plot(t3, trajErr3.AbsoluteError(:,2), 'LineWidth',1.5, 'DisplayName', name3);
        end
        title('Position error of component state estimators (m)', 'FontSize', 14);
        xlabel('Time since initialisation (s)', 'FontSize', 12);
        ylabel('Absolute position error (m)', 'FontSize', 12);
        legend;
        hold off;

        %Rotation error
        nexttile;
        plot(t1, trajErr1.AbsoluteError(:,1), 'LineWidth',1.5, 'DisplayName', name1, 'Color', '#e41a1c');
        if exist('trajErr2', 'var')
            hold on;
            plot(t2, trajErr2.AbsoluteError(:,1), 'LineWidth',1.5, 'DisplayName', name2, 'Color', '#377eb8');
        end
        if exist('trajErr3', 'var')
            hold on;
            plot(t3, trajErr3.AbsoluteError(:,1), 'LineWidth',1.5, 'DisplayName', name3, 'Color', '#984ea3');
        end
        title('Rotation error of state estimators (degrees)', 'FontSize', 14);
        xlabel('Time since initialisation (s)', 'FontSize', 12);
        ylabel('Absolute rotation error (degrees)', 'FontSize', 12);
        legend;
        hold off;
        
    end