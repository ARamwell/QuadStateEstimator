
    function figHandle = formatFigForLatex(figHandle)

        %general
        picturewidth = 12;%25; % set this parameter and keep it forever
        hw_ratio = 0.7;%0.3%0.65 % feel free to play with this ratio
        set(findall(figHandle,'-property','LineWidth'),'LineWidth',1.5) % adjust fontsize to your document
        set(findall(figHandle,'-property','FontSize'),'FontSize',14)
        set(findall(figHandle,'-property','Box'),'Box','off') % optional
        set(findall(figHandle,'-property','Interpreter'),'Interpreter','latex') 
        set(findall(figHandle,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
        set(figHandle,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])
        pos = get(figHandle,'Position');
        set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on');
        set(gca, 'GridAlpha', 0.3);
        set(gca, 'GridLineWidth', 1.0);
        set(gca, 'TickLabelInterpreter', 'none');
        set(gca, 'LineWidth', 1.5);
        

        
        % 
        % xlabel("time (s)");
        % ylabel("absolute rotation error (degrees)")

        %set(figHandle,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
        %print(figHandle,'testfig','-dpdf','-painters','-fillpage')
        %print(hfig,fname,'-dpng','-painters')

    end