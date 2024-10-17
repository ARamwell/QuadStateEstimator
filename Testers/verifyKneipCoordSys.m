function verifyKneipCoordSys(T_nu_tau, T_W_eta)
    % %% Do some plotting for verification
    KneipPlotFig = figure();
    KneipPlotAx = axes('Parent', KneipPlotFig);
    set(KneipPlotAx, 'DataAspectRatio', [1 1 1], 'View', [37.5 30]);
    xlim(KneipPlotAx, [-1, 2]);
    ylim(KneipPlotAx, [-1, 2]);
    zlim(KneipPlotAx, [-1, 2]);
    xlabel(KneipPlotAx, 'X_w');
    ylabel(KneipPlotAx, 'Y_w');
    zlabel(KneipPlotAx, 'Z_w');

    %Plot camera frame nu at random location
    R_cam_plot = [-1 0 0; 0 0 -1; 0 -1 0];
    plotframe(R_cam_plot, [1; 1; 1], 'LabelBasis', true, ...
    'Labels', {'X_{cam}','Y_{cam}','Z_{cam}'}, 'Parent', KneipPlotAx);
    hold (KneipPlotAx, 'on');

    %Plot intermed cam frame tau
    R_tau_plot = T_nu_tau * R_cam_plot;
    plotframe(R_tau_plot, [1; 1; 1], 'LabelBasis', true, ...
    'Labels', {'X_{tau}','Y_{tau}','Z_{tau}'}, 'Parent', KneipPlotAx);

    %Plot world frame
    plotframe('LabelBasis', true, ...
    'Labels', {'X_{W}','Y_{W}','Z_{W}'}, 'Parent', KneipPlotAx);

    %Plot intermed frame eta
    plotframe(T_W_eta, [0; 0; 0], 'LabelBasis', true, ...
    'Labels', {'X_{eta}','Y_{eta}','Z_{eta}'}, 'Parent', KneipPlotAx);


end
