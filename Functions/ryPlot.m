function ryPlot(traj1, endAxes, text1)
 xdata = [traj1.XData, 52]
 ydata = [traj1.YData, 53]
 zdata = [traj1.ZData, 36]
set(traj1, "XData", xdata, "YData", ydata, "ZData", zdata) %IF you want to see the properties of a graphical obj (or any obj for that matter), you can click F9 when break point (esentially run the handle), and then click show all properties in the command line!

xdata = [100, 130]
endAxes.XData = xdata

text1.Position = [100, 100, 100]

end

