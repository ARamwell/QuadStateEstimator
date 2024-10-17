function sim_MapInit(wallList, checkerList)
%SIM_INIT Summary of this function goes here
%   Detailed explanation goes here

    world =sim3d.World();

    %build walls
    for i=1:length(wallList)
        wall_i = wallList(i); 
        ActObj = wall_i.Actor;
        get()
        add(world, ActObj);
    end







end

