function plot_map()
    map = get_map();
    scatter(map(:,1),map(:,2),200,'r+');
    hold on