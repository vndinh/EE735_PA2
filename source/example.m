filename = sprintf('../report/view3d/my_data/final_views.ply');
ptcloud = pcread(filename);
pcshow(ptcloud);
xlim([-10, 10]);
ylim([-10, 10]);
zlim([0, 10]);