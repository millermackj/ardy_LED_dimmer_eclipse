blub_x = (0:100);
magnitude = 3;
blub_y = -magnitude * (1/2)*(cos((pi/length(blub_x))*blub_x) - 1);
plot(blub_x,blub_y)