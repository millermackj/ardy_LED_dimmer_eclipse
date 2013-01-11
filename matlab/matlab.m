blub_x = (0:100);
magnitude = 2;
blub_y = -magnitude*(1/2)*((cos((pi/length(blub_x))*blub_x))-0.5);
plot(blub_x,blub_y)