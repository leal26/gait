function SetDrawBody(x_o,y_o, ~, Body)


alpha = linspace(0, pi*2, 40);
vert_x_out = sin(alpha)*0.2+x_o;
vert_y_out = cos(alpha)*0.2+y_o;


set(Body.B_out,'xData',vert_x_out,'yData', vert_y_out);  


end