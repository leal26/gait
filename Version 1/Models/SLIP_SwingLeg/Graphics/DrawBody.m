function  Body=DrawBody(x,y)


alpha = linspace(0, pi*2, 40);
vert_x_out = sin(alpha)*0.2+x;
vert_y_out = cos(alpha)*0.2+y;

b1=patch(vert_x_out, vert_y_out,'white','linewidth',5);         
 
Body = struct('B_out',b1);

axis equal

end