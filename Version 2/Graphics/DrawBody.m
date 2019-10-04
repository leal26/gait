function  Body=DrawBody(x,y)

l    = 1.2;
h   = 0.4; 
n=30;
m=10;

% T= [ cos(gamma_leg), -sin(gamma_leg);
%          sin(gamma_leg),  cos(gamma_leg)];
% 
% v = [                                                                                            
%      repmat([-l/2, h/2],n ,1) + [linspace(0,l,n)',zeros(n,1)];                                                                             
%      repmat([ l/2, h/2],m,1)  - [zeros(m,1),linspace(0,h,m)'];                                                                                           
%      repmat([ l/2,-h/2],n ,1)  - [linspace(0,l,n)',zeros(n,1)];                                                                                                                      
%      repmat([-l/2,-h/2],m,1) + [zeros(m,1),linspace(0,h,m)'];              
%       ]; 
% 
%             v_rot = T*v';% Rotate
%             vert_x= v_rot (1,:) +  x_o ;
%             vert_y =v_rot (2,:) +  y_o ;
%             
% v=[vert_x',vert_y']; 
% 
% f = [ 4,78;7,76;10,74;13,72;16,68;19,65;22,62;
%        25,59; 28,56;32,53; 34,50;36,47;38,44];
%    

%                 phi = linspace(0, pi/2, 10);
%                 vert_x = [0,sin(phi)*0.1,0];
%                 vert_x = [vert_x;vert_x;-vert_x;-vert_x]' ;
%                 vert_y = [0,cos(phi)*0.1,0];
%                 vert_y = [vert_y;-vert_y;-vert_y;vert_y]' ;
%                 
% %         Create graphic objects:
% %         The representation of the COG as patch object:
%         phi = linspace(0, pi/2, 10);
%         vert_x = [0,sin(phi)*0.1,0];
%         vert_x = [vert_x;vert_x;-vert_x;-vert_x]' + x;
%         vert_y = [0,cos(phi)*0.1,0];
%         vert_y = [vert_y;-vert_y;-vert_y;vert_y]' + y;
%       obj.COGPatch = patch(vert_x, vert_y, cat(3,[1 0 1 0], [1 0 1 0],[1 0 1 0]));   
        alpha = linspace(0, pi*2, 40);
        vert_x_out = sin(alpha)*0.2+x;
        vert_y_out = cos(alpha)*0.2+y;



b1=patch(vert_x_out, vert_y_out,'white','linewidth',5);         
% b2=patch('faces', f, 'vertices', v,'linewidth',3,'FaceColor',[1 1 1],'EdgeColor',0.8*[1 1 1]);
% b2=patch(vert_x, vert_y, cat(3,[1 0 1 0], [1 0 1 0],[1 0 1 0])); 
% % cdata=cat(3,[1 0 1 0], [1 0 1 0],[1 0 1 0]);
% cdata=[1 0 1 0;
%              1 0 1 0;
%              1 0 1 0]';
% b3=patch(vert_x, vert_y,cdata,'FaceColor','none');   
Body = struct('B_out',b1);
% Body = struct('B_out',b1,'B_sha',b2,'B_COG',b3);
axis equal
% axis([-0.7,0.7 -0.4 0.4])
end