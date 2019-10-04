function  ShowTrajectory( T,Y,P ,FileName)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
StrideMoiveName = FileName;

RecordStrideVideo = 1;
ShowVideo = 1;

% figure(101); clf; grid on; plot(T,Y,'-');
% legend('x','dx','y','dy','alphaL','dalphaL','alphaR','dalphaR');

tL_TD = P(1);
tL_LO = P(2);
tR_TD = P(3);
tR_LO = P(4);
tAPEX = P(5);
k     = P(6);

if tL_TD < 0
     tL_TD = tL_TD + tAPEX;
end
if tL_TD > tAPEX
     tL_TD = tL_TD - tAPEX;
end 
if tL_LO < 0
     tL_LO = tL_LO + tAPEX;
end
if tL_LO > tAPEX
     tL_LO = tL_LO - tAPEX;
end 
if tR_TD < 0
     tR_TD = tR_TD + tAPEX;
end
if tR_TD > tAPEX
     tR_TD = tR_TD - tAPEX;
end 
if tR_LO < 0
     tR_LO = tR_LO + tAPEX;
end
if tR_LO > tAPEX
     tR_LO = tR_LO - tAPEX;
end

n = length(T);
FLy = zeros(n,1);
FRy = zeros(n,1);

    for  i = 1:n    
        % Figure out the current contact configuration (this is used in the
        % dynamics function)
        t_ = T(i);
        if ((t_>tL_TD && t_<tL_LO && tL_TD<tL_LO) || ((t_<tL_LO || t_>tL_TD) && tL_TD>tL_LO))
            contactL = true;
        else
            contactL = false;
        end
        if ((t_>tR_TD && t_<tR_LO && tR_TD<tR_LO) || ((t_<tR_LO || t_>tR_TD) && tR_TD>tR_LO))
            contactR = true;
        else
            contactR = false;
        end      

        y_       = Y(i,:);
        x        = y_(1);
        dx       = y_(2);
        y        = y_(3);
        dy       = y_(4);
        alphaL   = y_(5);
        dalphaL  = y_(6);
        alphaR   = y_(7);
        dalphaR  = y_(8);

        % Compute forces acting on the main body (only legs in contact
        % contribute): 
        if contactL
            FLy(i) =  + (1- y/cos(alphaL))*k*cos(alphaL);
        end
        if contactR
            FRy(i) =  + (1- y/cos(alphaR))*k*cos(alphaR);
        end
    end
    
 figure(102); clf;  plot(T,FLy,T,FRy,'LineWidth',2); grid on;  
 xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
 ylabel('Vertical GRFs $[m_o g]$','Interpreter','LaTex')
 legend('Left leg','Right leg');  
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
% Make stride video for each event     
if ShowVideo == 1
    graphOUTPUT = SLIP_Model_Graphics_PointFeet;     
    tEvents = [tL_TD; tL_LO; tR_TD; tR_LO; tAPEX]; 
    n = round(T(end)*50); % # of frames per step
    tFrame = linspace(0, T(end), n+1);
    frameCount = 1;
    % If desired, every iteration a rendered picture is saved to disc.  This
    % can later be used to create a animation of the monopod.
    for j = 1:n
        y = interp1(T' + linspace(0,1e-5,length(T)), Y, tFrame(j))';
        graphOUTPUT.update(y,tEvents,tFrame(j));
        % (un)comment the following line, if you (don't) want to save the individual frames to disc:
        % fig = gcf;
        % print(fig,'-r200','-djpeg',['MovieFrames/Frame',num2str(frameCount,'%04d.jpg')],'-opengl');
        % print(fig,'-dpdf',['MovieFrames/Frame',num2str(frameCount,'%04d.pdf')]);
        frameCount = frameCount + 1;
    end

end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
% Make stride video for each event     
if RecordStrideVideo == 1
    graphOUTPUT = SLIP_Model_Graphics_PointFeet;     
    tEvents = [tL_TD; tL_LO; tR_TD; tR_LO; tAPEX]; 
    n = round(T(end)*100); % # of frames per step
    tFrame = linspace(0, T(end), n+1);
    frameCount = 1;
    v = VideoWriter(StrideMoiveName,'MPEG-4');
    v.Quality = 100;
    open(v);
    % If desired, every iteration a rendered picture is saved to disc.  This
    % can later be used to create a animation of the monopod.
    for j = 1:n
        y = interp1(T' + linspace(0,1e-5,length(T)), Y, tFrame(j))';
        graphOUTPUT.update(y,tEvents,tFrame(j));
        % (un)comment the following line, if you (don't) want to save the individual frames to disc:
        % fig = gcf;
        % print(fig,'-r200','-djpeg',['MovieFrames/Frame',num2str(frameCount,'%04d.jpg')],'-opengl');
        % print(fig,'-dpdf',['MovieFrames/Frame',num2str(frameCount,'%04d.pdf')]);
        frameCount = frameCount + 1;
        writeVideo(v,getframe(gcf));
    end
    close(v);
end
    
end

