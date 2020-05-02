function ALPContinuation(configurations,idex, ALPstep, FileName, yANALYS,zANALYS,yPERIOD,zPERIOD, yOPTIM,zOPTIM,pOPTIM)


% starting point in config
yCYC_OLDS1 = configurations{idex}.yCYC;
yCYC_OLDS2 = configurations{idex-1}.yCYC;


yCYC = configurations{idex}.yCYC;
zCYC = configurations{idex}.zCYC;
pCYC = configurations{idex}.pCYC;

[~, ~, contStateIndices] = ContStateDefinition();
[~, ~, discStateIndices] = DiscStateDefinition();


yINIT = yCYC;
zINIT = zCYC;

% Search for solutions in eigenvector direction with arc length parametrization:
solveOptions.tMAX = 5;

sG = -1; % +1 same direction; -1 the opposite direction 
[eigenValuesCYC, eigenVectorsCYC, ~, ~] =  FloquetAnalysisConstrainted(@HybridDynamics, yCYC,    zCYC,   pCYC,  yANALYS, zANALYS);

% Calculate changes in other perturbed states based on the defined stepSize 
StepALP = ALPstep*eigenVectorsCYC(:,abs(eigenValuesCYC-1)<=0.05);

figure(5)
box on; hold on ;  grid on; grid minor;
xlabel('$\dot{x}  [\sqrt{gl}]$','Interpreter','LaTex')
ylabel('$\dot{y}  [\sqrt{gl}]$','Interpreter','LaTex')
zlabel('$\phi_R$   $[rad]$','Interpreter','LaTex')
DisAxis = ALPstep*10;
axis([yCYC(2)-DisAxis yCYC(2)+DisAxis yCYC(4)-DisAxis yCYC(4)+DisAxis yCYC(9)-DisAxis yCYC(9)+DisAxis])
axis square;
view([45 45]);
% view([0 90]);
scatter3(yCYC(2),yCYC(4),yCYC(9),'k','filled');

OldVec = [0 0 0 0];
% Initial guess for the periodic search;
InGu = scatter3(yINIT(2),yINIT(4),yINIT(9),'b','filled');
% Boundary for the periodic constraint
[spx, spy, spz] = sphere;
spx = ALPstep*spx;
spy = ALPstep*spy;
spz = ALPstep*spz;
sp = surfl(spx + yCYC_OLDS1(2), spy + yCYC_OLDS1(4), spz + yCYC_OLDS1(9)); 
shading interp;
set(sp, 'FaceAlpha', 0.1)    
% Previous direction of motion on the curve;
Ldir = quiver3(yCYC_OLDS1(2), yCYC_OLDS1(4), yCYC_OLDS1(9),...
               OldVec(1),   OldVec(2),   OldVec(4),0,'color','k'); 
Sdir = quiver3(yCYC(2), yCYC(4), yCYC(9),...
               StepALP(1),  StepALP(2),  StepALP(4),0,'color','r');      

% Prepare color for eigenvectors:
eigenCol = [0  1  0;  % First Direction green
            1  0  1;  % Second Direction magenta
            0  0  0]; % Third Direction black

if idex == 1
    iss = 1;
else
    iss = idex +1;
end    
        
for i = iss:120

    disp(['Current Iteration: ', num2str(i)]);
    yCYC_=yCYC;
    zCYC_=zCYC;
    
    % Check all the eigenvalues which are close to 1 ;
    eVecIndex = find (abs(eigenValuesCYC-1)<=0.05); 
    % If there are more than one eigenvalues like this; then choose one:
    if length(eVecIndex)>1
       while 1 
             prompt = 'Multiple directions detected, which direction do you want to go? \n (green type 1, magenta type 2)';
             disp(eigenVectorsCYC);
             disp('______________');
             disp(abs(eigenValuesCYC-1)<=0.1);
             % Show all the assoiciated eigenvectors:
             pvh = gobjects(length(eVecIndex),1);
            
             for j = 1:length(eVecIndex)
                  
                pvj = ALPstep* [eigenVectorsCYC(1,eVecIndex(j)),... % dx
                                eigenVectorsCYC(2,eVecIndex(j)),... % dy
                                eigenVectorsCYC(4,eVecIndex(j))];   % phiR
                   
                pvh(j,1) = quiver3(yCYC(2),yCYC(4),yCYC(9),...
                                 pvj(1),pvj(2),pvj(3),0,...
                                 'Color', eigenCol(j,:));                
             end
             
             drawnow();
             
             % Choose the direction manully:
             x = input(prompt);
             
             if (x>0)&&(x<=length(eVecIndex)) 
                eVecIndexD = eVecIndex(x);
                % Selection is done, delete all the arrows;
                delete(pvh);  
                break;
             end
             disp('Wrong direction, enter again!');
       end 
    % If there is only one eigenvalue of this kind, just use it. 
    else   
       eVecIndexD = eVecIndex;
    end


    % Check eigenvector direction to prevent the search going backwards:
    if i == 3
            StepALP =  sG*ALPstep*eigenVectorsCYC(:,eVecIndexD);
            disp(StepALP);
    else
        hold on
        scatter3(yCYC(2),yCYC(4),yCYC(9),'k','filled');
        
        OldVec = yCYC(yANALYS==1) - yCYC_OLDS1(yANALYS==1);
        
        if dot(OldVec,eigenVectorsCYC(:,eVecIndexD))< -1e-3 % samilar direction   
           StepALP = - ALPstep*eigenVectorsCYC(:,eVecIndexD);
        else % opposite direction
           StepALP = + ALPstep*eigenVectorsCYC(:,eVecIndexD);
        end 
        
        set(sp,'XData',spx + yCYC(2),...
               'YData',spy + yCYC(4),...
               'ZData',spz + yCYC(9)); 
           
        if length(OldVec) == 4 % For Double stance phase
         OVDirPhiR = OldVec(4);   
         SADirPhiR = StepALP(4);
        else 
         OVDirPhiR = OldVec(5); % For single stance phase  
         SADirPhiR = StepALP(5);  
        end    
        
        set(Ldir,'XData', real(yCYC_OLDS1(2)),...
                 'YData', real(yCYC_OLDS1(4)),...
                 'ZData', real(yCYC_OLDS1(9)),...
                 'UData', real(OldVec(1)),...
                 'VData', real(OldVec(2)),...
                 'WData', real(OVDirPhiR));    
        set(Sdir,'XData', real(yCYC(2)),...
                 'YData', real(yCYC(4)),...
                 'ZData', real(yCYC(9)),...
                 'UData', real(StepALP(1)),...
                 'VData', real(StepALP(2)),...
                 'WData', real(SADirPhiR));        
        
        axis(real([yCYC(2)-DisAxis yCYC(2)+DisAxis yCYC(4)-DisAxis yCYC(4)+DisAxis yCYC(9)-DisAxis yCYC(9)+DisAxis]));                      
 
    end
    
    yCYC_(yANALYS==1) = yCYC(yANALYS==1) + StepALP;
    
    % Modify all the other constrainted states
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    zCYC_(discStateIndices.rcontPt) = 1*sin(yCYC_(contStateIndices.phiR)); 
    yCYC_(contStateIndices.dphiR)   = - yCYC_(contStateIndices.dx)*cos(yCYC_(contStateIndices.phiR))/1 ...
                                      - yCYC_(contStateIndices.dy)*sin(yCYC_(contStateIndices.phiR))/1; 

    % update vertical position y            
    yCYC_(contStateIndices.y) = cos(yCYC_(contStateIndices.phiR))*1; 

    if zCYC_(discStateIndices.lphase) == 2 % if left leg is in stance
        % rest left leg contact position, leg length, and leg
        % rotational velocity;
        zCYC_(discStateIndices.lcontPt) =  yCYC_(contStateIndices.y)*tan(yCYC_(contStateIndices.phiL));
        LegL                            =  yCYC_(contStateIndices.y)/cos(yCYC_(contStateIndices.phiL));
        yCYC_(contStateIndices.dphiL)   = -yCYC_(contStateIndices.dx)*cos(yCYC_(contStateIndices.phiL))/LegL ...
                                          -yCYC_(contStateIndices.dy)*sin(yCYC_(contStateIndices.phiL))/LegL;
    end
    
    % Done with manipulation of initial guess, save it for next iteration:
    yINIT=yCYC_;
    zINIT=zCYC_;
    pINIT=pCYC; 
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Graphic output of current progress
     if i ~= 1 % Initialize everything for the first iteration
        % Eigenvector direction is chosen, draw initial guess; 
        set(InGu,'XData',yINIT(2), 'YData',yINIT(4),'ZData',yINIT(9)) ; 
        drawnow();
     end
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Update periodic solution and ready for next point
    yCYC_OLDS2 = yCYC_OLDS1; % This needs to go first before next line;
    yCYC_OLDS1 = yCYC;  
    
    for attemptNum = 1:10
    % An upper limit for the stride duration is set, such that the simulation
    % will be aborted if the terminal state is never reached.  In this case, an
    % error message will be created:  
    % Call the root-search function.
    
        if attemptNum == 1
          yINIT_d = yINIT; 
        else % If previous initial guess doesn't converge, try adding some perturbances;
          DisTb = attemptNum*1e-3;  
          yINIT_d(yANALYS==1) =  yINIT(yANALYS==1) -DisTb + 2*DisTb*rand(5,1);
        end
        
        [yTry, zTry, pTry] =  FindPeriodicSolutionALP(@HybridDynamics, yINIT_d,   zINIT,  pINIT,... 
                                                                       yOPTIM,  zOPTIM, pOPTIM,... 
                                                                       yPERIOD, zPERIOD,...
                                                                       yCYC_OLDS1, ALPstep,...
                                                                       solveOptions);

        % If the returned solution is not what we previous have                                                        
        if norm(yTry([2,4,7,9]) - yCYC_OLDS2([2,4,7,9])) >  ALPstep*0.05
           yCYC = yTry;
           zCYC = zTry;
           pCYC = pTry;
           break;                                                                                                         
        end        
        disp(attemptNum);
        disp('Solution goes back to previous values. Attemp to perturb the initial conditions.')
    end
    
    
    if attemptNum == 10
       warning('Attempted 10 times without finding a solution, root finder is aborted')
       break;
    end    
    
    [eigenValuesCYC, eigenVectorsCYC, ~, ~] =  FloquetAnalysisConstrainted(@HybridDynamics, yCYC,    zCYC,   pCYC,  yANALYS, zANALYS);
    % Save Data
    configurations{i}.yCYC = yCYC;
    configurations{i}.zCYC = zCYC;
    configurations{i}.pCYC = pCYC;
    configurations{i}.eigenV = eigenValuesCYC;
    disp(eigenVectorsCYC);
    save(FileName,'configurations'); 
    
end

%%
axis tight; grid off;
view([-45 45])

end

