function  parsaveDS(dyID, phiRID, phiLID, dphiLID,yINIT ,zINIT,pINIT)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
 save(['ExhaustiveSearch\solution',num2str(dyID),'_', ...
     num2str(phiRID),'_',num2str(phiLID),'_',num2str(dphiLID),'_DS','.mat'],...
     'yINIT','zINIT', 'pINIT','dyID', 'phiRID', 'phiLID', 'dphiLID');
end

