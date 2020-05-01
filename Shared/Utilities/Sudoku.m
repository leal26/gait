% *************************************************************************
%
% function SudokuVarOUT = Sudoku(SudokuVarIn)
%
% This matlab function solves Sudoku puzzels.
%
% Input:  - A 9x9 integer matrix representing the initial state of the
%           Sudoku. Unknown values are set to 0.  
% Output: - The final state of the Sudoku.
%
% Example:
%   SudokuVarIn = ...
%             [ 0 0 0 2 8 0 0 7 0;
%               0 0 0 7 0 1 0 4 0;
%               4 7 0 0 0 0 0 8 0;
%               9 0 0 0 2 0 8 0 0;
%               0 8 1 0 0 0 9 2 0;
%               0 0 5 0 9 0 0 0 4;
%               0 9 0 0 0 0 0 5 1;
%               0 3 0 8 0 5 0 0 0;
%               0 5 0 0 3 9 0 0 0];
%          
%   SudokuVarOUT = Sudoku(SudokuVarIn)
%
%
% NOTE:  This function only applies logical reasoning on possible and
%        necessary values for each row, column, and cell.  No recursive
%        search is performed if the outcome of this deduction is not
%        unique.  In this case an incomplete puzzle is returned.
%
%
% Created by C. David Remy on 01/15/2011
% Matlab 2010a 
%
% Documentation:
%  'A MATLAB Framework For Gait Creation', 2011, C. David Remy (1), Keith
%  Buffinton (2), and Roland Siegwart (1),  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA 
%
% (1) Autonomous Systems Lab, Institute of Robotics and Intelligent Systems, 
%     Swiss Federal Institute of Technology (ETHZ) 
%     Tannenstr. 3 / CLA-E-32.1
%     8092 Zurich, Switzerland  
%     cremy@ethz.ch; rsiegwart@ethz.ch
%
% (2) Department of Mechanical Engineering, 
%     Bucknell University
%     701 Moore Avenue
%     Lewisburg, PA-17837, USA
%     buffintk@bucknell.edu
%
function SudokuVarOUT = Sudoku(SudokuVarIn)

    if ~exist('SudokuVarIn','var')
        SudokuVar = ...
            [ 0 0 0 2 8 0 0 7 0;
              0 0 0 7 0 1 0 4 0;
              4 7 0 0 0 0 0 8 0;
              9 0 0 0 2 0 8 0 0;
              0 8 1 0 0 0 9 2 0;
              0 0 5 0 9 0 0 0 4;
              0 9 0 0 0 0 0 5 1;
              0 3 0 8 0 5 0 0 0;
              0 5 0 0 3 9 0 0 0];
    else
        SudokuVar = SudokuVarIn;
    end

    % This is used to check if we made progress in one iteration:
    oldSudokuVar = zeros(9);
    
    % Check for progress:
    while any(any((oldSudokuVar-SudokuVar)~=0))
        oldSudokuVar = SudokuVar;
        % Check if every entry was set:
        if sum(sum(SudokuVar>0)) == 81
            disp('**********************************')
            disp('*********** SOLVED ***************')
            disp(SudokuVar)
            disp('**********************************')
            disp('**********************************')
            SudokuVarOUT = SudokuVar;
            return
        end
        
        % Check wich numbers are still possible:
        % Each matrix of the 9 9x9 matrices defined in this multidim-array
        % represents a number.  Logical 1's indicate that this number
        % doesn't conflict with any of the previous entries, and the Sudoku
        % rules:
        possible = ones(9,9,9);
        for i = 1:9 % Sweep all columns:
            for j = 1:9 % Sweep all rows:
                for k = 1:9 % Sweep all numbers:
                    % Check if field is already occupied by another number:
                    if SudokuVar(i,j) ~= k 
                        possible(i,j,k) = 0;
                    end
                    % Check if number was already used in this column:
                    if any(SudokuVar(:,j) == k)
                        possible(i,j,k) = 0;
                    end
                    % Check if number was already used in this row:
                    if any(SudokuVar(i,:) == k)
                        possible(i,j,k) = 0;
                    end
                    % Check if number was already used in this cell:
                    i_ = floor((i-1)/3)+1;
                    j_ = floor((j-1)/3)+1;
                    if any(any(SudokuVar(i_*3-2:i_*3,j_*3-2:j_*3) == k))
                        possible(i,j,k) = 0;
                    end
                end
            end
        end

        % Update Sudoku:
        for i = 1:9 % Sweep all columns:
            for j = 1:9 % Sweep all rows:
                % If an entry is unambiguous, enter it in the updated
                % sudoku:
                if length(find(possible(i,j,:))) == 1
                    SudokuVar(i,j) = find(possible(i,j,:));
                end 
                for k = 1:9 % Sweep all numbers:
                    % If this is the only entry in the row, column or box,
                    % updated sudoku:
                    % Check column:
                    if possible(i,j,k) && sum(possible(:,j,k)) == 1
                        SudokuVar(i,j) = k;
                    end
                    % Check row:
                    if possible(i,j,k) && sum(possible(i,:,k)) == 1
                        SudokuVar(i,j) = k;
                    end
                    % Check box:
                    i_ = floor((i-1)/3)+1;
                    j_ = floor((j-1)/3)+1;
                    if possible(i,j,k) && sum(sum(possible(i_*3-2:i_*3,j_*3-2:j_*3,k))) == 1
                        SudokuVar(i,j) = k;
                    end
                end
                % Check for contradiction:
                if ~any(possible(i,j,:)) && SudokuVar(i,j) == 0
                    disp('Contradiction')
                    return
                end
            end
        end
    end
    SudokuVarOUT = SudokuVar;
end
% *************************************************************************
% *************************************************************************