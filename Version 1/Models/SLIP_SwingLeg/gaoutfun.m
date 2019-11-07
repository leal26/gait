function [state, options,optchanged] = gaoutputfun(options,state,flag, filename)
optchanged = false;
switch flag
    case 'init'
        disp('Starting the algorithm');
    case {'iter','interrupt'}
        disp('Iterating ...')
        fname=[pwd,'\',num2str(state.Generation),'.mat'];
        save(fname,'state')
    case 'done'
        disp('Performing final task');
        fname=[pwd,'\',num2str(state.Generation),'.mat'];
        save(fname,'state')
end