function status = Tracker(t,y,flag)
% OutputFcn sample
persistent counter
switch flag
    case 'init'
        counter = 2;
    case ''
        counter = counter + 1;
    case 'done' % when it's done
        assignin('base','counter',counter); % get the data to the workspace.
end
status = 0;
