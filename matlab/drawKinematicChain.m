function [chain, rawmat] =drawKinematicChain(filename)
%DRAWKINEMATICCHAIN Imports a kinematic chain and draws it on the figure
%
% Example:
%   [chain, rawmat] =drawKinematicChain(filename);
%
    M_PI = pi;
    CTRL_DEG2RAD = pi/180;

%% Read stuff
    delimiter = {'numLinks','offset','alpha','link_','min','max','H0',' ','(',')','A','D','home'};
    startRow = 1;
    endRow = inf;

    % Read columns of data as strings: 
    formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

    % Open the text file.
    fileID = fopen(filename,'r');

    % Read columns of data according to format string.
    textscan(fileID, '%[^\n\r]', startRow(1)-1, 'ReturnOnError', false);
    dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'EmptyValue' ,NaN,'ReturnOnError', false);
    for block=2:length(startRow)
        frewind(fileID);
        textscan(fileID, '%[^\n\r]', startRow(block)-1, 'ReturnOnError', false);
        dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'EmptyValue' ,NaN,'ReturnOnError', false);
        for col=1:length(dataArray)
            dataArray{col} = [dataArray{col};dataArrayBlock{col}];
        end
    end

    % Close the text file.
    fclose(fileID);

    % Create output variable(s)
    disp(dataArray{7})
    disp(dataArray{8})
    rraw = [dataArray{1:end-1}];

    rawmat = rraw(4:end,2:8);

%% Draw the chain
    [o,n,e]=fileparts('../app/conf/DH_right.ini');
    ch.name = n(4:end);

    ch.DH = [rawmat(:,1:4)];
    ch.DH(:,3:4)=ch.DH(:,3:4).*CTRL_DEG2RAD;
    
    ch.H0 = [rraw(1,:)];
    ch.H0 = reshape(ch.H0,[4 4])';
    ch.H_0 = eye(4);

    ch.Th = rraw(3,1:rraw(2,1));
    ch.Th = ch.Th * CTRL_DEG2RAD;
    ch.LinkColor = rand(1,3);

    chain=FwdKin(ch);
