function [reachedPts,hgroup] = drawWorkspace(varargin)
% Draw the workspace from a text file
% Function for importing data from a proper output.ini file
%
%   [reachedPts,filename] = drawWorkspace(varargin)
%
% INPUT:
%   varargin{1} -> output.ini file to be used for the drawing
%   varargin{2} -> flag to know if to draw either a set of isosurfaces (true) or points (false)
%   varargin{3} -> flag to know if to record a video or not. If true, it records an iCubWorkspace.avi file.
%
% OUTPUT:
%   reachedPtS  -> an Nx4 array of reached 3D points + their manipulability index.
%                  It has been ordered according to the magnitude of the manipulability (from low to high)
%   hgroup      -> handle for the surfaces that has been drawn

    %% Initialize variables according to input arguments

        % filename = '~/.local/share/yarp/contexts/iCubWorkspace/output.ini';
        % filename = '../app/conf/output.ini';
        filename = '../app/conf/output_right.ini';
        if nargin>0
            filename=varargin{1};
        end
        disp('Varargin:');
        disp(varargin);

        drawSurfaces=true;
        if nargin>1
            drawSurfaces=varargin{2};
        end

        videoOn=false;
        if nargin>2
            videoOn=varargin{3};
        end

        % Skip the first five lines of the output file
        startRow=8;

    %% Process text file
        delimiter = ' ';
        % Format string for each line of text:
        formatSpec = '%f%f%f%f%[^\n\r]';

        % Open the text file.
        fileID = fopen(filename,'r');
        dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,'HeaderLines' ,startRow-1,  'ReturnOnError', false);
        % dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'ReturnOnError', false);
        fclose(fileID);

        % Create output variable
        data = [dataArray{1:end-1}];
        % data(isnan(data)) = 0.0;

    %% Start drawing
    % If a figure with the title 'Workspace Evaluation' already exists, don't create a new one
    existingFig = findobj(0,'Name', 'Workspace Evaluation');
    if (isempty(existingFig))
        hfigure=figure('Position',[100 100 1000 800],'Color','white');
        set(gcf, 'Name','Workspace Evaluation','numbertitle','off');
        hold on;        grid on;        view(3);
        xlabel('x');    ylabel('y');    zlabel('z');

        axis([-0.7,0.1,-0.7,0.7,-0.4,0.8]);
        axis equal;
        drawRefFrame(eye(4),0.6);
    else
        hfigure=[];
    end

    rP=find(data(:,4)~=-1);
    reachedPts = data(rP,:);
    reachedPts(isnan(reachedPts)) = 0.0;

    % % Remove duplicates in order to fasten up the drawing
    % [Y idx ida]=unique(reachedPts(:,1:3),'rows');
    % reachedPts=reachedPts(idx,:);

    [simplegray,bluehot,hot2] = colormapRGBmatrices(size(reachedPts,1));
    bluehot=flipud(bluehot);
    colormap(bluehot);

    [hgroup,reachedPts] = drawPointCloud(reachedPts,drawSurfaces,bluehot,videoOn);
end