function [reachedPts,hgroup,chain] = drawWorkspace(varargin)
% Draw the workspace from a text file
% Function for importing data from a proper output.ini file
%
%   [reachedPts,filename] = drawWorkspace(varargin)
%
% INPUT:
%   varargin{1} -> output.ini file to be used for the drawing
%   varargin{2} -> flag to know if to draw either a set of isosurfaces (true, default) or points (false)
%   varargin{3} -> flag to know if to record a video or not (default false). If true, it records an iCubWorkspace.avi file.
%   varargin{4} -> flag to know if to draw the kinematic chain or not (default false).
%                  If true, it loads the kinematic info to a file placed in the same folder as the output.ini and called exactly 
%                  the same but with 'output' replaced by 'DH'
%
% OUTPUT:
%   reachedPtS  -> an Nx4 array of reached 3D points + their manipulability index.
%                  It has been ordered according to the magnitude of the manipulability (from low to high)
%   hgroup      -> handle for the surfaces that has been drawn
        addpath('./utils/');

    %% Initialize variables according to input arguments
        disp('Varargin:');
        disp(varargin);

        % filename = '~/.local/share/yarp/contexts/iCubWorkspace/output.ini';
        filename = '../app/conf/output_right.ini';
        if nargin>0
            if ~strcmp(varargin{1},'default')
                filename=varargin{1};
            end
        end
        disp(sprintf('    filename: %s',filename));

        drawSurfaces=true;
        if nargin>1
            drawSurfaces=varargin{2};
            disp(sprintf('    drawSurfaces: %i',drawSurfaces));
        end

        videoOn=false;
        if nargin>2
            videoOn=varargin{3};
            disp(sprintf('    videoOn: %i',videoOn));
        end

        drawChain=false;
        if nargin>3
            drawChain=varargin{4};
            disp(sprintf('    drawChain: %i',drawChain));
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

        % axis([-0.7,0.1,-0.7,0.7,-0.4,0.8]);
        % axis equal;
        drawRefFrame(eye(4),1);
    else
        hfigure=[];
    end

    rP=find(data(:,4)~=-1);
    reachedPts = data(rP,:);
    reachedPts(isnan(reachedPts)) = 0.0;

    % % Remove duplicates in order to fasten up the drawing
    [Y idx ida]=unique(reachedPts(:,1:3),'rows');
    reachedPts=reachedPts(idx,:);

    [simplegray,bluehot,hot2] = colormapRGBmatrices(size(reachedPts,1));
    bluehot=flipud(bluehot);
    colormap(bluehot);

    disp('Drawing point Cloud..');
    [hgroup,reachedPts] = drawPointCloud(reachedPts,drawSurfaces,bluehot,videoOn);
    axis equal;

    if drawChain==true
        hl = get(hgroup,'Children');% cb is handle of hggroup
        % set(hl,'FaceAlpha',0.3);

        chainfile=strrep(filename, 'output', 'DH');
        disp('Drawing kinematic chain..');
        disp(sprintf('    File to load: %s',chainfile));
        [chain, rawdata] = drawKinematicChain(chainfile);
    end
    axis equal;
end