function [reachedPts,filename] = drawWorkspace(varargin)
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
%   filename    -> the filename used for the visualization. Not that useful, but still.

    %% Initialize variables according to input arguments

        % filename = '~/.local/share/yarp/contexts/iCubWorkspace/output.ini';
        % filename = '../app/conf/output.ini';
        filename = '../app/conf/output.ini';
        if nargin>0
            filename=varargin{1};
        end
        disp('Varargin:');
        disp(varargin);

        drawSurfaces=true;
        if nargin>1
            drawSurfaces=varargin{2};
        end

        if drawSurfaces==true
            numSplits = 100;
        else
            numSplits = 200;
        end

        videoOn=false;
        if nargin>2
            videoOn=varargin{3};
        end

    %% Process text file
        delimiter = ' ';
        % Format string for each line of text:
        formatSpec = '%f%f%f%f%[^\n\r]';

        % Open the text file.
        fileID = fopen(filename,'r');
        % dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,'HeaderLines' ,startRow-1,  'ReturnOnError', false);
        dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'ReturnOnError', false);
        fclose(fileID);

        % Create output variable
        data = [dataArray{1:end-1}];
        % data(isnan(data)) = 0.0;

    %% Movie stuff
        if videoOn
            writerObj=VideoWriter('iCubWorkspace','Motion JPEG AVI');
            writerObj.FrameRate=15;
            open(writerObj);
        end

    %% Start drawing
    figure('Position',[100 100 1000 800],'Color','white');
    hold on;        grid on;        view(3);
    xlabel('x');    ylabel('y');    zlabel('z');

    axis([-0.7,0.1,-0.7,0.7,-0.4,0.8]);
    axis equal;
    drawRefFrame(eye(4),0.6);

    rP=find(data(:,4)~=0 & data(:,4)~=-1);
    reachedPts = data(rP,:);
    reachedPts(isnan(reachedPts)) = 0.0;

    % Sort the reached points according to the manipulability
    [Y,idx]=sort(reachedPts(:,4));
    reachedPts=reachedPts(idx,:);

    % Decompose the matrix
    l = int32(linspace(1,size(reachedPts,1),numSplits+1));

    x = reachedPts(:,1);
    y = reachedPts(:,2);
    z = reachedPts(:,3);
    c = reachedPts(:,4);
    K = convhull(x,y,z);
    h = trisurf(K,x,y,z,c,'facealpha',0.5);
    shading interp;
    set(h,'Visible','Off');
    colorbar;
    caxis([min(c) max(c)]);

    % Create custom colormaps:
        M = [0,0;1,1;];
        MR=[0,0; 0.02,0.3; 0.3,1; 1,1];
        MG=[0,0;  0.3,0;   0.7,1; 1,1];
        MB=[0,0;  0.7,0;          1,1];
        simplegray = colormapRGBmatrices(length(c), M, M, M);
        hot2 = colormapRGBmatrices(length(c),MR,MG,MB);
        bluehot = colormapRGBmatrices(length(c),MB,MG,MR);
        colormap(bluehot)

    % Split the workspace in #numSplit different surfaces and draw them over the time
    for i = numSplits:-1:1
        x = reachedPts(l(i):l(i+1),1);
        y = reachedPts(l(i):l(i+1),2);
        z = reachedPts(l(i):l(i+1),3);

        if drawSurfaces==true
            c = reachedPts(l(i):l(i+1),4);
            K = convhull(x,y,z);
            trisurf(K,x,y,z,c,'facealpha',0.7);
            shading interp;
        else
            c = bluehot(l(i):l(i+1),:);
            scatter3(x,y,z,40,c,'fill');
        end

        if videoOn
            frame=getframe(gcf);
            writeVideo(writerObj,frame);
        else
            pause(0.0125);
        end
    end

    % Plot only the surface of the points under evaluation.
    % K = convhull(x,y,z);
    % trisurf(K,x,y,z,c,'facealpha',0.5);

    % colormap('summer');

    if videoOn
        close(writerObj);
    end
end