function [crossedPts,hgroup] = drawJointWorkspace(varargin)
%% Draws the workspace of more than one chain.
%
% Function for importing data from a proper output.ini file
%
%   [reachedPts,hgroup,chain] = drawJointWorkspace(varargin)
%
% It can be called as it is (in this case it uses default parameters), or with a growing number of input
% arguments, specified below.
% 
% INPUT ARGUMENTS:
%   drawSurfaces -> int  to know what to draw. It can be either 0 (set of 3d points), 1 (set of convex hulls, default),
%                   or 2 (a single convex hull encompassing all of the points). If available, the points will be color-coded
%                   according to their manipulability measure. If not, the coloration is random.
%   videoOn      -> flag to know if to record a video or not (default false). If true, it records an iCubWorkspace.avi file.
%   drawChain    -> flag to know if to draw the kinematic chain or not (default trye).
%                   If true, it loads the kinematic info to a file placed in the same folder as the output.ini and called exactly 
%                   the same but with 'output' replaced by 'DH' (e.g. if I specify 'output_whatever.ini' it will search for 'DH_whatever.ini')
%   filename     -> cell of ini files to draw.
%
% OUTPUT:
%   reachedPtS  -> an Nx4 array of reached 3D points + their manipulability index.
%                  It has been ordered according to the magnitude of the manipulability (from low to high)
%   hgroup      -> handle for the surfaces that have been drawn
%   chain       -> class enclosing everything related to the kinematic chain. if drawChain has bee set to false, 
%                  this will be an empty variable.
% 
% EXAMPLE USAGE:
%   [r,h,c] = drawWorkspace(2,1,0):
%           1. draw a single convex hull for all the points
%           2. record a video
%           3. do not draw the kinematic chain
%           4. load the default set of ini files
%

    close all;
    if exist('iCubWorkspace.avi','file')
        delete iCubWorkspace.avi;
    end

    drawSurfaces=2;
    if nargin>0
        drawSurfaces=varargin{1};
        disp(sprintf('    drawSurfaces: %i',drawSurfaces));
    end

    videoOn=false;
    if nargin>1
        videoOn=varargin{2};
        disp(sprintf('    videoOn: %i',videoOn));
    end

    drawChain=true;
    if nargin>2
        drawChain=varargin{3};
        disp(sprintf('    drawChain: %i',drawChain));
    end

    if nargin>3
        filenames=varargin{4};
    else
        i=1;
        % filenames{i+1} = '../app/conf/output_left.ini';  i=i+1;
        % filenames{i+1} = '../app/conf/output_right.ini';  i=i+1;
        
        filenames{i} = 'c9/output_Thumb.ini';  i=i+1;
        filenames{i} = 'c9/output_Index.ini';  i=i+1;
        % filenames{i+1} = 'c9/output_Middle.ini';  i=i+1;
        % filenames{i+1} = 'c9/output_Ring.ini';  i=i+1;
        % filenames{i+1} = 'c9/output_Little.ini';   i=i+1;
        
        % filenames{i+1} = 'c9/output_v2Thumb.ini';  i=i+1;
        % filenames{i+1} = 'c9/output_v2Index.ini';  i=i+1;
        % filenames{i+1} = 'c9/output_v2Middle.ini';  i=i+1;
        % filenames{i+1} = 'c9/output_v2Ring.ini';  i=i+1;
        % filenames{i+1} = 'c9/output_v2Little.ini';   i=i+1;
    end

    % [reachedPts{1},hgroup{1}] = drawWorkspace(filenames{1},1,videoOn,1);
    % hl = get(hgroup{1},'Children');% cb is handle of hggroup
    % % set(hl,'FaceAlpha',0.01);
    % % set(hl,'Visible','Off');

    for i = 1:length(filenames)
        [reachedPts{i},hgroup{i}] = drawWorkspace(filenames{i},drawSurfaces,videoOn,drawChain);
    end

    for i = 1:length(filenames)
        hchildren = get(hgroup{i},'Children');% cb is handle of hggroup
        set(hchildren,'FaceColor',[0 0.5 1]);
    end

    % freezeColors

    % if length(filenames) == 2
    %     A=reachedPts{1};
    %     B=reachedPts{2};

    %     crossedPts = intersectWorkspaces(A,B);
    %     reachedPts{3} = crossedPts;

    %     [simplegray,bluehot,hot2] = colormapRGBmatrices(size(crossedPts,1));
    %     hot2 = flipud(hot2);
    %     colormap(hot2);

    %     % hfigure=figure('Position',[100 100 1000 800],'Color','white');
    %     % set(gcf, 'Name','Workspace Evaluation','numbertitle','off');
    %     % hold on;        grid on;        view(3);
    %     % xlabel('x');    ylabel('y');    zlabel('z');

    %     % axis([-0.7,0.1,-0.7,0.7,-0.4,0.8]);
    %     % axis equal;
    %     % drawRefFrame(eye(4),0.6);


    %     hgroup{3}=drawPointCloud(crossedPts,1,hot2,videoOn+2);
    %     hl = get(hgroup{3},'Children');% cb is handle of hggroup
    % end

    
end

function iW = intersectWorkspaces(A,B)

    [iW ia ib ] = intersect(A(:,1:3),B(:,1:3),'rows');

    for i=1:length(ia)
        iW(i,4) = A(ia(i),4)+B(ib(i),4);
    end

    % iW=[];
    % for i=1:size(A,1)
    %     for j=1:size(B,1)
    %         if (isequal(A(i,1:3),B(j,1:3)))
    %             iW=[iW;A(i,1:3),A(i,4)+B(j,4)];
    %         end
    %     end
    % end

end