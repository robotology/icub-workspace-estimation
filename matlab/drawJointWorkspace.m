function [crossedPts,hgroup] = drawJointWorkspace(varargin)

    close all;
    if exist('iCubWorkspace.avi','file')
        delete iCubWorkspace.avi;
    end

    videoOn = 0;
    if nargin>0
        videoOn = varargin{1};
    end

    if nargin>1
        filenames=varargin{2};
    else
        filenames{1} = '../app/conf/output_left.ini';
        filenames{2} = '../app/conf/output_right.ini';
    end

    [reachedPts{1},hgroup{1}] = drawWorkspace(filenames{1},1,videoOn);
    hl = get(hgroup{1},'Children');% cb is handle of hggroup
    set(hl,'FaceAlpha',0.01);
    % set(hl,'Visible','Off');

    for i = 2:length(filenames)
        [reachedPts{i},hgroup{i}] = drawWorkspace(filenames{i},1,videoOn+1);
        hl = get(hgroup{i},'Children');% cb is handle of hggroup
        set(hl,'FaceAlpha',0.01);
    end

    freezeColors

    if length(filenames) == 2
        A=reachedPts{1};
        B=reachedPts{2};

        crossedPts = intersectWorkspaces(A,B);
        reachedPts{3} = crossedPts;

        [simplegray,bluehot,hot2] = colormapRGBmatrices(size(crossedPts,1));
        hot2 = flipud(hot2);
        colormap(hot2);

        % hfigure=figure('Position',[100 100 1000 800],'Color','white');
        % set(gcf, 'Name','Workspace Evaluation','numbertitle','off');
        % hold on;        grid on;        view(3);
        % xlabel('x');    ylabel('y');    zlabel('z');

        % axis([-0.7,0.1,-0.7,0.7,-0.4,0.8]);
        % axis equal;
        % drawRefFrame(eye(4),0.6);


        hgroup{3}=drawPointCloud(crossedPts,1,hot2,videoOn+2);
        hl = get(hgroup{3},'Children');% cb is handle of hggroup
    end

    
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