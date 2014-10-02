function [handleGroup, reachedPts] = drawPointCloud(pC,dS,cM,vO)

    reachedPts   = pC;
    drawSurfaces = dS;
    videoOn      = vO;
    cMap         = cM;

    handleGroup  = hggroup;

    % Sort the reached points according to the manipulability
    [Y,idx]=sort(reachedPts(:,4));
    reachedPts=reachedPts(idx,:);

    % Change the reference system from m to mm
    reachedPts(:,1:3)=reachedPts(:,1:3).*1000;

    % Set a suitable number of splits (for both video and visualization purposes)
    if drawSurfaces==true
        if size(reachedPts,1)>500
            numSplits = 50;
        else
            numSplits = 1+int16(size(reachedPts,1)/10);
            disp(numSplits);
        end
    else
        numSplits = 100;
    end

    %% Movie stuff
        if videoOn>0
            if ~exist('iCubWorkspace.avi','file')
                writerObj=VideoWriter('iCubWorkspace','Motion JPEG AVI');
                writerObj.Quality = 100;
                writerObj.FrameRate=15;
                open(writerObj);
            else
                readerObj=VideoReader('iCubWorkspace.avi');
                writerObj=VideoWriter('iCubWorkspace2','Motion JPEG AVI');
                writerObj.FrameRate=readerObj.FrameRate;
                writerObj.Quality = 100;
                open(writerObj);
                for i = 1:readerObj.NumberofFrames
                    frame = read(readerObj,i);
                    writeVideo(writerObj,frame);
                end
            end
        end

    % Decompose the matrix
    l = int32(linspace(1,size(reachedPts,1),numSplits+1));

    x = reachedPts(:,1);
    y = reachedPts(:,2);
    z = reachedPts(:,3);
    c = reachedPts(:,4);

    try
        K = convhull(x,y,z);
        h = trisurf(K,x,y,z,c,'FaceAlpha',0.175);
        shading interp;
        set(h,'Visible','Off');
        colorbar;
        caxis([min(c) max(c)]);
    end

    if videoOn==1
        for i=1:30
            camorbit(-4,-0.1);
        end
        % Add 15 empty frames at the beginning in order to have ~1 sec of wait
        for i=1:15
            frame=getframe(gcf);
            writeVideo(writerObj,frame);
        end
    end

    % Split the workspace in #numSplit different surfaces and draw them over the time
    for i = numSplits:-1:1
        x = reachedPts(l(i):l(i+1),1);
        y = reachedPts(l(i):l(i+1),2);
        z = reachedPts(l(i):l(i+1),3);

        if drawSurfaces==true
            c = reachedPts(l(i):l(i+1),4);
            K = convhull(x,y,z);
            h = trisurf(K,x,y,z,c,'FaceAlpha',0.175);
            set(h,'Parent',handleGroup);
            shading interp;
        else
            c = cMap(l(i):l(i+1),:);
            scatter3(x,y,z,40,c,'fill');
        end

        if videoOn>0
            frame=getframe(gcf);
            camorbit(3,-0.1);
            writeVideo(writerObj,frame);
            frame=getframe(gcf);
            camorbit(3,-0.1);
            writeVideo(writerObj,frame);
        else
            pause(0.0075);
        end
    end

    % if drawSurfaces==false
    %     for i = 1:size(reachedPts,1)
    %         x=[0 reachedPts(i,1)];
    %         y=[0 reachedPts(i,2)];
    %         z=[0 reachedPts(i,3)];
    %         plot3(x,y,z);
    % end

    if videoOn
        % Add 30 empty frames at the end in order to have ~2 sec of wait at the end
        if videoOn==3
            for i=1:60
                camorbit(-2,-0.1);
                frame=getframe(gcf);
                writeVideo(writerObj,frame);
            end
        end
        close(writerObj);

        if exist('iCubWorkspace2.avi','file')
            movefile('iCubWorkspace2.avi','iCubWorkspace.avi');
        end
    end

    % Compute the volume of the point cloud
    try
        P = reachedPts(:,1:3)';
        K = convhull(P');
        K = unique(K(:));
        Q = P(:,K);
        [A, c] = MinVolEllipse(Q, .01);
        % Ellipse_plot(A,c);
        [U Q V] = svd(A);
        radius(1) = 1/sqrt(Q(1,1));
        radius(2) = 1/sqrt(Q(2,2));
        radius(3) = 1/sqrt(Q(3,3));
        vol = (4/3)*pi*sqrt(det(A^-1));

        % Prints out some statistics
        disp('Workspace statistics:');
        disp(sprintf('\tMin   (x,y,z): %g\t%g\t%g\t[mm]',min(reachedPts(:,1:3))));
        disp(sprintf('\tMax   (x,y,z): %g\t%g\t%g\t[mm]',max(reachedPts(:,1:3))));
        disp(sprintf('\tRadii (x,y,z): %g\t%g\t%g\t[mm]',radius));
        disp(sprintf('\tVolume:        %g\t[mm^3]',vol));
        disp('Manipulability statistics:');
        disp(sprintf('\tMin: %g\t[mm]',min(reachedPts(:,4))));
        disp(sprintf('\tMax: %g\t[mm]',max(reachedPts(:,4))));
    catch
        disp('   I could not Compute the volume (there was some problem)!');
    end
end