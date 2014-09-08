function drawPointCloud(pC,dS,cM,vO)

    reachedPts   = pC;
    drawSurfaces = dS;
    videoOn      = vO;
    cMap         = cM;

    % Sort the reached points according to the manipulability
    [Y,idx]=sort(reachedPts(:,4));
    reachedPts=reachedPts(idx,:);

    % Set a suitable number of splits (for both video and visualization purposes)
    if drawSurfaces==true
        numSplits = 50;
    else
        numSplits = 100;
    end

    %% Movie stuff
        if videoOn
            writerObj=VideoWriter('iCubWorkspace','Motion JPEG AVI','Quality',100);
            writerObj.FrameRate=15;
            open(writerObj);
        end

    % Decompose the matrix
    l = int32(linspace(1,size(reachedPts,1),numSplits+1));

    x = reachedPts(:,1);
    y = reachedPts(:,2);
    z = reachedPts(:,3);
    c = reachedPts(:,4);
    K = convhull(x,y,z);
    h = trisurf(K,x,y,z,c,'FaceAlpha',0.03);
    shading interp;
    set(h,'Visible','Off');
    colorbar;
    caxis([min(c) max(c)]);

    if videoOn
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
            trisurf(K,x,y,z,c,'FaceAlpha',0.03);
            shading interp;
        else
            c = cMap(l(i):l(i+1),:);
            scatter3(x,y,z,40,c,'fill');
        end

        if videoOn
            frame=getframe(gcf);
            writeVideo(writerObj,frame);
        else
            pause(0.0125);
        end
    end

    if videoOn
        % Add 20 empty frames at the end in order to have ~2 sec of wait at the end
        for i=1:15
            writeVideo(writerObj,frame);
        end
        close(writerObj);
    end

    % Compute the volume of the point cloud
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
    disp(sprintf('\tMin   (x,y,z): %g\t%g\t%g\t[m]',min(reachedPts(:,1:3))));
    disp(sprintf('\tMax   (x,y,z): %g\t%g\t%g\t[m]',max(reachedPts(:,1:3))));
    disp(sprintf('\tRadii (x,y,z): %g\t%g\t%g\t[m]',radius));
    disp(sprintf('\tVolume:        %g\t[m^3]',vol));
    disp('Manipulability statistics:');
    disp(sprintf('\tMin: %g\t[m]',min(reachedPts(:,4))));
    disp(sprintf('\tMax: %g\t[m]',max(reachedPts(:,4))));
end