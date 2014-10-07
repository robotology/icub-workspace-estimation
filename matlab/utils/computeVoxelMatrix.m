function vxl = computeVoxelMatrix(pts,numBins)

    % numBins = [64 64 16];

    % Create the empty output matrix
    vxl = zeros(numBins(1),numBins(2),numBins(3));

    % Compute the extensions of the bins in the 3 axis
    binW   = [(max(pts(:,1))-min(pts(:,1)))/numBins(1),
              (max(pts(:,2))-min(pts(:,2)))/numBins(2),
              (max(pts(:,3))-min(pts(:,3)))/numBins(3)]; 

    for i=1:size(pts,1)
        for j=1:3
            idx(j) = floor((pts(i,j)-min(pts(:,j)))/binW(j))+1;
            if idx(j)==numBins(j)+1
                idx(j)=numBins(j);
            end
        end
        vxl(idx(1),idx(2),idx(3)) = vxl(idx(1),idx(2),idx(3)) + 1;
    end

    vxl=smooth3(vxl);

    % hiso = patch(isosurface(Ds,0.3),'FaceColor',rand(3,1),'EdgeAlpha',0.2,'FaceAlpha',0.6)

end