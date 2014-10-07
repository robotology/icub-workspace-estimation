function vxl = computeVoxelMatrix(pts,numBins)

    numBins = [64 64 16];

    % Create the empty output matrix
    vxl = zeros(numBins(1),numBins(2),numBins(3));

    % Compute the extensions of the bins in the 3 axis
    binW   = [(max(pts(:,1))-min(pts(:,1)))/numBins(1),
              (max(pts(:,2))-min(pts(:,2)))/numBins(2),
              (max(pts(:,3))-min(pts(:,3)))/numBins(3)]; 

    for i=1:size(pts,1)
        idxX = floor((pts(i,1)-min(pts(:,1)))/binW(1))+1;
        if idxX==numBins(1)+1
            idxX=1;
        end
        idxY = floor((pts(i,2)-min(pts(:,2)))/binW(2))+1;
        if idxY==numBins(2)+1
            idxY=1;
        end
        idxZ = floor((pts(i,3)-min(pts(:,3)))/binW(3))+1;
        if idxZ==numBins(3)+1
            idxZ=1;
        end
        vxl(idxX,idxY,idxZ) = vxl(idxX,idxY,idxZ) + 1;
    end

    vxl=smooth3(vxl);

end