function [simplegray, bluehot, hot2] = colormapRGBmatrices(N)
% This function creates a custom color map (fucking cool!)
% 
% It expects 4 input parameters: N is the number of intermediate points that your colormap should have. The other three are matrices that contain the transitions for each channel. Such a matrix should have the following form:
%
% M = [ 0, b1;
%       x1,b2;
%       ...
%       x_n, b_(n+1);
%       1, b_(n+1);
%     ];
%
% the first column give the fractions, where a brightness value should be defined. The second column should contain the brightness levels. Make sure to start the first column at 0 and end it with 1!
%
% A simple, linear grayscale map can be created with
%
%
% M = [0,0;1,1;];
% simplegray = colormapRGBmatrices( 256, M, M, M);
%
% Try also with:
%
% MR=[0,0; 
%     0.02,0.3; %this is the important extra point
%     0.3,1;
%     1,1];
% MG=[0,0;
%     0.3,0; 
%     0.7,1;
%     1,1];
% MB=[0,0; 
%     0.7,0;
%     1,1];
% hot2 = colormapRGBmatrices(500,MR,MG,MB);
%
% Or:
%
% bluehot = colormapRGBmatrices(500,MB,MG,MR);
%

    M  = [0,0;1,1;];
    MR = [0,0; 0.02,0.3; 0.3,1; 1,0.95];
    MG = [0,0;  0.3,0;   0.7,1; 1,0.95];
    MB = [0,0;  0.7,0;          1,0.95];
    simplegray = createColormap(N, M, M, M);
    hot2       = createColormap(N,MR,MG,MB);
    bluehot    = createColormap(N,MB,MG,MR);
end

function mymap=createColormap(N,rm,gm,bm)
    x = linspace(0,1, N);
    rv = interp1( rm(:,1), rm(:,2), x);
    gv = interp1( gm(:,1), gm(:,2), x);
    mv = interp1( bm(:,1), bm(:,2), x);
    mymap = [ rv', gv', mv'];
    %exclude invalid values that could appear
    mymap( isnan(mymap) ) = 0;
    mymap( (mymap>1) ) = 1;
    mymap( (mymap<0) ) = 0;
end
