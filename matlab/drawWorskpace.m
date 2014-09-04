%% Import data from text file.
% Script for importing data from the following text file:
%
%    /home/alecive/.local/share/yarp/contexts/iCubWorkspace/output.ini
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

%% Initialize variables.
% filename = '~/.local/share/yarp/contexts/iCubWorkspace/output.ini';
% filename = '../app/conf/output.ini';


filename = '../app/conf/output.ini';
if nargin>1
    filename=varargin{1};
end
    
videoOn=false;
if nargin>2
    videoOn=varargin{2};
end

delimiter = ' ';

%% Format string for each line of text:
formatSpec = '%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');
% dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,'HeaderLines' ,startRow-1,  'ReturnOnError', false);
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'ReturnOnError', false);
fclose(fileID);

%% Create output variable
data = [dataArray{1:end-1}];
% data(isnan(data)) = 0.0;
%% Clear temporary variables
clearvars delimiter formatSpec fileID dataArray ans;

figure('Position',[100 100 1000 800],'Color','white');
hold on; grid on; view(3);

xlabel('x')
ylabel('y')
zlabel('z')

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
numSplits = 200;
l = int16(linspace(1,size(reachedPts,1),numSplits+1));

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
    clear simplegray hot2

% Split the workspace in #numSplit different surfaces and draw them over the time
for i = numSplits:-1:1
    x = reachedPts(l(i):l(i+1),1);
    y = reachedPts(l(i):l(i+1),2);
    z = reachedPts(l(i):l(i+1),3);
    c = bluehot(l(i):l(i+1),:);
    K = convhull(x,y,z);
    % trisurf(K,x,y,z,c,'facealpha',0.7);
    % shading interp;
    scatter3(x,y,z,40,c,'fill');
    pause(0.0125);
end

% Plot only the surface of the points under evaluation.
% K = convhull(x,y,z);
% trisurf(K,x,y,z,c,'facealpha',0.5);

% colormap('summer');


clear i x y z c rP ans Y idx;