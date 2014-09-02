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

if ~exist('filename','var')
    filename = './output.ini';
end

delimiter = ' ';

%% Format string for each line of text:
formatSpec = '%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');
% dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,'HeaderLines' ,startRow-1,  'ReturnOnError', false);
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'ReturnOnError', false);
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

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

rP=find(data(:,4)~=0 & data(:,4)~=-1);
reachedPts = data(rP,:);
reachedPts(isnan(reachedPts)) = 0.0;

% Sort the reached points according to the manipulability
[Y,idx]=sort(reachedPts(:,4));
reachedPts=reachedPts(idx,:);

% Decompose the matrix
numSplits = 20;
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

for i = numSplits:-1:1
    x = reachedPts(l(i):l(i+1),1);
    y = reachedPts(l(i):l(i+1),2);
    z = reachedPts(l(i):l(i+1),3);
    c = reachedPts(l(i):l(i+1),4);
    K = convhull(x,y,z);
    trisurf(K,x,y,z,c,'facealpha',0.7);
    shading interp;
    % scatter3(reachedPts(l(i):l(i+1),1),reachedPts(l(i):l(i+1),2),reachedPts(l(i):l(i+1),3),40,reachedPts(l(i):l(i+1),4),'fill');
    pause(0.5);
end

% Plot only the surface of the points under evaluation.
% K = convhull(x,y,z);
% trisurf(K,x,y,z,c,'facealpha',0.5);

% colormap('summer');
drawRefFrame(eye(4),0.6);

clear i x y z c rP ans Y idx;