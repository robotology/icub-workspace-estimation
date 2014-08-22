%% Import data from text file.
% Script for importing data from the following text file:
%
%    /home/alecive/.local/share/yarp/contexts/iCubWorkspace/output.ini
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

%% Initialize variables.
filename = '~/.local/share/yarp/contexts/iCubWorkspace/output.ini';
% filename = '../app/conf/output.ini';
delimiter = ' ';

%% Format string for each line of text:
formatSpec = '%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Create output variable
data = [dataArray{1:end-1}];
%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;

figure;
hold on; grid on; view(3);

xlabel('x')
ylabel('y')
zlabel('z')

axis([-0.5,0.2,-0.5,0.5,-0.2,0.6]);
axis equal;

rP=find(data(:,4)~=0);
reachedPts = data(rP,:);
x = reachedPts(:,1);
y = reachedPts(:,2);
z = reachedPts(:,3);
c = reachedPts(:,4);

% Plot only the surface of the points under evaluation.
K = convhull(x,y,z);
trisurf(K,x,y,z);

% scatter3(reachedPts(:,1),reachedPts(:,2),reachedPts(:,3),12,reachedPts(:,4),'.');

drawRefFrame(eye(4),0.6);

clear i;