%% Import data from text file.
% Script for importing data from the following text file:
%
%    /home/alecive/.local/share/yarp/contexts/iCubWorkspace/output.ini
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

%% Initialize variables.
filename = '/home/alecive/.local/share/yarp/contexts/iCubWorkspace/output.ini';
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

figure; hold on; grid on;

for i=1:1
    if data(i,4) ~= 0
        plot3(data(i,1),data(i,2),data(i,3));%,'Markersize',4,'Markercolor',data(i,4));
    end
end

xlabel('x')
ylabel('y')
zlabel('z')

axis([-0.5,0.2,-0.5,0.5,-0.2,0.6]);


scatter3(data(:,1),data(:,2),data(:,3),24,data(:,4),'.');
view(3)