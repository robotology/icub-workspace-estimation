function [Oxyz ] = drawRefFrame(G,num,varargin)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Draw reference frames
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
length = num;
num = num*6;

Origin1 = G*[0 0 0 1]';

hex = [ 'c44d58';
        '3fb9af';
		'58add4';];
rgb = zeros(3,3);

for i=1:size(hex,1)
	hx = hex(i,:);
	rgb(i,:) = [hex2dec(hx(1:2)) hex2dec(hx(3:4)) hex2dec(hx(5:6))]/255;
end

x_axis = G*[length 0 0 1]';
h = quiver3(Origin1(1), Origin1(2), Origin1(3), x_axis(1) - Origin1(1), x_axis(2) - Origin1(2), x_axis(3) - Origin1(3), 0);
set(h, 'Color', rgb(1,:), 'LineWidth', num, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')

y_axis = G*[0 length 0 1]';
h = quiver3(Origin1(1), Origin1(2), Origin1(3), y_axis(1) - Origin1(1), y_axis(2) - Origin1(2), y_axis(3) - Origin1(3), 0);
set(h, 'Color', rgb(2,:), 'LineWidth', num, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')

z_axis = G*[0 0 length 1]';
h = quiver3(Origin1(1), Origin1(2), Origin1(3), z_axis(1) - Origin1(1), z_axis(2) - Origin1(2), z_axis(3) - Origin1(3), 0);
set(h, 'Color', rgb(3,:), 'LineWidth', num, 'MaxHeadSize', 4, 'ShowArrowHead', 'on')

h = text(x_axis(1)+0.05, x_axis(2), x_axis(3), 'x', 'Color', rgb(1,:));
h = text(y_axis(1), y_axis(2)+0.05, y_axis(3), 'y', 'Color', rgb(2,:));
h = text(z_axis(1), z_axis(2), z_axis(3)+0.05, 'z', 'Color', rgb(3,:));

set(h, 'FontSize', num + 10)

O = G*[0 0 0 1]'; O(end) = []; O = O';
x = G*[1 0 0 0]'; x(end) = []; x = x';
y = G*[0 1 0 0]'; y(end) = []; y = y';
z = G*[0 0 1 0]'; z(end) = []; z = z';

Oxyz = [O; x; y; z]; Oxyz = Oxyz';