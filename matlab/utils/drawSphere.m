function HandleSph = drawSphere(radius, Trasl, Color, NumberOfFaces, TransparencyFactor)
% Genova 03/08/2014
% Author: Alessandro Roncone
%
% This function draws a sphere of a given radius and traslation.
% The vector Trasl is a 3x1 vector.

[X,Y,Z] = sphere(20);
X = X*radius + Trasl(1);
Y = Y*radius + Trasl(2);
Z = Z*radius + Trasl(3);

C(:,:,1) = ones(size(Z)).*Color(1);
C(:,:,2) = ones(size(Z)).*Color(2);
C(:,:,3) = ones(size(Z)).*Color(3);

HandleSph = surf(X, Y, Z, C, 'FaceColor', Color, 'EdgeColor', 'none');
set(HandleSph, 'EdgeAlpha', TransparencyFactor, 'AlphaDataMapping', 'none', 'FaceAlpha', TransparencyFactor)