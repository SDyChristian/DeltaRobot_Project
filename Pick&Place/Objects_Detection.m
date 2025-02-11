function [x,y] = Object_Detection

% Author: Christian Trejo SD&C
% This function enables camera, performs image processing to detect objects
% and returns two sets of vectors (px,py) that contain objects centroid coordinates

%% Camera Config.
clear all
% Set camera configuration
vid = videoinput('winvideo', 1, 'RGB24_640x480');
src = getselectedsource(vid);
%
vid.FramesPerTrigger = 1;
frame = getsnapshot(vid);
delete(vid)
im = frame; % Image to be processed


% Edges get rid of
im = im( 2:end-2, 2:end-2, : );
figure, imshow( im )

% Image converted to grayscale
img = im2double( im(:,:,2) );

% Threshholding
umbral = graythresh( img ); % Otsu method
imbw = img < umbral;

% Morphological closing operation
ee = ones(9,9); % Elemento estructurante 7x7 para HD (1280x780)
imbw = imclose( imbw, ee );

% Connected components
CC = bwconncomp( imbw );
% k_min and kmax are empirically tuned constants 
k_min = 0.00015; k_max = 0.02;
porcmin = prod( CC.ImageSize ) * k_min;
porcmax = prod( CC.ImageSize ) * k_max;
% Objects with less and greater than certain number of pixels
% are removed (cleaning objects using an upper and lower threshold)
for i = 1 : CC.NumObjects
    if numel( CC.PixelIdxList{i} ) < porcmin || numel( CC.PixelIdxList{i} ) > porcmax
        % Set to zero the coordinates
        imbw( CC.PixelIdxList{i} ) = 0;
    end
end
figure
imshow(imbw)
hold on

% Get centroid of objects 
s = regionprops(imbw, 'Centroid');
% Plot centroid
for k = 1:length(s)
    % Centroid coordeiantes
    xcen = s(k).Centroid(1);
    ycen = s(k).Centroid(2);
    % plot centroid
    x(k) = xcen; y(k)=ycen;
    plot(xcen, ycen, 'b+','MarkerSize', 6);
end
hold off
end