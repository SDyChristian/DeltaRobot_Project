function Tray = closed_path(Zmin,Zmax)  
% Author: Christian Trejo from SD&C
% This algorithm converts an image into a trajectory so delta robot can draw content image (actually a path) 

% Input parameters: End efector height for drawing (Zmin) and relocalization (Zmax)
% Output: Trajectory extracted from an image


%% Image Processing
Oim = imread('polea.png');      % Load image
im = rgb2gray(Oim);             % Gray Scale
bw = im2bw(im,0.5);             % Binary Image
bw = 1-bw;                      % Invert image
bw = bwmorph(bw,'thin',Inf);    % thinning contours to a pixel size
se=[0 0 0; 0 1 0 ; 0 0 0];      % Structural element
bw = bwmorph(bw,'spur',inf);    % Remove branches

figure(1)
subplot(1,2,1)
imshow(Oim) 
title('Polea without processing')
subplot(1,2,2)
imshow(bw) 
title('Polea after image processing')

%% Detection of number of objects found in "bw" image
CC = bwconncomp(bw);
L  = labelmatrix(CC); % Label each object found in image  
RGB = label2rgb(L);   % Assign a color to each object

% Plot objects
figure(2)
imshow(RGB)
title('Objects found on image')

%% Clean All Noise Objects 
imz=zeros(CC.ImageSize);    % Defien Matriz to draw
c=0; % Counter
objects = zeros( [], CC.NumObjects*2 );
for i=1 : CC.NumObjects
    imz(CC.PixelIdxList{i})=1;  % Set to 1 objects index 
    % subplot(3,6,i) ;imshow(imz,[]); % To see each object

    [ row, col ] = find( imz, 1, 'first' );
    if length(CC.PixelIdxList{i})<10
        % If there is an object with less than ten index then it is noise
        % Ignore it, do nothing.
    else
        % contour contains object coordinates 
        contour = bwtraceboundary(imz, [row, col], 'E', 8, Inf,'counterclockwise');
        % objects stores all paths for each object 
        objects(1 : size(contour, 1), c+1: c+2)=contour;
        % An increment of two is used to store the pair (X,Y) of each
        % object
        c=c+2;
        
    end   
    % Imz is set to zero to be used in next "for" iteration
    imz=imz*0;
end

figure(3)
for i = 0 : 2 : length(objects(1,:))-2
        plot( objects(:,i+1) , objects(:,i+2),'b.') 
        hold on 
end
title("Objects paths (without noise elements)")

%% All objects are linked into a single path
Rt=objects;
Pt=[0,0];
% Auxiliar variables
c=0;
B=0;
for i=0 : 2 : length(Rt(1,:))-2
    % (X,Y) Coordinates extracted from an object 
    X=(Rt(:,c+1)); 
    Y=(Rt(:,c+2));
    
    % If there is a pixel size object then ignore it
    if Rt(2,c+1)==0 && Rt(2,c+2)==0
        % Do nothing.
    else
        % Convert from pixels to Spline
        t = 1:length( X(X>0) );
        xy = [ X(X>0) , Y(Y>0) ]';
        infty = csape(t, xy, 'periodic');
        Pt = fnplt(infty, 1); Pt=Pt';
        Nps=length(Pt(:,1)); 
    end
    
    % Cycle to walk through path point to point 
    t=1; 
    % First (X,Y) position is realted to Z0=Zmax
    X0( t+B )=Pt(t,1); 
    Y0( t+B )=length(Pt(t,2))-Pt(t,2); 
    Z0( t+B )=Zmax;
    % Set B++ so that previous position assigned is not rewritten 
    B = B+1; 
    % Walk through each point but at Z0=Zmin (End effector goes down)
    for t=1 : length(Pt(:,1)) 
        X0( t+B )=Pt(t,1); 
        Y0( t+B )=length(Pt(t,2))-Pt(t,2); 
        Z0( t+B )=Zmin; 
    end
    % Set last (X,Y) position but with Z0=Zmax (End effector goes up)
    X0( t+B+1 )=Pt(t,1); 
    Y0( t+B+1 )=length(Pt(t,2))-Pt(t,2);
    Z0( t+B+1 )=Zmax; 
  
    B=B+1; % Increase B+1 to compensate (t+B+1) in X0-Y0-Z0
    Tray=[X0;Y0;Z0]; % Save first path
    % Increase B as B+Nps to do not overwrite current path with the new one
    B=B+Nps;  
    c=c+2; % Increase C as C+2 to access next path 
end

figure(4)
plot3(Tray(1,:),Tray(2,:),Tray(3,:))
title("Plot full path with end effector movements")
grid on