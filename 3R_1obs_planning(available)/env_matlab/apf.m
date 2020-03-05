function  f=apf()
%APF 建立人工势场图
%% 产生采样点

nrows = 600;
ncols = 600;

obstacle = false(nrows, ncols);

[x, y] = meshgrid (1:ncols, 1:nrows);
%% 产生障碍点

obstacle (270:330, 270:end) = true;

%% Compute distance transform
d = bwdist(obstacle);
% Rescale and transform distances
d2 = (d/100) + 1;
d0 = 2;
nu = 800;
repulsive = nu*((1./d2 - 1/d0).^2);
repulsive (d2 > d0) = 0;
%% 
%% Display repulsive potential 斥力

figure;
m = mesh (repulsive);
m.FaceLighting = 'phong';
axis equal;
title ('Repulsive Potential');
%%  Compute attractive force
goal = [300, 180];
xi = 1/700;
attractive = xi * ( (x - goal(1)).^2 + (y - goal(2)).^2 );
figure;
m = mesh (attractive);
m.FaceLighting = 'phong';
axis equal;

title ('Attractive Potential');

%% Display 2D configuration space
figure;
imshow(~obstacle);
hold on;
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
hold off;
axis ([0 ncols 0 nrows]);
axis xy;
axis on;
xlabel ('x');
ylabel ('y');

title ('Configuration Space');
%% Combine terms
f = attractive + repulsive;

figure;
m = mesh (f);
m.FaceLighting = 'gouraud';
axis equal;

title ('Total Potential');
%% quiver plot
[gx, gy] = gradient (-f);
skip = 20;

figure;

xidx = 1:skip:ncols;
yidx = 1:skip:nrows;

quiver (x(yidx,xidx), y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);

axis ([1 ncols 1 nrows]);

end

