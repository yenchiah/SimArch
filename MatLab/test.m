clc; clear all;

%% load floorplan
floorplan = imread('floorplan/floorplan.png');
display = imread('floorplan/display.png');
display_obstacle = display(:,:,1);
display = display(:,:,2); % green
wall = imread('floorplan/wall.png');
wall = wall(:,:,1); % red
window = imread('floorplan/window.png');
window = window(:,:,2); % green
region = imread('floorplan/region.png');
region = region(:,:,1);
boundary = imread('floorplan/boundary.png');
boundary = boundary(:,:,1);
obstacle = 255-((255-wall)+(255-window)+(255-display_obstacle));
obstacle(obstacle<255) = 0;
density = boundary;
density(density==density) = 0;