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
density = double(density);

%% variables
add_people_time = 600; % the time interval to add more people
max_people_num_each = 40; % the maximum number of people that can be added each time
add_people_num_threshold = 10; % the threshold that people can be added
near_threshold_least = 10; % the threshold to define 'near'
near_threshold_most = 20; % the threshold to define 'near'
scale = 3; % the scale to draw the people in the floorplan
start.row = 266; % starting row position
start.col = 83; % starting column position
leave.row = 265; % leaving row position
leave.col = 82; % leaving column position
decay_low = 500; % the lower bound of decay factor
decay_high = 7000; % the higher bound of decay factor
search_low = 500; % the lower bound of search factor
search_high = 1000; % the higher bound of search factor

%% add people
people_num = 0; % number of people
pr_searchTarget = randi([search_low search_high],1,people_num)/search_high; % the probability to search an interesting target in state 2
pr_searchBoundary = randi([search_low search_high],1,people_num)/search_high; % the probability to search an interesting target in state 4
decay_factor = 1./randi([decay_low decay_high],1,people_num); % the time decay factor of state 9
pos.row = zeros(1,people_num) + start.row; % row position of all people
pos.col = zeros(1,people_num) + start.col; % column position of all people
goal.row =  zeros(1,people_num) + inf; % goal row position of all people
goal.col =  zeros(1,people_num) + inf; % goal columnn position of all people
stopTime = zeros(1,people_num) + inf; % stopping time
map_display_remain(1:people_num) = {inf};
map_display(1:people_num) = {inf};
map_region_remain(1:people_num) = {region};
map_boundary_remain(1:people_num) = {boundary};
map_path(1:people_num) = {inf};
state = zeros(1,people_num)+2; % starting state
% 1: leave (floor)
% 2: in floor
% 3: far (boundary)
% 4: in boundary
% 5: safe (boundary)
% 6: safe (leaving F)
% 7: far (leaving F)
% 8: far (interested)
% 9: near (interested)
% 10: safe (interested)

%% Behavior Model (Markov Decision Process)
t = 0;
add_people_num = 0;
% compute scaled obstacle map
obstacle_withScale = 1-obstacle/max(max(obstacle));
filter = ones(1+scale*2);
obstacle_withScale = conv2(obstacle_withScale,filter,'same');
obstacle_withScale(obstacle_withScale~=0) = 1;
while true
    t = t + 1;
    idx_leave = []; % the people who want to leave
    temp_floorplan = floorplan; % for rendering
    % update
    for idx=1:people_num
%         [idx state(idx)]
        switch state(idx)
            case 1 % 1: leave (floor)
                map_display_remain{idx} = inf;
                map_display{idx} = inf;
                map_path{idx} = inf;                
                [map_path{idx} goal.row(idx) goal.col(idx)]...
                    = searchExit(pos.row(idx),pos.col(idx),obstacle,leave,scale);
                state(idx) = 7; % 7: far (leaving F)
            case 2 % 2: in floor
                dice = rand(1);
                if(dice<=pr_searchBoundary(idx)) % search a boundary
                    if(min(min(map_region_remain{idx}))~=255) % find
                        [map_boundary_remain{idx} map_region_remain{idx} map_display{idx} map_display_remain{idx} map_path{idx} goal.row(idx) goal.col(idx)]...
                            = searchBoundary(pos.row(idx),pos.col(idx),obstacle,map_region_remain{idx},map_boundary_remain{idx},display,scale);
                        state(idx) = 3; % 3: far (boundary)
                     else % cannot find
                        state(idx) = 1; % 1: leave (floor)
                    end 
                else % search an exit
                    map_display_remain{idx} = inf;
                    map_display{idx} = inf;
                    map_path{idx} = inf;
                    [map_path{idx} goal.row(idx) goal.col(idx)]...
                        = searchExit(pos.row(idx),pos.col(idx),obstacle,leave,scale);
                    state(idx) = 7; % 7: far (leaving F)                        
                end
            case 3 % 3: far (boundary)
                % compute obstacles
                obstacle_people = (obstacle~=obstacle)+255;
                for id_p=1:people_num
                    if(id_p~=idx)
                        obstacle_people(pos.row(idx),pos.col(idx)) = 0;
                    end
                end
                % try to move
                move = tryToMoveToTarget(goal.row(idx),goal.col(idx),pos.row(idx),pos.col(idx),map_path{idx},obstacle_people,scale);
                if(strcmp(move,'safe')==1)
                    state(idx) = 5; % 5: safe (boundary)
                end
                if(strcmp(move,'collided')==1)
                    state(idx) = 3; % 3: far (boundary)
                end
            case 4 % 4: in boundary
                dice = rand(1);
                if(dice<=pr_searchTarget(idx)) % search an interesting target
                    if(min(min(map_display{idx}))~=255) % find
                        [map_display{idx} map_path{idx} goal.row(idx) goal.col(idx)]...
                            = searchTarget(pos.row(idx),pos.col(idx),obstacle,map_display{idx},scale);
                        state(idx) = 8; % 8: far (interested)
                     else % cannot find
                        state(idx) = 2; % 2: in floor
                    end 
                else % in floor
                    map_display_remain{idx} = inf;
                    map_display{idx} = inf;
                    map_path{idx} = inf;
                    state(idx) = 2; % % 2: in floor                       
                end                
            case 5 % 5: safe (boundary)
                [map_path{idx} pos.row(idx) pos.col(idx)]...
                    = moveToTarget(goal.row(idx),goal.col(idx),pos.row(idx),pos.col(idx),map_path{idx});
                row_distance = abs(goal.row(idx)-pos.row(idx));
                col_distance = abs(goal.col(idx)-pos.col(idx));
                if(obstacle_withScale(pos.row(idx),pos.col(idx))==0)
                    if(row_distance~=0 || col_distance~=0) % far
                        state(idx) = 3; % 3: far (boundary)
                    else % near
                        state(idx) = 4; % 4: in boundary           
                    end
                else % step onto a obstacle
                    % move back to the place with no obstacle
                    if(obstacle_withScale(pos.row(idx)+1,pos.col(idx))==0)
                        pos.row(idx) = pos.row(idx)+1;
                    else
                        if(obstacle_withScale(pos.row(idx)-1,pos.col(idx))==0)
                            pos.row(idx) = pos.row(idx)-1;
                        else
                            if(obstacle_withScale(pos.row(idx),pos.col(idx)+1)==0)
                                pos.col(idx) = pos.col(idx)+1;
                            else
                                if(obstacle_withScale(pos.row(idx),pos.col(idx)-1)==0)
                                    pos.col(idx) = pos.col(idx)-1;
                                end
                            end
                        end
                    end
                    state(idx) = 4; % 4: in boundary              
                end  
            case 6 % 6: safe (leaving F)
                [map_path{idx} pos.row(idx) pos.col(idx)]...
                    = moveToTarget(goal.row(idx),goal.col(idx),pos.row(idx),pos.col(idx),map_path{idx});
                row_distance = abs(goal.row(idx)-pos.row(idx));
                col_distance = abs(goal.col(idx)-pos.col(idx));
                if(row_distance~=0 || col_distance~=0) % far
                    state(idx) = 7; % 7: far (leaving F)
                else % near
                    idx_leave(end+1) = idx; % end 
                end               
            case 7 % 7: far (leaving F)
                % compute obstacles
                obstacle_people = (obstacle~=obstacle)+255;
                for id_p=1:people_num
                    if(id_p~=idx)
                        obstacle_people(pos.row(idx),pos.col(idx)) = 0;
                    end
                end
                % try to move
                move = tryToMoveToTarget(goal.row(idx),goal.col(idx),pos.row(idx),pos.col(idx),map_path{idx},obstacle_people,scale);
                if(strcmp(move,'safe')==1)
                    state(idx) = 6; % 6: safe (leaving F)
                end
                if(strcmp(move,'collided')==1)
                    state(idx) = 7; % 7: far (leaving F)
                end               
            case 8 % 8: far (interested)
                % compute obstacles
                obstacle_people = (obstacle~=obstacle)+255;
                for id_p=1:people_num
                    if(id_p~=idx)
                        obstacle_people(pos.row(idx),pos.col(idx)) = 0;
                    end
                end
                % try to move
                move = tryToMoveToTarget(goal.row(idx),goal.col(idx),pos.row(idx),pos.col(idx),map_path{idx},obstacle_people,scale);
                if(strcmp(move,'safe')==1)
                    state(idx) = 10; % 10: safe (interested)
                end
                if(strcmp(move,'collided')==1)
                    state(idx) = 8; % 8: far (interested)
                end
            case 9 % 9: near (interested)
                lambda = decay_factor(idx);
                P_decay = exp(-1*lambda*stopTime(idx));
                dice = rand(1);
                if(dice<=P_decay)
                    stopTime(idx) = stopTime(idx) + 1;
                    state(idx) = 9; % 9: near (interested)
                else
                    stopTime(idx) = inf;
                    map_path{idx} = inf;
                    state(idx) = 4; % 4: in boundary
                end
            case 10 % 10: safe (interested)
                [map_path{idx} pos.row(idx) pos.col(idx)]...
                    = moveToTarget(goal.row(idx),goal.col(idx),pos.row(idx),pos.col(idx),map_path{idx});
                row_distance = abs(goal.row(idx)-pos.row(idx));
                col_distance = abs(goal.col(idx)-pos.col(idx));
                elements = near_threshold_least:near_threshold_most;
                dice = ceil(rand(1)*numel(elements));
                threshold = elements(dice);
                if(obstacle_withScale(pos.row(idx),pos.col(idx))==0)
                    if(row_distance>threshold || col_distance>threshold) % far
                        state(idx) = 8; % 8: far (interested)
                    else % near
                        stopTime(idx) = 0;
                        state(idx) = 9; % 9: near (interested)                
                    end
                else
                    % move back to the place with no obstacle
                    if(obstacle_withScale(pos.row(idx)+1,pos.col(idx))==0)
                        pos.row(idx) = pos.row(idx)+1;
                    else
                        if(obstacle_withScale(pos.row(idx)-1,pos.col(idx))==0)
                            pos.row(idx) = pos.row(idx)-1;
                        else
                            if(obstacle_withScale(pos.row(idx),pos.col(idx)+1)==0)
                                pos.col(idx) = pos.col(idx)+1;
                            else
                                if(obstacle_withScale(pos.row(idx),pos.col(idx)-1)==0)
                                    pos.col(idx) = pos.col(idx)-1;
                                end
                            end
                        end
                    end
                    % update state
                    stopTime(idx) = 0;
                    state(idx) = 9; % 9: near (interested)                    
                end
        end % end of switch
        temp_floorplan(pos.row(idx)-scale:pos.row(idx)+scale,pos.col(idx)-scale:pos.col(idx)+scale,1) = 0; % red
        temp_floorplan(pos.row(idx)-scale:pos.row(idx)+scale,pos.col(idx)-scale:pos.col(idx)+scale,2) = 50; % green
        temp_floorplan(pos.row(idx)-scale:pos.row(idx)+scale,pos.col(idx)-scale:pos.col(idx)+scale,3) = 255; % blue
        density(pos.row(idx)-scale:pos.row(idx)+scale,pos.col(idx)-scale:pos.col(idx)+scale)...
            = density(pos.row(idx)-scale:pos.row(idx)+scale,pos.col(idx)-scale:pos.col(idx)+scale) + 1;
    end % end of for idx=1:people_num   
    % render the floorplan
    figure(1)
    imshow(temp_floorplan)
    title(sprintf('%s%d','Floorplan, time=',t))
    % render the density map
    if(mod(t,500)==0)
        figure(2)
        temp_density = density;
        temp_density_max = max(max(temp_density));
        temp_density = temp_density/temp_density_max;
        imshow(temp_density)
        colormap(hot)
        title(sprintf('%s%d','Density, time=',t))
    end
    % delete people
    while numel(idx_leave)~=0
        idx = max(idx_leave);
        idx_leave(idx_leave==idx) = [];
        decay_factor(idx) = [];
        pr_searchTarget(idx) = [];
        pr_searchBoundary(idx) = [];
        goal.row(idx) = [];
        goal.col(idx) = [];
        pos.row(idx) = [];
        pos.col(idx) = [];
        stopTime(idx) = [];
        map_display_remain(idx) = [];
        map_display(idx) = [];
        map_region_remain(idx) = [];
        map_boundary_remain(idx) = [];
        map_path(idx) = [];
        state(idx) = [];
        people_num = people_num - 1;
    end     
    % randomly add people
    if(people_num==0 || (mod(t,add_people_time)==0 && people_num<add_people_num_threshold))
        random_add = randi([1 max_people_num_each],1,1);
        add_people_num = add_people_num + random_add;
    end
    blockR = temp_floorplan(start.row-scale*4:start.row+scale*4,start.col-scale*4:start.col+scale*4,1);
    blockG = temp_floorplan(start.row-scale*4:start.row+scale*4,start.col-scale*4:start.col+scale*4,2);
	blockB = temp_floorplan(start.row-scale*4:start.row+scale*4,start.col-scale*4:start.col+scale*4,3);
    if(add_people_num>0 && min(min(blockR))==255 && min(min(blockG))==255 && min(min(blockB))==255)
        add_people_num = add_people_num - 1;
        pr_searchTarget(end+1) = randi([search_low search_high])/search_high;
        pr_searchBoundary(end+1) = randi([search_low search_high])/search_high;
        decay_factor(end+1) = 1/randi([decay_low decay_high]);
        goal.row(end+1) = inf;
        goal.col(end+1) = inf;
        pos.row(end+1) = start.row;
        pos.col(end+1) = start.col;
        stopTime(end+1) = inf;
        map_display_remain{end+1} = inf;
        map_display{end+1} = inf;
        map_region_remain{end+1} = region;
        map_boundary_remain{end+1} = boundary;
        map_path{end+1} = inf; 
        state(end+1) = 2; % starting state
        people_num = people_num + 1;
    end
end % end of while loop

