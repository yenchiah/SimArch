%% search an interesting display (weighted A* search)
function [ boundary_new_remain region_new_remain display_new display_new_remain map_path goal_row goal_col ] = searchBoundary( pos_row,pos_col,obstacle,region_remain,boundary_remain,display_all,scale )
    % the weight factor of weighted A* search
    weight = 10;
    heuristic_noise_variance = 1000;
    
    % randomly find boundary as a goal
    [rows cols] = size(boundary_remain);
    row_distance = repmat((1:rows)',1,cols);
    row_distance = abs(row_distance - pos_row);
    col_distance = repmat(1:cols,rows,1);
    col_distance = abs(col_distance - pos_col);
    distance = row_distance + col_distance;
    distance(boundary_remain==255) = inf;
    [g_row g_col] = find(distance==min(min(distance)));
    dice = ceil(rand(1)*numel(g_row));
    g_row = g_row(dice);
    g_col = g_col(dice);    
    goal_row = g_row;
    goal_col = g_col;
    boundary_new_remain = boundary_remain;
    target = boundary_remain(g_row,g_col);
    boundary_new_remain(boundary_remain==target) = 255;
    region_selected = region_remain;
    region_selected(region_remain~=target) = 255;
    region_new_remain = region_remain;
    region_new_remain(region_remain==target) = 255;
    display_new = display_all;
    display_new(region_selected==255) = 255;
    display_new_remain = display_all;
    display_new_remain(region_selected~=255) = 255;
    
    % read all variables
    s_col = pos_col; % start column
    s_row = pos_row; % start row
    obstacle = double(obstacle);
    map = 1-obstacle/max(max(obstacle));
    map_optimal = map;
    map(boundary_remain==target) = 0;
    filter = ones(1+scale*2);
    map = conv2(map,filter,'same');
    map(map~=0) = 1;
    map(boundary_remain==target) = 0;
    map_from_row = zeros(rows,cols);
    map_from_row = map_from_row+inf;
    map_from_col = zeros(rows,cols);
    map_from_col = map_from_col+inf;

    % heuristic function
    [map_row map_col] = find(map==map);
    map_col = reshape(map_col,rows,cols);
    map_row = reshape(map_row,rows,cols);
    map_col = abs(map_col - g_col);
    map_row = abs(map_row - g_row);
    map_heu = weight*(map_col+map_row)+rand(rows,cols)*heuristic_noise_variance;
%     figure(3)
%     temp_map_heu = map_heu;
%     temp_map_heu_max = max(max(temp_map_heu));
%     temp_map_heu = temp_map_heu/temp_map_heu_max;
%     temp_map_heu(map==1) = 1;
%     imshow(temp_map_heu)
%     colormap(hot)
%     pause

    % A* search, set open set
    row_array = [s_row];
    col_array = [s_col];
    h_array = [map_heu(s_row,s_col)]; % estimate of remaining work
    g_array = [0]; % work done so far (energy)
    counterVisit = 0;
    counterExplore = 0;
    
    while true      
        % selec the node with the lowest cost
        f_array = g_array + h_array;
        min_idx = find(f_array==min(f_array));
        % randomly select from all nodes with the same lowest f value
        same_num = numel(min_idx);
        min_idx = min_idx(ceil(same_num*rand(1)));
        % get values
        now_row = row_array(min_idx);
        now_col = col_array(min_idx);
        energy = g_array(min_idx);
        % check goal
        if(now_row==g_row && now_col==g_col)
            break;
        end
        % draw on map and mark as close set
        close = 0.4; % red color
        counterVisit = counterVisit+1;
        map(now_row,now_col) = close;
        % remove the node from waiting list
        g_array(min_idx) = [];
        h_array(min_idx) = [];
        row_array(min_idx) = [];
        col_array(min_idx) = [];
        % explore new nodes
        open = 0.8; % yellow color
        if(now_row+1>=1 && now_row+1<=rows && map(now_row+1,now_col)==0) % up node
            % add to open set
            row_array(end+1) = now_row+1;
            col_array(end+1) = now_col;
            h_array(end+1) = map_heu(now_row+1,now_col);
            g_array(end+1) = energy+1;
            map(now_row+1,now_col) = open;
            counterExplore = counterExplore+1;
            % construct path
            map_from_row(now_row+1,now_col) = now_row;
            map_from_col(now_row+1,now_col) = now_col;                    
        end
        if(now_row-1>=1 && now_row-1<=rows && map(now_row-1,now_col)==0) % down node
            row_array(end+1) = now_row-1;
            col_array(end+1) = now_col;
            h_array(end+1) = map_heu(now_row-1,now_col);
            g_array(end+1) = energy+1;
            map(now_row-1,now_col) = open;
            counterExplore = counterExplore+1;
            % construct path
            map_from_row(now_row-1,now_col) = now_row;
            map_from_col(now_row-1,now_col) = now_col;                     
        end
        if(now_col+1>=1 && now_col+1<=cols && map(now_row,now_col+1)==0) % right node
            row_array(end+1) = now_row;
            col_array(end+1) = now_col+1;
            h_array(end+1) = map_heu(now_row,now_col+1);
            g_array(end+1) = energy+1;
            map(now_row,now_col+1) = open;
            counterExplore = counterExplore+1;
            % construct path
            map_from_row(now_row,now_col+1) = now_row;
            map_from_col(now_row,now_col+1) = now_col;                       
        end        
        if(now_col-1>=1 && now_col-1<=cols && map(now_row,now_col-1)==0) % left node
            row_array(end+1) = now_row;
            col_array(end+1) = now_col-1;
            h_array(end+1) = map_heu(now_row,now_col-1);
            g_array(end+1) = energy+1;
            map(now_row,now_col-1) = open;
            counterExplore = counterExplore+1;
            map_from_row(now_row,now_col-1) = now_row;
            map_from_col(now_row,now_col-1) = now_col;                      
        end
%             % show the searching process
%             figure(22) 
%             imshow(map)
%             colormap(hot)
    end
    
    % draw optimal path from goal state
    op_row = g_row;
    op_col = g_col;
    temp_map_optimal = map_optimal;
    while true
        if(op_row==s_row & op_col==s_col) break; end   
        map_optimal(op_row,op_col) = 0.4;
        next_row = map_from_row(op_row,op_col);
        next_col = map_from_col(op_row,op_col);
        op_row = next_row;
        op_col = next_col;                                    
    end
    map_optimal(temp_map_optimal==1) = 1;
    map_path = map_optimal;
    
%     % images
%     figure(30)
%     imshow(map_optimal)
%     title(sprintf('weighted A* optimal path, weight=%2.1f',weight))
%     colormap(hot)  
%     print(figure(30),'-r150', '-dpng', sprintf('search_weight=%d_optimalPath.png',weight));
%     pause;
%     % save image
%     figure(29)
%     imshow(map)
%     title(sprintf('weighted A* search, weight=%2.1f',weight))
%     colormap(hot)
%     print(figure(29),'-r150', '-dpng', sprintf('search_weight=%d.png',weight));

end

