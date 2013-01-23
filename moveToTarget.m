%% move towards the target
function [ map_path pos_row_new pos_col_new ] = moveToTarget( goal_row,goal_col,pos_row,pos_col,map_path )   
%     figure(2)
%     imshow(map_path)
%     colormap(hot)
    pos_row_new = inf;
    pos_col_new = inf;
    map_path(pos_row,pos_col) = 0;
    if(map_path(pos_row+1,pos_col)==0.4) % right
        pos_row_new = pos_row+1;
        pos_col_new = pos_col;
    end
    if(map_path(pos_row-1,pos_col)==0.4) % left
        pos_row_new = pos_row-1;
        pos_col_new = pos_col;
    end
    if(map_path(pos_row,pos_col+1)==0.4) % up
        pos_row_new = pos_row;
        pos_col_new = pos_col+1;
    end
    if(map_path(pos_row,pos_col-1)==0.4) % down
        pos_row_new = pos_row;
        pos_col_new = pos_col-1;
    end
end

