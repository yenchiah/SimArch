%% move towards the target
function move = tryToMoveToTarget( goal_row,goal_col,pos_row,pos_col,map_path,obstacle_people,scale )   
    move = 'collided';
    pos_row_new = inf;
    pos_col_new = inf;
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
    if(pos_row_new~=inf && pos_col_new~=inf) % there exits a path
%         block = obstacle_people(pos_row_new-scale:pos_row_new+scale,pos_col_new-scale:pos_col_new+scale)
        block = obstacle_people(pos_row_new,pos_col_new);
        if(min(min(block))==255) % empty
            move = 'safe';
        end
    end
%     figure(1)
%     imshow(map_path+(1-(obstacle_people/255)))
%     colormap(hot)    
end

