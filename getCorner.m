function [world_Cord] = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    % preallocation of world coordinates matrix
    world_Cord=zeros(3,length(id)*4);
    % create the array of all IDs of April tags
    tag_ids=[0,12,24,36,48,60,72,84,96;
        1,13,25,37,49,61,73,85,97;
        2,14,26,38,50,62,74,86,98;
        3,15,27,39,51,63,75,87,99;
        4,16,28,40,52,64,76,88,100;
        5,17,29,41,53,65,77,89,101;
        6,18,30,42,54,66,78,90,102;
        7,19,31,43,55,67,79,91,103;
        8,20,32,44,56,68,80,92,104;
        9,21,33,45,57,69,81,93,105;
        10,22,34,46,58,70,82,94,106;
        11,23,35,47,59,71,83,95,107];

for i=1:length(id)
    tag=id(1,i);
    [row,col]=find(tag_ids==tag);
    %for k=i+3*(i-1):4*i
    if col >3 && col<7
        % y top left point
        world_Cord(2,i+3*(i-1))=(col-1)*0.152+(col-2)*0.152+0.178; % calc y topleft

    elseif col>=7
        world_Cord(2,i+3*(i-1))=(col-1)*0.152+(col-3)*0.152+2*0.178;% ytopleft

    else
        world_Cord(2,i+3*(i-1))=(col-1)*0.152*2;% ytopleft

    end

    world_Cord(1,i+3*(i-1))=2*(row-1)*0.152; % xtopleft
    world_Cord(1,1+i+3*(i-1))=world_Cord(1,i+3*(i-1)); % x topright
    world_Cord(2,1+i+3*(i-1))=world_Cord(2,i+3*(i-1))+0.152; % y topright
    world_Cord(1,2+i+3*(i-1))=world_Cord(1,i+3*(i-1))+0.152; % x bottom left
    world_Cord(2,2+i+3*(i-1))=world_Cord(2,i+3*(i-1)); %y bottom left
    world_Cord(1,3+i+3*(i-1))=world_Cord(1,2+i+3*(i-1)); % x bottom right
    world_Cord(2,3+i+3*(i-1))=world_Cord(2,1+i+3*(i-1)); % y bottom right
    
end
       
       
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method

    world_Cord;
end

