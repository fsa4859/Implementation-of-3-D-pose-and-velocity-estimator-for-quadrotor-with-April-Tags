function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter   
    three_points_pos=ones(3,2);
    three_points_vel=ones(6,1);
    A_Ran=ones(6,3);
    B_Ran=ones(6,3);
    H=ones(6,6);

    k=log(1-0.99)/log(1-e^3); % required no of iterations
    for s=1:round(k)

        three_points_pos=optPos(s:s+2,1:2); % select three points
        three_points_vel=[optV(s,1);optV(s,2);optV(s+1,1);optV(s+1,2);
            optV(s+2,1);optV(s+2,2)];
        % calculate A matrix for three points

        A_Ran=1/Z*[-1 0 three_points_pos(1,1);0 -1 three_points_pos(1,2);
            -1 0 three_points_pos(2,1);0 -1 three_points_pos(2,2);
            -1 0 three_points_pos(3,1);0 -1 three_points_pos(3,2)];

        % calculate the B matrix

        B_Ran=[three_points_pos(1,1)*three_points_pos(1,2),-1-three_points_pos(1,1)^2,three_points_pos(1,2);
            1+three_points_pos(1,2)^2,-three_points_pos(1,1)*three_points_pos(1,2),-three_points_pos(1,1);
            three_points_pos(2,1)*three_points_pos(2,2),-1-three_points_pos(2,1)^2,three_points_pos(2,2);
            1+three_points_pos(2,2)^2,-three_points_pos(2,1)*three_points_pos(2,2),-three_points_pos(2,1);
            three_points_pos(3,1)*three_points_pos(3,2),-1-three_points_pos(3,1)^2,three_points_pos(3,2);
            1+three_points_pos(3,2)^2,-three_points_pos(3,1)*three_points_pos(3,2),-three_points_pos(3,1)];

        % get H matrix and initial estimates of velocity
        H=horzcat(A_Ran,B_Ran);
        Out_Ran=pinv(H)*three_points_vel;

        % now get the A and B matrices for all the points
        [row_ran,col_ran]=size(optV);
        tic;
        f=1:row_ran;
        A_all(2*f-1,1)=-1*(1/Z);
        B_all(2*f-1,1)=optPos(f,1).*optPos(f,2);
        A_all(2*f-1,2)=0;
        B_all(2*f-1,2)=-1-optPos(f,1).*optPos(f,1);
        A_all(2*f-1,3)=optPos(f,1).*(1/Z);
        B_all(2*f-1,3)=optPos(f,2);
        A_all(2*f,1)=0;
        B_all(2*f,1)=1+optPos(f,2).*optPos(f,2);
        A_all(2*f,2)=-1.*(1/Z);
        B_all(2*f,2)=-optPos(f,1).*optPos(f,2);
        A_all(2*f,3)=optPos(f,2).*(1/Z);
        B_all(2*f,3)=-optPos(f,1);
       
   
        H_all=horzcat(A_all,B_all);

        vel_points_estimates=H_all*Out_Ran;

        % reshape the actual velocity points to get differnce
        j=1:row_ran;
        vel_points_actual(2*j-1,1)=optV(j,1);
        vel_points_actual(2*j,1)=optV(j,2);

        

        % calculate the difference
        for u=1:row_ran
            diff_x=vel_points_estimates(2*u-1,1)-vel_points_actual(2*u-1,1);
            diff_y=vel_points_estimates(2*u,1)-vel_points_actual(2*u,1);
            if abs(diff_x)<0.1 && abs(diff_y)<0.1
                inliers(u,1)=1;
            else
                inliers(u,1)=0;
            end
        end
        count_inliers(s,1)=sum(inliers(:)==1);

     if s==1
         find_1=find(inliers);
     elseif s==2
         find_2=find(inliers);
     elseif s==3
         find_3=find(inliers);
     else 
         find_4=find(inliers);
     end
         
    end


    % now find the maximum number of inliers
    % use the inliers to compute the velocity

    [val,idx]=max(count_inliers);
    if idx==1
        inliers_indices=find_1;
    elseif idx==2
        inliers_indices=find_2;
    elseif idx==3
        inliers_indices=find_3;
    else
        inliers_indices=find_4;
    end
    % get the infices of inliers
    %inliers_indices=indices_vector(:,idx);
    % form the new points and new velocities for inliers
    j=1:length(inliers_indices);
    inliers_points(j,1)=optPos(inliers_indices(j,1),1);
    inliers_points(j,2)=optPos(inliers_indices(j,1),2);
    inliers_vel(j,1)=optV(inliers_indices(j,1),1);
    inliers_vel(j,2)=optV(inliers_indices(j,1),2);


    [row_fin,col_fin]=size(inliers_points);

    y=1:row_fin;
    A_fin(2*y-1,1)=-1*(1/Z);
    B_fin(2*y-1,1)=inliers_points(y,1).*inliers_points(y,2);
    A_fin(2*y-1,2)=0;
    B_fin(2*y-1,2)=-1-inliers_points(y,1).*inliers_points(y,1);
    A_fin(2*y-1,3)=inliers_points(y,1).*(1/Z);
    B_fin(2*y-1,3)=inliers_points(y,2);
    A_fin(2*y,1)=0;
    B_fin(2*y,1)=1+inliers_points(y,2).*inliers_points(y,2);
    A_fin(2*y,2)=-1.*(1/Z);
    B_fin(2*y,2)=-inliers_points(y,1).*inliers_points(y,2);
    A_fin(2*y,3)=inliers_points(y,2).*(1/Z);
    B_fin(2*y,3)=-inliers_points(y,1);
    points_vel_fin(2*y-1,1)=inliers_vel(y,1);
    points_vel_fin(2*y,1)=inliers_vel(y,2);


    H_fin=horzcat(A_fin,B_fin);
    Vel=pinv(H_fin)*points_vel_fin;
    
    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
end