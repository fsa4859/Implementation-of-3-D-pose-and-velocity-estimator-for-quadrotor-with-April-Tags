function [position, orientation] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
% individual april tag
% all april tags
imag_Cord=ones(3,4*length(data(t).id));
Normalized_Image_Cord=ones(3,4*length(data(t).id));


K=[311.0520, 0 201.8724;0 311.3885 113.6210;0 0 1];
[world_Cord]=getCorner(data(t).id); % get all april tags.
A=ones(8*length(data(t).id),9); % all april tags.
rot_Matrix_mod=rotx(180)*rotz(45);
% get image coordinates for all april tags
%for i=k:4*k
for k=1:length(data(t).id)
    imag_Cord(1,4*k-3)=data(t).p4(1,k);
    imag_Cord(2,4*k-3)=data(t).p4(2,k);
    imag_Cord(1,4*k-2)=data(t).p3(1,k);
    imag_Cord(2,4*k-2)=data(t).p3(2,k);
    imag_Cord(1,4*k-1)=data(t).p1(1,k);
    imag_Cord(2,4*k-1)=data(t).p1(2,k);
    imag_Cord(1,4*k)=data(t).p2(1,k);
    imag_Cord(2,4*k)=data(t).p2(2,k);
end

% multiply pixel coordinates by intrinsic matrix K
Normalized_Image_Cord=inv(K)*imag_Cord;

% calculate A matrix
for i=1:8*length(data(t).id)
    if rem(i,2)==0
        A(i,1:3)=0;
        A(i,4)=world_Cord(1,i/2);
        A(i,5)=world_Cord(2,i/2);
        A(i,6)=1;
        A(i,7)=-Normalized_Image_Cord(2,i/2)*world_Cord(1,i/2);
        A(i,8)=-Normalized_Image_Cord(2,i/2)*world_Cord(2,i/2);
        A(i,9)=-Normalized_Image_Cord(2,i/2);
    else
        A(i,1)=world_Cord(1,(i+1)/2);
        A(i,2)=world_Cord(2,(i+1)/2);
        A(i,3)=1;
        A(i,4:6)=0;
        A(i,7)=-Normalized_Image_Cord(1,(i+1)/2)*world_Cord(1,(i+1)/2);
        A(i,8)=-Normalized_Image_Cord(1,(i+1)/2)*world_Cord(2,(i+1)/2);
        A(i,9)=-Normalized_Image_Cord(1,(i+1)/2);
    end
end
A;


[U4,S4,V4]=svd(A);
H_prime=V4(:,9);
H_L_prime=[H_prime(1,:) H_prime(2,:) H_prime(3,:);H_prime(4,:) H_prime(5,:) H_prime(6,:);H_prime(7,:) H_prime(8,:) H_prime(9,:)];
H_L_prime=H_L_prime*sign(transpose(world_Cord(:,1))*H_L_prime*Normalized_Image_Cord(:,1));
R1_hat_prime=H_L_prime(:,1);
R2_hat_prime=H_L_prime(:,2);
t_hat_prime=H_L_prime(:,3);
hat_matrix_prime=[R1_hat_prime,R2_hat_prime,cross(R1_hat_prime,R2_hat_prime)];
[U5,S5,V5]=svd(hat_matrix_prime);
R_prime=U5*[1 0 0;0 1 0;0 0 det(U5*V5)]*V5;
tra=t_hat_prime/norm(R1_hat_prime);


Rbw=rot_Matrix_mod*R_prime;

eulzyx=rotm2eul(transpose(Rbw));
orientation=eulzyx;


t_hat_new=[0.04*cos(pi/4);-0.04*sin(pi/4);-0.03]+rot_Matrix_mod*tra;
homog_matrix(1:3,1:3)=Rbw;
homog_matrix(1:3,4)=t_hat_new;
homog_matrix(4,:)=[0 0 0 1];
homog_matrix;
position=-transpose(Rbw)*t_hat_new;


%orientation=min(R,U2*S2*V2);

    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX
end