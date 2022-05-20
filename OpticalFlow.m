close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
K=[311.0520 0 201.8724;0 311.3885 113.6210;0 0 1];

for r=2:length(sampledData)
    dt(1,r)=sampledData(r).t-sampledData(r-1).t;
end

dt_new=lowpass(dt,100,1000);

for n = 2:length(sampledData)
    [position, orientation,R_c2w]=estimatePose(sampledData,n);

    R_c2w_2=R_c2w(1:3,1:3);
    
    I=sampledData(n-1).img;
    corners=detectFASTFeatures(I);
    out=corners.selectStrongest(200).Location;

    pointTracker = vision.PointTracker;
    initialize(pointTracker,out,I);
    %% Find the location of the next points;
    [points]=pointTracker(sampledData(n).img);

    % add third column for multiplying by inverse k
    points(:,3)=1;
    out(:,3)=1;

    % find the difference
    diff_1=transpose(points)-transpose(out);

    % calibrated image frame
    diff_calib=inv(K)*diff_1;

    vel_cal=diff_calib(1:2,:)/(sampledData(n).t-sampledData(n-1).t);
    vel=transpose(vel_cal);
    

    [row_one,col_one]=size(points);

    points_calc=inv(K)*transpose(points);
    points_A=transpose(points_calc);
    
    % depth
  

    for u=1:row_one
        knowns=[points_A(u,1) -R_c2w(1,1) -R_c2w(1,2);
            points_A(u,2) -R_c2w(2,1) -R_c2w(2,2);
            1 -R_c2w(3,1) -R_c2w(3,2)];
        results=inv(knowns)*position;
        d(u,1)=results(1,1);
    end
  
   l=1:row_one;
   A(2*l-1,1)=-1*(1/position(3,1));
   B(2*l-1,1)=points_A(l,1).*points_A(l,2);
   A(2*l-1,2)=0;
   B(2*l-1,2)=-1-points_A(l,1).*points_A(l,1);
   A(2*l-1,3)=(1/position(3,1)*points_A(l,1));% even rows are neg, odd positive
   B(2*l-1,3)=points_A(l,2);
   A(2*l,1)=0;
   B(2*l,1)=1+points_A(l,2).*points_A(l,2);
   A(2*l,2)=-1.*(1/position(3,1));
   B(2*l,2)=-points_A(l,1).*points_A(l,2);
   A(2*l,3)=(1/position(3,1)*points_A(l,2));
   B(2*l,3)=-points_A(l,1);
   points_vel(2*l-1,1)=vel(l,1);
   points_vel(2*l,1)=vel(l,2);


   H=horzcat(A,B);
   output=pinv(H)*points_vel;


   %[output]=velocityRANSAC(vel,points_A,position(3,1),R_c2w,0.9);

   new(1:3,:)=inv(R_c2w_2)*output(1:3,1);
   new(4:6,:)=inv(R_c2w_2)*output(4:6,1);

   estimatedV(:,n)=new;


end


estimatedV(1,:)=lowpass(estimatedV(1,:),100,1000);
estimatedV(2,:)=lowpass(estimatedV(2,:),100,1000);
estimatedV(3,:)=lowpass(estimatedV(3,:),100,1000);
estimatedV(4,:)=lowpass(estimatedV(4,:),100,1000);
estimatedV(5,:)=lowpass(estimatedV(5,:),100,1000);
estimatedV(6,:)=lowpass(estimatedV(6,:),100,1000);

plotData(estimatedV, sampledData, sampledVicon, sampledTime,datasetNum)





