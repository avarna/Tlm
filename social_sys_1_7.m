% Traffic Dynamics
clear all
close all
clc
cc=0;
%% Defining constants
L=400; % length of 2 lane highway, L=400 m
x0=300; % Position of stopped car
vmax=2; % in m/s
Xc=4; % Safety distance
a=1; % Sensitivity
inflow1=4; % Inflow of cars on lane 1-distance between new car and next car, in meters
inflow2=8; % Inflow of cars on lane 2

%% Control parameters
deltaT=.1; % timestep
n_time_steps=3000; % number of time steps
t=0:deltaT:n_time_steps*deltaT;

%to calculate density
delx=10; 
x=0:delx:L; 
nx=L/delx+1;
%Pre allocating
density1=zeros(nx,1);
density2=zeros(nx,1);

n1=L/2/inflow1+1; % number of cars in lane 1 (right lane)
n2=L/2/inflow2+1; % number of cars in lane 2 (left lane)

%% Initial conditions
%Initial position of cars
x1=0:inflow1:L/2;
x1=[x1' x1']; %lane 1 cars-Right lane
x2=0:inflow2:L/2; %lane 2 cars-Left lane
x2=[x2' x2'];

%Initial velocity of the cars
v1=zeros(n1,2); v1(n1,1:2)=vmax;
v2=zeros(n2,2); v2(n2,1:2)=vmax;

color1=zeros(n1,1); % For animation
color2=ones(n2,1);

rg1=zeros(n1,1); % For rogue behavior
rg2=zeros(n2,1);

rg1(20,1)=1;%10- 20th driver  in lane 1 are rogue


timcount=0;
flowcount=0;

%% Time loop
for k=2:n_time_steps
    
    count1=0; % no of cars which changed lane 1
    count2=0; % no of cars which changed lane 2
    
    timcount=timcount+1;
    
    
%% Optimal velocity model to lane 1 cars
    for i=1:n1-1
        
        if(rg1(i)==1)
            a=0.5;
            vmax=2; % in m/s
            Xc=3;
        else 
            a=2;
            vmax=2; % in m/s
            Xc=4;
        end
        
        delX=x1(i+1,k)-x1(i,k); % Headway of car i
        Vopt=vmax/2*(tanh(delX-Xc)+tanh(Xc)); % Optimal velocity
        x1(i,k+1)=(a*Vopt*deltaT^2 + x1(i,k)*(2+a*deltaT) - x1(i,k-1)) / (1+a*deltaT); %position
        v1(i,k+1)=(x1(i,k+1) - x1(i,k))/deltaT; %Velocity
        
        %Accident at x0=300m
        if((x0-Xc)<x1(i,k+1) && x1(i,k+1)<x0 && (timcount<1500))
            x1(i,k+1)=x1(i,k);%Car stops, v1(i,k+1)=0
        end
    end


    %Accident at x0=300m(condition check for the 'n1' th car)
    if((x0-Xc)<x1(n1,k) && x1(n1,k)<x0 && (timcount<1500))
        x1(n1,k+1)=x1(n1,k);%Car stops, v1(n1,k+1)=0
    else
        x1(n1,k+1)=x1(n1,k)+vmax*deltaT;
        v1(n1,k+1)=vmax;
    end
    
    %Inflow of cars
    if(x1(1,k+1)>inflow1)
       n1=n1+1;
       for j=n1:-1:2
           x1(j,k+1)=x1(j-1,k+1);
           v1(j,k+1)=v1(j-1,k+1);
           x1(j,k)=x1(j-1,k);  % initial condtions
           v1(j,k)=v1(j-1,k); 
           color1(j,1)=color1(j-1,1);
           rg1(j,1)=rg1(j-1,1);
       end
       x1(1,k+1)=0; %adding new car
       v1(1,k+1)=vmax;
       x1(1,k)=0; 
       v1(1,k)=vmax;  
       color1(1,1)=0;
       rg1(1,1)=0;
    end

%% Optimal velocity model to lane 2 cars    
    for i=1:n2-1
        
        if(rg2(i)==1)
            a=0.5;
            vmax=2; % in m/s
            Xc=3;
        else 
            a=2;
            vmax=2; % in m/s
            Xc=4;
        end
        
        delX=x2(i+1,k)-x2(i,k); % Headway of car i
        Vopt=vmax/2*(tanh(delX-Xc)+tanh(Xc));
        x2(i,k+1)=(a*Vopt*deltaT^2 + x2(i,k)*(2+a*deltaT) - x2(i,k-1)) / (1+a*deltaT); %position
        v2(i,k+1)=(x2(i,k+1) - x2(i,k))/deltaT; %Velocity
    end
    
    x2(n2,k+1)=x2(n2,k)+vmax*deltaT;
    v2(n2,k+1)=vmax;
    
    %Inflow of cars
    if(x2(1,k+1)>inflow2)
       n2=n2+1;
       for j=n2:-1:2
           x2(j,k+1)=x2(j-1,k+1);
           v2(j,k+1)=v2(j-1,k+1);
           x2(j,k)=x2(j-1,k);
           v2(j,k)=v2(j-1,k);
           color2(j,1)=color2(j-1,1);
           rg2(j,1)=rg2(j-1,1);
       end
       x2(1,k+1)=0; %adding new car
       v2(1,k+1)=vmax;
       x2(1,k)=0; 
       v2(1,k)=vmax;
       color2(1,1)=1;
       rg2(1,1)=0;
    end
 
%% lane changing for lane 1(right) cars
    for i=1:n1-1      
     if(x1(i,k+1)>100)
        delX=x1(i+1,k+1)-x1(i,k+1); % Headway of car i
            
        [headway,ind]=min(abs(x2(:,k+1)-x1(i,k+1))); % Headway between car i and the car in target lane
        
        flag1=0;
        
        if ((x2(ind,k+1)-x1(i,k+1))>0)
            delXf=headway; % Headway between car i and the front car in target lane
            
            if(ind==1)
                delXb=10*Xc; %Assigning large value-because there is no car behind
            else
                delXb=x1(i,k+1)-x2(ind-1,k+1); % Headway between car i and the back car in target lane
            end
            flag1=1;
        else
            
            if(ind>=n2)
                delXf=10*Xc; %Assigning large value-because there is no car in the front
            else
                delXf=x2(ind+1,k+1)-x1(i,k+1); % Headway between car i and the front car in target lane
            end
            
            delXb=headway; % Headway between car i and the back car in target lane
       
        end
        
   
        %Lane changing Rules
        if ((v1(i,k+1)>1.02*v1(i+1,k+1) && delX<4*Xc && delXf>2*Xc && delXb>Xc)|| (delX<2*Xc && delXf>delX && delXb>Xc))
            n2=n2+1;  % number of cars on lane 2(n2) is increased by 1 as lane1 car enters lane2
           
            if(flag1==1)
                for j=(n2-1):-1:ind
                    x2(j+1,k+1)=x2(j,k+1); % Shifting the cars forward-assigning new positions on lane 2
                    v2(j+1,k+1)=v2(j,k+1);
                    
                    x2(j+1,k)=x2(j,k); % this is done as the difference technique needs 2 previous time step values to calculate
                    v2(j+1,k)=v2(j,k);
                    color2(j+1,1)=color2(j,1);
                    rg2(j+1,1)=rg2(j,1);
                end
            
                x2(ind,k+1)=x1(i,k+1); % Assigning a new 'x2'matrix position to lane 1 car which shifted to lane 2
                v2(ind,k+1)=v1(i,k+1);
                
                x2(ind,k)=x1(i,k);
                v2(ind,k)=v1(i,k);
                color2(ind,1)=color1(i,1);
                rg2(ind,1)=rg1(i,1);
            else
                for j=(n2-1):-1:(ind+1)
                    x2(j+1,k+1)=x2(j,k+1);
                    v2(j+1,k+1)=v2(j,k+1);
                    
                    x2(j+1,k)=x2(j,k);
                    v2(j+1,k)=v2(j,k);
                    color2(j+1,1)=color2(j,1);
                    rg2(j+1,1)=rg2(j,1);
                end
            
                x2(ind+1,k+1)=x1(i,k+1);
                v2(ind+1,k+1)=v1(i,k+1);
                
                x2(ind+1,k)=x1(i,k);
                v2(ind+1,k)=v1(i,k);
                color2(ind+1,1)=color1(i,1);
                rg2(ind+1,1)=rg1(i,1);
            end
          
            
            for j=i:(n1-1)
                x1(j,k+1)=x1(j+1,k+1);  % Assigning new position to cars in lane 1
                v1(j,k+1)=v1(j+1,k+1);
                
                x1(j,k)=x1(j+1,k);
                v1(j,k)=v1(j+1,k);
                color1(j,1)=color1(j+1,1);
                rg1(j,1)=rg1(j+1,1);
            end
            
            x1(n1,k+1)=NaN;
            n1=n1-1; % Reducing the number of cars on lane 1 by 1.
        end
        
     end 
     end
    
%% lane changing for lane 2(left) cars
    for i=1:n2-1
      if(x2(i,k+1)>100)     
        delX=x2(i+1,k+1)-x2(i,k+1); % Headway of car i
        [headway,ind]=min(abs((x1(:,k+1)-x2(i,k+1)))); % Headway between car i and the car in target lane
        
        flag2=0;
        
        if ((x1(ind,k+1)-x2(i,k+1))>0)
            delXf=headway; % Headway between car i and the front car in target lane
            
            if(ind==1)
                delXb=10*Xc; %Assigning large value-because there is no car behind
            else
                delXb=x2(i,k+1)-x1(ind-1,k+1); % Headway between car i and the back car in target lane
            end
            
            flag2=1;
        else
            
            if(ind>=n1)
                delXf=10*Xc; %Assigning large value-because there is no car in the front
            else
                delXf=x1(ind+1,k+1)-x2(i,k+1); % Headway between car i and the front car in target lane
            end
            
            delXb=headway; % Headway between car i and the back car in target lane
       
        end
        
        %Lane changing Rules
        if (v2(i,k+1)>1.02*v2(i+1,k+1) && delX<4*Xc && delXf>2*Xc && delXb>Xc || (delX<2*Xc && delXf>delX && delXb>Xc))
            n1=n1+1;
            
            if(flag2==1)
                for j=(n1-1):-1:ind
                    x1(j+1,k+1)=x1(j,k+1);
                    v1(j+1,k+1)=v1(j,k+1);
                    
                    x1(j+1,k)=x1(j,k);
                    v1(j+1,k)=v1(j,k);
                    color1(j+1,1)=color1(j,1);
                    rg1(j+1,1)=rg1(j,1);
                end
            
                x1(ind,k+1)=x2(i,k+1);
                v1(ind,k+1)=v2(i,k+1);
                
                x1(ind,k)=x2(i,k);
                v1(ind,k)=v2(i,k);
                color1(ind,1)=color2(i,1);
                rg1(ind,1)=rg2(i,1);
            else
                for j=(n1-1):-1:(ind+1)
                    x1(j+1,k+1)=x1(j,k+1);
                    v1(j+1,k+1)=v1(j,k+1);
                    
                    x1(j+1,k)=x1(j,k);
                    v1(j+1,k)=v1(j,k);
                    color1(j+1,1)=color1(j,1);
                    rg1(j+1,1)=rg1(j,1);
                end
            
                x1(ind+1,k+1)=x2(i,k+1);
                v1(ind+1,k+1)=v2(i,k+1);
                
                x1(ind+1,k)=x2(i,k);
                v1(ind+1,k)=v2(i,k);
                color1(ind+1,1)=color2(i,1);
                rg1(ind+1,1)=rg2(i,1);
            end 
            
            for j=i:(n2-1)
                x2(j,k+1)=x2(j+1,k+1);
                v2(j,k+1)=v2(j+1,k+1);
                
                x2(j,k)=x2(j+1,k);
                v2(j,k)=v2(j+1,k);
                color2(j,1)=color2(j+1,1);
                rg2(j,1)=rg2(j+1,1);
            end
            
            x2(n2,k+1)=NaN;
            n2=n2-1;
        end
      end
    end
cc=cc+1;
%% Animation    
if(cc==5)
cc=0;
    N=L;
    y1=0;
    y2=0.1;
        clf; hold on;
        plot(0:N, y1*(0:N), 'Color', [.75  .75 .75], 'LineWidth', 50)
        hold on;
        plot([0,N] ,[y2,y2] ,'Color', [.75 .75 .75], 'LineWidth', 50)
        hold on;
        plot([0,N] ,[y1,y1],'--w')
        hold on;
        plot([0,N] ,[y2,y2],'--w')
        hold on;
        plot([0,N] ,[0.05,0.05],'w','LineWidth', 1)
        hold on;
        plot([0,N] ,[-0.05,-0.05],'k','LineWidth', 2)
        hold on;
        plot([0,N] ,[0.15,0.15],'k','LineWidth', 2)
        xlim([0 N])
        ylim([-N/800 0.25+N/800])
 
        for i=1:n1
            
            if(rg1(i)==1)
                color1(i)=2;
            end
            
            draw_car(x1(i,k+1), y1, 4,1,color1(i));
        end
        for i=1:n2
            
            if(rg2(i)==1)
                color2(i)=2;
            end
            
            draw_car(x2(i,k+1), y2, 4,1,color2(i));
        end
        pause(.01)
    
end
%% Determining density
    for i=2:nx-1
        count_cars=0;
        for j=1:n1
            if((i*delx-delx/2)<x1(j,k+1) && x1(j,k+1)<(i*delx+delx/2))
                count_cars=count_cars+1;  % counting cars present on the region considered above
            end
        end
        density1(i,k+1)=count_cars/delx; % number of cars per unit decameter
        
        count_cars=0;
        for j=1:n2
            if((i*delx-delx/2)<x2(j,k+1) && x2(j,k+1)<(i*delx+delx/2))
                count_cars=count_cars+1;
            end
        end
        density2(i,k+1)=count_cars/delx;
    end
        
end  % End of TIME loop

%% Saving the data

save('position_1','x1','-ASCII');
save('position_2','x2','-ASCII');
save('velocity_1','v1','-ASCII');
save('velocity_2','v2','-ASCII');
save('Density_1','density1','-ASCII');
save('Density_2','density2','-ASCII');

%% Plots
figure; surf(density1(:,1:10:n_time_steps),'EdgeColor','none'); 
view(0,90); colorbar;
set(gca,'FontSize',16);
xlabel('time-in seconds'); ylabel('location- in decameters'); title('Density on lane 1');

figure; surf(density2(:,1:10:n_time_steps),'EdgeColor','none');
view(0,90); colorbar;
set(gca,'FontSize',16);
xlabel('time-in seconds'); ylabel('location- in decameters'); title('Density on lane 2');

figure; surf(v1(:,1:10:n_time_steps),'EdgeColor','none');
view(0,90); colorbar;
set(gca,'ZDir','rev','FontSize',16);  %to reverse the z-scale
xlabel('time-in seconds'); ylabel('position of cars'); title('Velocity on lane 1');

figure; surf(v2(:,1:10:n_time_steps),'EdgeColor','none');
view(0,90); colorbar;
set(gca,'ZDir','rev','FontSize',16);
xlabel('time-in seconds'); ylabel('position of cars'); title('Velocity on lane 2');