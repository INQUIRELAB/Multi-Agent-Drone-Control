% In the name of GOD
% Ya Hussain (a.s.)
%===================
% The presented code simulates the MAS control 
% for Barrier Coverage problem 
% SNN-based Control of Flocking 
%===================
% Preparing MATLAB

clc
clear
close all
%===================
% Definition of Parameters  
global Snum iteration NOA 

%~~~~~~~~~~~~~~~~~~~
% General Parameters 

NOA = 12;         % Number of Agents 
Snum = 2000;      % number of samples per iteration
iteration = 300;  % number of algorithm iteration



%~~~~~~~~~~~~~~~~~~~
% Cloud based Control Parameters

Vx = 1;   % Flock Movement speed in x-direction

rd = 2;   % Detection zone limit
rs = 1;   % Safety zone limit

Kpf = diag([3,3,3]); 
Kvf = diag([5,5,5])/1; 

kox = .1;
koy = .5;
koz = .1;

kr = .5;

ko1 = 5;
ko2 = 1;
 
kc1 = 10;
kc2 = 1;

% initial condition of agents 
x0 = 2*rand(1,NOA);
y0 = 2*rand(1,NOA);
z0 = 0*rand(1,NOA);

X0_mat = zeros(6,NOA); % each column corresponds to one UAV
for i = 1:NOA
    X0_mat(:,i) = [x0(i);y0(i);z0(i);0;0;0];
end

%~~~~~~~~~~~~~~~~~~~~~~~~~
% Obstacle characteristics
Building_location = 50; 
Critical_distance_to_building = 15;

Obs_placer = [35,0,0]';
x_obs = [35,10/2 + 5 ,4]' + Obs_placer;
x_obs2 = [25,14/2 + 5,6]' + Obs_placer;
x_obs3 = [20,5/2 + 5,5]'+ Obs_placer;
x0_obs4 = [35,2/2 + 6,5]'+ Obs_placer;

X_obs = [x_obs,x_obs2,x_obs3];

obs_radious = 1;
FOV_theta = 60*pi/180;


%~~~~~~~~~~~~~~~~~~~~~~~~
% moving obs velocities

vx_obs = 2*0.1;
vy_obs = 2*0.025; 

% Determination of two vertices on the edges of considered Barrier

% Horizontal arrangement

e1 = [0;0;5];
e2 = [30;0;5];
el = [0;20;5];

%====================
% Simulation Parameters 

tf = 285/2;
dt = .1;
tspan = 0:dt:tf;
E = length(tspan);
%====================
% Prelocations
Vehicles_Pos = cellmat(1,NOA,6,1);  % just define the spaces 

Vehicles_input = cellmat(1,NOA,3,1);


%Initialization

X_mat = X0_mat;


x_obs4 = x0_obs4;
X_obs = [X_obs,x_obs4];
for ii = 1:E 

    % Data saving 
  for k = 1:NOA
      if ii == 1
        Vehicles_Pos{k} = X_mat(:,k);
        
      else
        Vehicles_Pos{k} = [Vehicles_Pos{k},X_mat(:,k)];
        
      end
  end 
  
  %======================================
  % Flocking Optimal Position Computation
    
    if ii == 1
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Barrier-frame Transfromation Matrix
    T_I_to_B = TransMat(e1,e2,el);     

    % Coordinate transformation from I to B
    Xmat_B = zeros(4,NOA);
    for j = 1:NOA
        Xmat_B(:,j) = T_I_to_B*[X_mat(1:3,j);1];
    end
        
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Voronoi centrioids computation in B frame
    Xmat_B(4,:) = [];                 % removing the 4th line        
    Xmat_BPro = Xmat_B;
    
    % Computing Centroids for each Vehicle by j index
    CentroidsMat_B = Centroid_Calculator(Xmat_BPro);
    
    % Transform centroids position to Inertial frame I
    T_B_to_I = T_I_to_B^(-1);
    CentroidsMat_I = zeros(4,NOA);
        for j = 1:NOA
            CentroidsMat_I(:,j) = T_B_to_I*[CentroidsMat_B(:,j);1];
        end
        CentroidsMat_I(4,:) = []; % removing 4th row
    end
    
    
    % Flock Movement Dynamics
    if ii == 1
       CentroidsMat_I0 = CentroidsMat_I; 
    end
    CentroidsMat_I(1,:) = Vx*tspan(ii) + CentroidsMat_I0(1,:);  % x = vt + x0
    %================================
    % Formation change trigger
    
    %================================
    % Problem Simulation (homogeneous agents have the same dynamics)
    
    
  for j = 1:NOA
      
     
      
      %======================= 
      % Formation Controller 
      
      Closest_to_building = max(X_mat(1,:));
      distance_to_building = abs(Closest_to_building - Building_location);
      if distance_to_building < Critical_distance_to_building
          CentroidsMat_I(2,j) =  CentroidsMat_I(2,j)/2 + 5;
      elseif X_mat(1,j) > 100
          CentroidsMat_I(2,j) =  CentroidsMat_I0(2,j);
      end
      
 
      Uf = -Kpf*(X_mat(1:3,j)-CentroidsMat_I(:,j)) - Kvf*(X_mat(4:6,j)-[0;0;0]);
      
      %=======================
      % Obstacle Avoidance Controller
      
      % obstacle detection
      
       X_pos = X_mat(1:3,j);
       X_vel = X_mat(4:6,j);
       OD_metrico = zeros(1,4);
       for i = 1:4
           % presence in detection range of radar
           obs_vector = -[X_pos(1) - X_obs(1,i); X_pos(2) - X_obs(2,i); X_pos(3) - X_obs(3,i)];
           OD_metric_range = norm(obs_vector);
           
           % presence in sensor FOV
           OD_metric_FOV = abs(acos(dot(obs_vector,X_vel)/norm(obs_vector)/norm(X_vel)));
          
           if OD_metric_range < (obs_radious + rd) && OD_metric_FOV < FOV_theta
                  disp(OD_metric_FOV*180/pi)
                  OD_metrico(i) = 1;
           else
                  OD_metrico(i) = 0;
           end
       end
       
       % velocity adjustment 
       SOD_metric = sum(OD_metrico);
       if SOD_metric > 0 
           
           Upk = zeros(3,4);
           Urk = zeros(3,4);
           
           for i = 1:4 % considering four obstacles;
               
               gradient_operator = [X_pos(1) - X_obs(1,i); X_pos(2) - X_obs(2,i); X_pos(3) - X_obs(3,i)]/norm([X_pos(1) - X_obs(1,i); X_pos(2) - X_obs(2,i); X_pos(3) - X_obs(3,i)]);
               
               % rotation angle 
               if norm([X_pos(1) - X_obs(1,i); X_pos(2) - X_obs(2,i);X_pos(3) - X_obs(3,i)]) < rd && norm([X_pos(1) - X_obs(1,i); X_pos(2) - X_obs(2,i); X_pos(3) - X_obs(3,i)]) >= rs
                   alpha_rotation = ((rd - norm([X_pos(1) - X_obs(1,i); X_pos(2) - X_obs(2,i); X_pos(3) - X_obs(3,i)]))/(rd - rs))*pi/2;
               else 
                   alpha_rotation = 0;
               end
               
               % rotation matrix 
               Trz = [cos(alpha_rotation) -sin(alpha_rotation) 0; sin(alpha_rotation) cos(alpha_rotation) 0; 0 0 1];
               Try = [cos(alpha_rotation) 0  sin(alpha_rotation); 0 1 0; -sin(alpha_rotation) 0 cos(alpha_rotation)];
               Trx = [1 0 0; 0 1 0; 0 0 1];
               
               Tr = Trx*Try*Trz;
               
               % Potentials Gradients
               delta_Vp = (norm(diag([kox,koy,koz])*[X_pos(1) - X_obs(1,i); X_pos(2) - X_obs(2,i); X_pos(3) - X_obs(3,i)]) - (rd + obs_radious))*gradient_operator;
               delta_Vr = kr*Tr*norm([X_pos(1) - X_obs(1,i); X_pos(2) - X_obs(2,i); X_pos(3) - X_obs(3,i)])*gradient_operator;
 
               Upk(:,i) = delta_Vp;
               Urk(:,i) = delta_Vr;
               
           end
           Uo = -ko1*sum(Upk')' - sum(Urk')' - ko2*X_vel(1:3);
       else 
           Uo = zeros(3,1); 
       end
       
       %=======================
       % Collision Avoidance Controller 
       if abs(X_pos(3)-5)<2
       for i = 1:NOA
           if i~=j
               prel = X_mat(1:3,i)-X_pos(1:3);
               vrel = X_mat(4:6,i)-X_vel(1:3);
               %~~~~~~~~~~~~~~~~
               pc = (1/(norm(prel)-rs)^2)*(prel/norm(prel));
               uc = -kc1*pc + kc2*vrel;
           else
               uc = [0;0;0];
           end
           uci(:,i) = uc;
       end
       Uc = sum(uci')';
       if sum(Uc)>0 
           disp('colision')
       end 
       else
           Uc = [0;0;0];
       end
      
      %=========================
      % Overal Controller 
       
       U = Uf + Uo + Uc;
      
      
      input_mat(:,j) = U;
     
   

    % Cloud Dynamic Propagation
    f1 = dt*Dyn(X_mat(:,j),U);
    f2 = dt*Dyn(f1/2+X_mat(:,j),U);
    f3 = dt*Dyn(f2/2+X_mat(:,j),U);
    f4 = dt*Dyn(f3+X_mat(:,j),U);
    X_mat(:,j)  = X_mat(:,j) + (f1 + 2*f2 + 2*f3 + f4)/6;

   
    % Building wall constraint
    if X_mat(1,j) > 50 && X_mat(1,j) <100
        if X_mat(2,j) > 13.5 
            X_mat(2,j) = 13.5;
        elseif X_mat(2,j) < 6.5 
            X_mat(2,j) = 6.5;
        end
    end
    
    if j == 1 
        for i = 1:NOA
            if i~=j
                relative_distance(i,ii) = norm(X_mat(1:3,j) - X_mat(1:3,i));
            end
        end
    end
    
  end
  
  % obstacle movement model
       if tspan(ii) >= 42
           X_obs(1:2,4) = (tspan(ii)-45)*[vx_obs;vy_obs] + x0_obs4(1:2);
       else
           X_obs(1:2,4) =  x0_obs4(1:2);
       end
       x_obs4t(:,ii) = X_obs(:,4); % obstacle movement data saving

      % Data Saving 
      for k = 1:NOA
          if ii == 1
            Vehicles_input{k} = input_mat(:,k);
           
          else
            Vehicles_input{k} = [Vehicles_input{k},input_mat(:,k)];
           
          end
      end    
end
%% ========== Plots 
% Voronoi Diagram 

figure(2)
voronoi(CentroidsMat_B(1,:),CentroidsMat_B(2,:),'k')
xlabel('x (m)')
ylabel('y (m)')
xlim([0,10])
ylim([0,10])

 

%% ===============================
% Trajectory Video
% prelocation for projection Voronoi
Projection = zeros(2,NOA); 

figure
% V2 = VideoWriter('multi_UAV');  % for .avi format we can delete the second arqument
% V2.FrameRate = 100;
% open(V2);
for i = 1:E
    if i <= 25
        hold off
str = {'  '};
for k = 1:NOA
    Pos = Vehicles_Pos{k};
    plot3(Pos(1,1:i),Pos(2,1:i),Pos(3,1:i),':','linewidth',2)
    hold on
    grid on 
    plot3(Pos(1,i),Pos(2,i),Pos(3,i),'kx','linewidth',2)
    char = int2str(k);
    char = strcat(str,char);
    text(Pos(1,i),Pos(2,i),Pos(3,i),char)
    Projection(:,k) = Pos(1:2,i);
end
% 
voronoi(Projection(1,:),Projection(2,:),'k')
delta = Projection(1,1) - CentroidsMat_I0(1,1);

% buildings

B1 = [50 0 5];   
B2 = [100 0 5];
B3 = [100 6 5];
B4 = [50 6 5];

XB = [B1(1) B2(1) B3(1) B4(1)];
YB = [B1(2) B2(2) B3(2) B4(2)];
ZB = [B1(3) B2(3) B3(3) B4(3)];
fill3(XB,YB,ZB,1,'facealpha',1,'facecolor','k')

BB1 = [50 14 5];   
BB2 = [100 14 5];
BB3 = [100 20 5];
BB4 = [50 20 5];

XBB = [BB1(1) BB2(1) BB3(1) BB4(1)];
YBB = [BB1(2) BB2(2) BB3(2) BB4(2)];
ZBB = [BB1(3) BB2(3) BB3(3) BB4(3)];
fill3(XBB,YBB,ZBB,1,'facealpha',1,'facecolor','k')

P1 = [0+delta 0 5];   
P2 = [10+delta 0 5];
P3 = [10+delta 20 5];
P4 = [0+delta 20 5];

XP = [P1(1) P2(1) P3(1) P4(1)];
YP = [P1(2) P2(2) P3(2) P4(2)];
ZP = [P1(3) P2(3) P3(3) P4(3)];
fill3(XP,YP,ZP,1,'facealpha',0.2,'facecolor','g')



if delta > 40 && delta < 100
    P1 = [0+delta 6 5];   
    P2 = [10+delta 6 5];
    P3 = [10+delta 14 5];
    P4 = [0+delta 14 5];

    XP = [P1(1) P2(1) P3(1) P4(1)];
    YP = [P1(2) P2(2) P3(2) P4(2)];
    ZP = [P1(3) P2(3) P3(3) P4(3)];
    fill3(XP,YP,ZP,1,'facealpha',0.3,'facecolor','g')
else
    P1 = [0+delta 0 5];   
    P2 = [10+delta 0 5];
    P3 = [10+delta 20 5];
    P4 = [0+delta 20 5];

    XP = [P1(1) P2(1) P3(1) P4(1)];
    YP = [P1(2) P2(2) P3(2) P4(2)];
    ZP = [P1(3) P2(3) P3(3) P4(3)];
    fill3(XP,YP,ZP,1,'facealpha',0.3,'facecolor','g')
end


    thetavec = linspace(0,pi,10);
    phivec = linspace(0,2*pi,2*10);
    [th, ph] = meshgrid(thetavec,phivec);
    R = 0.8*obs_radious*ones(size(th)); % should be your R(theta,phi) surface in general
    X =  R.*sin(th).*cos(ph);
    Y =  R.*sin(th).*sin(ph);
    Z =  R.*cos(th);

surf(X+x_obs(1),Y+x_obs(2),Z+x_obs(3), 'facealpha',0.5, 'edgealpha',1,'linewidth',1)
surf(X+x_obs2(1),Y+x_obs2(2),Z+x_obs2(3), 'facealpha',0.5, 'edgealpha',1,'linewidth',1)
surf(X+x_obs3(1),Y+x_obs3(2),Z+x_obs3(3), 'facealpha',0.5, 'edgealpha',1,'linewidth',1)
surf(X+x_obs4t(1,i),Y+x_obs4t(2,i),Z+x_obs4t(3,i), 'facealpha',0.5, 'edgealpha',1,'linewidth',1)


text(x_obs(1),x_obs(2),x_obs(3),'   Obs.1')
text(x_obs2(1),x_obs2(2),x_obs2(3),'   Obs.2')
text(x_obs3(1),x_obs3(2),x_obs3(3),'   Obs.3')
text(x_obs4t(1,i),x_obs4t(2,i),x_obs4t(3,i),'   Obs.4')

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
zlim([0,10])
xlim([0,150])
view([-30 35]);

drawnow()

    elseif mod(i,10) == 0
hold off
str = {'  '};
for k = 1:NOA
    Pos = Vehicles_Pos{k};
    plot3(Pos(1,1:i),Pos(2,1:i),Pos(3,1:i),':','linewidth',2)
    hold on
    grid on 
    plot3(Pos(1,i),Pos(2,i),Pos(3,i),'kx','linewidth',2)
    char = int2str(k);
    char = strcat(str,char);
    text(Pos(1,i),Pos(2,i),Pos(3,i),char)
    Projection(:,k) = Pos(1:2,i);
end
% 
voronoi(Projection(1,:),Projection(2,:),'k')
delta = Projection(1,1) - CentroidsMat_I0(1,1);

% buildings

B1 = [50 0 5];   
B2 = [100 0 5];
B3 = [100 6 5];
B4 = [50 6 5];

XB = [B1(1) B2(1) B3(1) B4(1)];
YB = [B1(2) B2(2) B3(2) B4(2)];
ZB = [B1(3) B2(3) B3(3) B4(3)];
fill3(XB,YB,ZB,1,'facealpha',0.5,'facecolor','k')

BB1 = [50 14 5];   
BB2 = [100 14 5];
BB3 = [100 20 5];
BB4 = [50 20 5];

XBB = [BB1(1) BB2(1) BB3(1) BB4(1)];
YBB = [BB1(2) BB2(2) BB3(2) BB4(2)];
ZBB = [BB1(3) BB2(3) BB3(3) BB4(3)];
fill3(XBB,YBB,ZBB,1,'facealpha',0.5,'facecolor','k')



if delta > 40 && delta < 100
    P1 = [0+delta 6 5];   
    P2 = [10+delta 6 5];
    P3 = [10+delta 14 5];
    P4 = [0+delta 14 5];

    XP = [P1(1) P2(1) P3(1) P4(1)];
    YP = [P1(2) P2(2) P3(2) P4(2)];
    ZP = [P1(3) P2(3) P3(3) P4(3)];
    fill3(XP,YP,ZP,1,'facealpha',0.3,'facecolor','g')
else
    P1 = [0+delta 0 5];   
    P2 = [10+delta 0 5];
    P3 = [10+delta 20 5];
    P4 = [0+delta 20 5];

    XP = [P1(1) P2(1) P3(1) P4(1)];
    YP = [P1(2) P2(2) P3(2) P4(2)];
    ZP = [P1(3) P2(3) P3(3) P4(3)];
    fill3(XP,YP,ZP,1,'facealpha',0.3,'facecolor','g')
end
    thetavec = linspace(0,pi,50);
    phivec = linspace(0,2*pi,2*50);
    [th, ph] = meshgrid(thetavec,phivec);
    R = 0.8*obs_radious*ones(size(th)); % should be your R(theta,phi) surface in general
    X =  R.*sin(th).*cos(ph);
    Y =  R.*sin(th).*sin(ph);
    Z =  R.*cos(th);
    
surf(X+x_obs(1),Y+x_obs(2),Z+x_obs(3), 'facealpha',1, 'edgealpha',1,'linewidth',2)
surf(X+x_obs2(1),Y+x_obs2(2),Z+x_obs2(3), 'facealpha',1, 'edgealpha',1,'linewidth',2)
surf(X+x_obs3(1),Y+x_obs3(2),Z+x_obs3(3), 'facealpha',1, 'edgealpha',1,'linewidth',2)
surf(X+x_obs4t(1,i),Y+x_obs4t(2,i),Z+x_obs4t(3,i), 'facealpha',1, 'edgealpha',1,'linewidth',2)


text(x_obs(1),x_obs(2),x_obs(3),'   Obs.1')
text(x_obs2(1),x_obs2(2),x_obs2(3),'   Obs.2')
text(x_obs3(1),x_obs3(2),x_obs3(3),'   Obs.3')
text(x_obs4t(1,i),x_obs4t(2,i),x_obs4t(3,i),'   Obs.4')


xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
zlim([0,15])
xlim([0,150])
view([0 90]);

drawnow()
    end
%   frame = getframe(gcf);
%   writeVideo(V2,frame);
end
% close(V2)

%% Relative Distance (Collision avoidance check)

figure(3)
hold on 

for i = 1:NOA
    if i~=1
        plot(tspan, relative_distance(i,:),'linewidth',2);
    end
end

grid on 
xlabel('time(s)')
ylabel('|d_i_j| (m)')
xlim([0,max(tspan)])

