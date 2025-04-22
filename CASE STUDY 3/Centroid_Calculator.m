function centroids = Centroid_Calculator(Xmat)
% In the name of GOD
% Ya Hussain (a.s.)
%========================
% This is for implementing space devision 
%========================
% Preparing MATLAB

%========================
% Defintion of Parameters 
global Snum iteration NOA

Gnum = NOA;
alpha1 = 0.01;
alpha2 = 0.99;
beta1 = 0.01;
beta2 = 0.99;


xlimit = 10;     % boundary of Omega (desired area)
ylimit = 20;     % boundary of Omega (desired area)
zlimit = 1;     % boundary of Omega (desired area)

%========================
% discretize the desired area
xdir = xlimit*rand([Snum,1]); 
ydir = ylimit*rand([Snum,1]);
zdir = zlimit*randn([Snum,1]);

%=======================
% set vehicle positions as the random generators (xi)
Gens = Xmat(1:3,:);

%~~~~~~~~~~~~~~~~~~~~~~~
Ji = 1; % setting Ji paremeter for all the initial points
Jis = ones(1,Gnum);

for iter = 1:iteration
    
   % random sampling (yr) from the desired area using uniform density function
    samples = zeros(3,Snum);
    for i = 1:Snum
    
        selector = Snum*rand;      % find a selector
        selector = ceil(selector); % round off

        samples_x = xdir(selector);
        samples_y = ydir(selector);
        samples_z = zdir(selector);

        samples(:,i) = [samples_x;samples_y;samples_z];
    end
    
    % voronoi area (set of points) computation for j th point
    Wis = cellmat(1,Gnum,3,1);
    for i = 1:Snum % distance computation of each sample to all vehichles
            distance = zeros(1,Gnum);
        for j = 1:Gnum 
            distance(j) = sqrt((Gens(1,j)-samples(1,i))^2 + (Gens(2,j)-samples(2,i))^2 + (Gens(3,j)-samples(3,i))^2);
        end
        nearest_vehicle_number = find(distance == min(distance));
        Wis{nearest_vehicle_number} = [Wis{nearest_vehicle_number},samples(:,i)];
    end
    
    % check for empty Wi
    
    for i = 1:Gnum 
   
        Wi_holder = Wis{i};
        temp1 = size(Wi_holder);
        elements_inWi_number = temp1(2);
        
        empty_check = sum(sum(Wi_holder));
        if empty_check ~= 0
            u = (sum(Wi_holder')')/(elements_inWi_number-1); % minus 1 is for considering the first zeros column which comes from definition of cellmat array
            Gens(:,i) = ((alpha1*Jis(i) + beta1)*Gens(:,i) + (alpha2*Jis(i) +beta2)*u)/(Jis(i)+1);
            Jis(i) = Jis(i) + 1;
        end
    end

end

centroids = Gens;
end






