clear all
close all

% Identifier case number of simulation scenario
case_num = 12;

% Define pedestrian flux phi = flux_ped/flux_dt
% Constant flux of flux_ped pedestrians every flux_dt seconds
% Initial spacing between pedestrians is v_d*flux_dt
v_d = 1.2;
flux_ped = 1;
flux_dt = 2;

% Global variables can be avoided by modifying the code.

global n_groups
global N 
global r 
global m 
global v0 
global v 
global J
global tau 
global A 
global B 
global C
global D
global Aw
global Bw
global beta2
global k1 
global k2 
global p 
global e_act 
global e_ind 
global e_seq 
global e_n 
global ps 
global num_walls 
global map_walls 
global alfa
global kd ko k1g k2g
global d_o d_f
global include_groups


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% System definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Load map %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
map_walls = map_def;
[dnum_walls, ~] = size(map_walls);
%number of walls
num_walls = dnum_walls/2;         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
%%%%%%%%%%%%%%%%%%% Simulation and model parameters %%%%%%%%%%%%%%%%%%%%%%%
% Simulation time 
TF=60;
t_fine=1/30; % timing accuracy in the movie (framerate=30)

% SFM Parameters
tau=0.7;  
A=2000;
B=0.08;%0.8
Aw=2000;
Bw=0.08;
beta2=3;
k1=1.2*10^5;
k2=2.4*10^5;
% HSFM Parameters
kd=500;
ko=1;
k1g=200; % forward group cohesion force strength
k2g=400; % sideward group cohesion force strength
d_o=0.5; % sideward maximum distance from the center of mass
d_f=0.5;   % forward maximum distance from the center of mass   

% individual characteristics
% Radius
rm=0.25; % minimum radius
rM=0.25; % maximum radius
% Mass
mm=60; % minimum mass
mM=90; % maximum mass
% Desired speed
v0m=1.1; % minimum velocity
v0M=1.3; % maximum velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%% Initial conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Number of individuals in each group
% Define n_i the number of individuals in group i, then
% n_groups = [n_1, n_2, ..., n_N];
% Individuals are treated as groups of 1 with no group force


% For simulation time of 60 seconds
% need constant flux of flux_ped pedestrians every flux_dt seconds
% in A-B direction and C-D direction
num_groups = 60/flux_dt *2; 

if flux_ped ==1
    % single pedestrian in group
    include_groups = false;
else
    include_groups = true;
end

% Creating n_groups vector
n_groups= flux_ped*ones(num_groups,1);

% Total number of individuals
N=sum(n_groups);

% s{i} contains the starting point of pedestrian i (group size 1)

xg1_ab = 12.5; 
yg1_ab = 0;

xg1_cd = 0;
yg1_cd = 12.5;

s{1} = [xg1_ab yg1_ab];
for i=2 : num_groups/2
    s{i} = [xg1_ab yg1_ab-(v_d*flux_dt)*(i-1)];
end

s{num_groups/2+1} = [xg1_cd yg1_cd];
i = 1;
for j = num_groups/2 +2 :num_groups
    i = i+1;
    s{j} = [xg1_cd-(v_d*flux_dt)*(i-1) yg1_cd];
end


% "am" represents the amplitude of the starting zone. Fix the starting 
% points at least at "am" meters from the walls
am=2;   

% Individual characteristics
v0=v0m+(v0M-v0m)*rand(N,1);             % random desired speed in range
v=0*ones(N,2);                          % initial speed 
th=2*pi*rand(N,1)-pi;                   % initial orientation
omg=0;                                  % initial angular velocity

for i =1:N
    r(i)=rM;                            % fixed radius
end

for i=1:length(n_groups)                % random masses in range
    % random masses
    m(sum(n_groups(1:i))-n_groups(i)+1:sum(n_groups(1:i)))=sort((mm)+(mM-mm)*rand(n_groups(i),1)); 
    cumulative(i)=sum(n_groups(1:i));
end
J=0.5*r.^2;                             % Inertia

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Path definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% waypoints sequence
e_n=cell(0);
e_ind=cell(0);

% e_seq{i} contains the points through which the members of group i have to
% pass

% e_seq{1}=[3 10; 2.5 1000]';
% e_seq{2}=[2 10; 2.5 -1000]';

for i = 1:num_groups/2                  % Corridor AB
e_seq{i} = [13 20; 12.5 1000]';
end
for j = num_groups/2 +1 : num_groups    % Corridor CD
e_seq{j} = [20 12; 1000 12.5]';
end


for i=1:length(n_groups)
    [~,e_n{i}]=size(e_seq{i});      % number of waypoints of group i
    e_ind{i}=zeros(n_groups(i),1);  % current waypoint for group i
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% END OF THE PART TO BE CHANGED %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% System initialization
p=zeros(N,6);
p(1,:)=[s{1}(1)-am+2*am*rand(1,1) s{1}(2)-am+2*am*rand(1,1) v(1,:)  r(i) m(i)];
% initial HSFM state vector (1,4*N) -> [Position Heading Speed 0 Angular_velocity]
X1=[p(1,1:2) th(1) norm(v(1,:)) 0 omg]; 
i=2;
% random initial positions
while i<=N
    gr=find(cumulative>=i,1,'first');
    pos=[s{gr}(1)-am+2*am*rand(1,1)  s{gr}(2)-am+2*am*rand(1,1)];
    % minimum distance between pedestrians
    for l=1:i
        d(l)=norm(pos-p(l,1:2))<=r(i)+r(l);
    end
    % minimum distance from walls
    for l=i+1:i+num_walls
        xp=pos(1);
        yp=pos(2);
        rp=[xp yp]';
        ra=map_walls(2*(l-i)-1:2*(l-i),1); 
        rb=map_walls(2*(l-i)-1:2*(l-i),2);
        xa=ra(1);
        ya=ra(2);
        xb=rb(1);
        yb=rb(2);
        t=((xp-xa)*(xb-xa)+(yp-ya)*(yb-ya))/(((xb-xa)^2+(yb-ya)^2));
        t_star=min(max(0,t),1);
        rh=ra+t_star*(rb-ra);
        d(l)=norm(rp-rh)<=r(i);
    end
    if sum(d)==0
        p(i,:)=[pos(1) pos(2) v(i,:) r(i) m(i)];
        X1=[X1 pos(1) pos(2) th(i) norm(v(i,:)) 0 omg];
        i=i+1;
    end
end  


ps=p(:,1:2);
e_act{1}=ps(1:n_groups(1),:);
for i=2:length(n_groups)
     % current waypoint
    e_act{i}=ps(sum(n_groups(1:i-1))+1:sum(n_groups(1:i)),:);
end

include_groups

%%%%%%%%%%%%%%%%%%%%%%%%%%% System simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alfa=3;
tic
tspan=0:t_fine:TF;
% [t,X]=ode45(@system_model_NHM,tspan,X1');
 [t,X]=ode45(@system_model_NHM,tspan,X1');
toc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
str_case = sprintf('results/case_%02d.mat', case_num);   
save(str_case)

%%% Movie of the simulation %%%
% movieplay_rk