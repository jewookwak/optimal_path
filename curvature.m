clc
clear all
close all

% initial path
load Inner.mat
load Outer.mat
load Center.mat

x= Center_sync(:,1);
y= Center_sync(:,2);
x_l = Outer_sync(:,1);
x_r = Inner_sync(:,1);
y_l = Outer_sync(:,2);
y_r = Inner_sync(:,2);
start = 151;
ends = 180;
%p,p_r,p_l is coefficient of 3rd polynomial 
p = polyfit(x(151:ends),y(151:ends),3);
p_r = polyfit(x_r(51:54),y_r(51:54),3);
p_l = polyfit(x_l(51:54),y_l(51:54),3);

f = polyval(p,x(151:ends));
figure;
plot(x(151:ends),y(151:ends),'o')
hold on
plot(x(151:ends),f,'r--')


n=1413;
step = 1; %1 3 9 157
t=[];
x_i=[];
y_i=[];
dx=0;
% defining t(s)
for i=1:step:n-step
    t=[t,Distance(x(1),y(1),x(i+step),y(i+step))/Distance(x(i),y(i),x(i+1),y(i+1))];
end

% defining x(i), y(i) with t(s)
% [dx] array
for i=1:step:n-4-step
    x_i=[x_i;polyfit(t(i:i+3),x(i:i+3),3)];
    y_i=[y_i;polyfit(t(i:i+3),y(i:i+3),3)];
    dx=[dx,x_l(i)-x_r(i)];
end

dx(1)=[];
lb = zeros(2,1);
ub = ones(size(lb));
D=[];
d_s1=0; d_s2=0; d_s3=0; d_s4=0;
h_s1=0; h_s2=0; h_s3=0; h_s4=0;
b_s1=0; b_s2=0; b_s3=0; b_s4=0;
xr=[];
W=[];
for i=1:step:n-4-step
    d_s1 = 6*x_i(i,4);
    d_s2 = 2*x_i(i,3);
    d_s3 = 6*y_i(i,4);
    d_s4 = 2*y_i(i,3);
    D=[d_s1,d_s2;d_s3,d_s4];
    xr=[x_r(i+1);x_r(i)];
    H=dx(i)*transpose(D)*D*dx(i);
    B=2*transpose(xr)*transpose(D)*D*dx(i);
    w=quadprog(H,B.',[],[],[],[],lb,ub);
    W=[W,w];
end

%W(:,1)=[];
%Path_min_dis=Center_sync(1,:);
Path_min_dis=[];

for i=1:1:length(W)
    L = x_l(i)-x_r(i);
    %L = sqrt((x_l(i)-x_r(i))^2+(y_l(i)-y_r(i))^2);
    path=[x_r(i)+L*W(1,i),y_r(i)+L*W(1,i)];
    Path_min_dis=[Path_min_dis;path];
end
%Path(1,:)=[];

%% Koh Data Integration %%
Centerline = [Center_sync.'; Center_sync(:,1).'*0+1];
Outer_sync = Outer_sync.';
Inner_sync = Inner_sync.';
Path_min_dis = Path_min_dis.';

%% Path Plot with repect to minimum distance %%
f=figure('position',[100 50 1000 850]);
hold on
plot(Path_min_dis(1,:),Path_min_dis(2,:),'c.','linewidth',1);
plot(Inner_sync(1,:),Inner_sync(2,:),'b.','linewidth',1);
plot(Outer_sync(1,:),Outer_sync(2,:),'b.','linewidth',1);
plot(Centerline(1,:),Centerline(2,:),'m.','linewidth',1);

axis equal
xlabel('Global X[m]');ylabel('Global Y[m]')
set(gca,'fontsize',12)