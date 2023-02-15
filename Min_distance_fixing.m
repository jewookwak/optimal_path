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

x=round((x-437700)*10);
x_l = round((x_l-437700)*10);
x_r = round((x_r-437700)*10);
y=round((y-4205730)*10);
y_l=round((y_l-4205730)*10);
y_r=round((y_r-4205730)*10);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n=1413;
step = 1; %1 3 9 157
h_s1=0; h_s2=0; h_s3=0; h_s4=0;
b_s1=0; b_s2=0; b_s3=0; b_s4=0;

%quadprog parameters
lb = zeros(2,1);
ub = ones(size(lb));

W=zeros(2,1);

% iteration 1st step to end step
for i=1:step:n-step
    h_s1=(x_l(i+step)-x_r(i+step))^2+(y_l(i+step)-y_r(i+step))^2;
    h_s2=-(x_l(i)-x_r(i))*(x_l(i+step)-x_r(i+step))-(y_l(i)-y_r(i))*(y_l(i+step)-y_r(i+step));
    h_s3=h_s2;
    h_s4=(x_l(i)-x_r(i))^2+(y_l(i)-y_r(i))^2;

    H_s=[h_s1,h_s2;h_s3,h_s4];

    b_s1=2*(x_r(i+step)-x_r(i))*(x_l(i+step)-x_r(i+step))+2*(y_r(i+step)-y_r(i))*(y_l(i+step)-y_r(i+step));
    b_s2=-2*(x_r(i+step)-x_r(i))*(x_l(i)-x_r(i))-2*(y_r(i+step)-y_r(i))*(y_l(i)-y_r(i));
    
    B_s=[b_s1;b_s2];
    
    w=quadprog(H_s,B_s,[],[],[],[],lb,ub);
    W=[W,w];
    
end
W(:,1)=[];
%Path_min_dis=Center_sync(1,:);
Path_min_dis=[];

for i=1:1:length(W)
    L= x_l(i)-x_r(i);
    %L = sqrt((x_l(i)-x_r(i))^2+(y_l(i)-y_r(i))^2);
    path=[x_r((i-1)*step+1)*(1-W(1,i))+x_l((i-1)*step+1)*W(1,i),y_r((i-1)*step+1)*(1-W(1,i))+y_l((i-1)*step+1)*W(1,i)];
    %path=[x_r(i)+L*W(1,i),y_r(i)];
    Path_min_dis=[Path_min_dis;path];
end
last_index = length(W);
path = [x_r(last_index*step+1)*(1-W(2,last_index))+x_l(last_index*step+1)*W(2,last_index),y_r(last_index*step+1)*(1-W(2,last_index))+y_l(last_index*step+1)*W(2,last_index)];
Path_min_dis=[Path_min_dis;path];
%Path(1,:)=[];
Path_min_dis = Path_min_dis.';



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f=figure('position',[0 100 1000 850]);
hold on
plot(Path_min_dis(1,:),Path_min_dis(2,:),'c-','linewidth',1);
plot(x_r(:),y_r(:),'b.','linewidth',1);
plot(x_l(:),y_l(:),'b.','linewidth',1);
plot(x(:),y(:),'m.','linewidth',1);

axis equal
xlabel('Global X[m]');ylabel('Global Y[m]')
set(gca,'fontsize',12)