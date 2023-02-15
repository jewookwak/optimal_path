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
d=[];
s=[];
D=[];
S=[];
n=1413;
step = 1;
for i=1:step:n-step
    [d,s] = ClosestRefPoint(x(i),y(i),x,y);
    D=[D,d];
    S=[S,s];
end

for i=1:step:n-step
    [d,s] = Cart2FRT(x(i),y(i),0,x,y);
    D=[D,d];
    S=[S,s];
end
% [d,s] = ClosestRefPoint(x(1),y(1),x(10),y(10));
