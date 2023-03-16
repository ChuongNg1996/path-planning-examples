% clc
% clear

px = linspace(0,600,10);
px = px/100;
py = 0.5*sin(px);

hold on
plot(px,py);
% plot(simout.Data(:,1),simout.Data(:,2)); 
hold off