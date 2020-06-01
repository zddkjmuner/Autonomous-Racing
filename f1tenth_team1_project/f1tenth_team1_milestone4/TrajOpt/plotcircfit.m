function plotcircfit(x,y)
%PLOTCIRCFIT  Fit X-Y data to a circle and create plot
%   PLOTCIRCFIT(X,Y) calls CIRCFIT to find best fit circle and creates a plot
%   that shows original data superimposed on fit and displays error in the
%   figure title. X and Y are equal length 1-D arrays of position data in a
%   rectilinear coordinate system.
%
%   Examples:
%       % Fit of just five noisy points
%       x1=[1 0 -1 0 1]+0.05*randn(1,5); y1=[0 1 0 -1 0]+0.05*randn(1,5);
%       plotcircfit(x1,y1);
%
%       % CIRCFIT can sometimes perfom poorly if less than 180-degrees arc used
%       t=0:0.1:pi; lt=length(t);
%       x2=cos(t)+0.04*randn(1,lt); y2=sin(t)+0.04*randn(1,lt);
%       plotcircfit(x2(1:floor(lt/2)),y2(1:floor(lt/2)));
%       plotcircfit(x2,y2);
%
%   See also CIRCFIT, CIRCRMSE
%   Andrew D. Horchler, horchler @ gmail . com, Created 5-12-7
%   Revision: 1.2, 4-8-16
[r,xc,yc,err]=circfit(x,y);
figure;
rectangle('Curvature',[1 1],'Position',[xc-r yc-r 2*r 2*r]); % Plot circle fit
grid on;
axis equal;
hold on;
plot(x,y,'b',x(1),y(1),'g.',x(end),y(end),'r.',xc,yc,'k.')
xlabel('x (meters)');
ylabel('y (meters)');
title(['Best Fit Circle, Root Mean Squared Error = ' num2str(err,5) ' meters']);
text(xc+0.5,yc,['R = ' num2str(r,5)],'BackgroundColor','w');