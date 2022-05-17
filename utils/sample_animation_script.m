z = 0:0.05:10;
y = sin(2*z);
x = cos(2*z);

curve = animatedline('LineWidth',2);
set(gca,'XLim',[-1.5 1.5],'YLim',[-1.5 1.5],'ZLim',[0 10]);
view(43,24);
hold on;
for i=1:length(z)
    addpoints(curve,x(i),y(i),z(i));
    head = scatter3(x(i),y(i),z(i),'filled','MarkerFaceColor','b');
    drawnow
    pause(0.01);
    delete(head);
end