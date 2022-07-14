x_sim = 0:0.001:20;
k = 150;
L = 0.25;
c = 0.025;
scale = @(x)(L./(1+exp(-k*(x-c))));

figure;
plot(x_sim, scale(x_sim))

%%
x_sim = 0:0.001:2;
r = @(x)(exp(-100*x))
figure;
plot(x_sim,r(x_sim))