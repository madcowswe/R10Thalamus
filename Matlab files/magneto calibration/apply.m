figure;
hold on;
scatter3(Mag(:,1),Mag(:,2),Mag(:,3), '.');

scale = sum(max(Mag) - min(Mag))/6;
N = size(Mag, 1);
Magcor=(Mag+repmat(magcor_c,N,1))*magcor_U*scale;
scatter3(Magcor(:,1),Magcor(:,2),Magcor(:,3),'.r');
axis equal
view([20 12]);
hold off

clear scale N;