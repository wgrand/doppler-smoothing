clearvars;
%% initialization
x = [0;
     0;
     0;
     0];
L=readmatrix('2026-07-11_18_50_37_my_iOS_device.csv');
geoplot(L(20000:26000,4),L(20000:26000,5),'LineWidth',2);