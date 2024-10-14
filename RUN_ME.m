close all;clc;
percentage=100; % Perturbation percentage
[mpc,apc,master]= MRAC_startup(percentage);
plotter(mpc,apc,master);