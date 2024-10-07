function [cascaded, cascaded_adaptive,master] = MRAC_startup(perc,plot_choice)
clc;close all;
%%
sim_choice=1; % TURN ON IF YOU WANT TO RUN A SIM
%%
if sim_choice
    clc;clearvars -except perc plot_choice;close all;
    %%
    if nargin<1
        plot_choice=1; % Do you want the plots or no?
    end
    if nargin<2
        perc=100; % MANIPULATE THE PARAMETER PERTURBATION PERCENTAGE
    end

    loud = 0; % Plays video of the whole simulation if its set to 1

    %%
    % Simulation time and step
    master.tsim=20; % Total simulation time of 20 seconds
    master.simTs=0.01; % Time step of 0.01 seconds
    
    %%
    % Mass perturbation
    master.m_p_0=4;  
    % Function that varies m_p. Ouputs m_p(t)
    master.m_vec= varyParamsDiscrete(master.m_p_0,perc,randi([1 3]),randi([9 12]),master.tsim,master.simTs,~loud);

    %%
    % l_p (Length) perturbation
    master.l_0= 0.53;          % Initial parameter value
    master.sine_min_freq = 1;         % Minimum frequency of 1 Hz
    master.sine_max_freq = 10;        % Maximum frequency of 20 Hz
    master.num_freqs = 5;             % Number of frequencies per segment     
    master.change_interval = 4;       % Change frequency every X seconds
   
    % This function varies the l_p as a fourier series of various
    % frequencies combined
        [master.t,master.l_vec]= varyParamsFourierSeries(master.l_0,...
            perc,master.sine_min_freq,master.sine_max_freq,...
            5,master.tsim,master.simTs,master.change_interval,~loud);
        
    % % This function varies l_p as a smooth sine with noise
    %     [master.t,master.l_vec]= varyParamsSmoothSine(master.l_0,...
    %         perc,master.sine_min_freq,master.sine_max_freq,...
    %         0.001,master.tsim,master.simTs,1,~loud);
    
    
%%
    % Simulates the vanilla MPC... Open-> MPC/main.m for details
    cd MPC\
     main;
     clearvars -except cascaded loud master plot_choice
     clc;close all;
    cd ..
    
    % Simulates the Adaptive Predictive Control... APC/main.m for details
    cd APC\
     main;
     clearvars -except cascaded cascaded_adaptive master plot_choice loud
     clc;close all;
    cd ..
end

if plot_choice
    plotter(cascaded,cascaded_adaptive,master)
end