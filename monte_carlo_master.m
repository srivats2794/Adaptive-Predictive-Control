clc;clear all;close all;

APC_wins=0; MPC_wins=0;
APC_bonus=0; MPC_bonus=0;

for i=1:1
    for j=1:10
        clc;clearvars -except APC_wins APC_bonus MPC_wins MPC_bonus i j;
        perc=5;
        plot_choice=0;
        [MPC,APC,master]= MRAC_startup(perc,plot_choice);
        results=calculateMetrics(MPC,APC);
        if results.winner=="MPC"
            MPC_wins=MPC_wins+1;
            MPC_bonus= MPC_bonus+results.MPCbonus;
        elseif results.winner=="APC"
            APC_wins=APC_wins+1;
            APC_bonus= APC_bonus+results.APCbonus;
        end
    end
end

save('5_perc_monte_carlo_run.mat')

plotter(MPC,APC,master)