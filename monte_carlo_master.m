clc;close all;


for i=1:10
    for j=1:10
        clc;clearvars -except results;
        perc=i*5;
        plot_choice=0;
        MRAC_startup;
        results(i,j)=calculateMetrics(cascaded,cascaded_adaptive);
    end
end