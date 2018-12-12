clear all
close all

%% Flick 1
time = 5; %seconds
tag = [ "flick","shake","baseline" ];
for i=1:length(tag)
    filename = [strcat('accl-',tag(i),'.csv'),...
                ];
    accl = csvread(filename(1));

    figure
    hold on;
    title(strcat('accl-',tag(i)))
    xlabel("Time")
    ylabel("Accl")
    %ylim([min(accl(:,1))-10 max(accl(:,1))+10])
    
    hold on;
    n = length(accl);
    plot(1/n:10/n:10,accl(1:n,1),'b')
    plot(1/n:10/n:10,accl(1:n,2),'g')
    plot(1/n:10/n:10,accl(1:n,3),'r')
    
    legend({'x','y','z',})
    
    hold off;
    figure;
        title(strcat(strcat('accl-',tag(i)), ' Shake'))
    plot(1/n:10/n:10,accl(1:n,4))
end

