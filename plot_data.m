clear all
close all

%% Flick 1
%"flick","shake","baseline",
tag = [  "shake01-x-paddle1" "shake01-z-paddle1"];
for i=1:length(tag)
    filename = [strcat("accl-",tag(i),".csv"),...
                ];
            
    accl = csvread(filename(1));

    figure
    hold on;
    title(strcat('gyro-',tag(i)))
    xlabel("Time")
    ylabel("Accl")
    %ylim([min(accl(:,1))-10 max(accl(:,1))+10])
    
    n = length(accl);
    time = accl(1:n,1);
    plot(time,accl(1:n,2),'b')
    plot(time,accl(1:n,3),'g')
    plot(time,accl(1:n,4),'r')
    
    legend({'x','y','z',})
    hold off;
    
    figure;
    title(strcat('shake ',tag(i)))
    plot(time,accl(1:n,5))
    title(strcat('shake-',tag(i)))
    
    figure
    hold on;
    title(strcat('gyro-',tag(i)))
    xlabel("Time")
    ylabel("Angle")
    %ylim([min(accl(:,1))-10 max(accl(:,1))+10])
    
    n = length(accl);
    plot(time,accl(1:n,6),'b')
    plot(time,accl(1:n,7),'g')
    plot(time,accl(1:n,8),'r')
    
    legend({'x','y','z',})
    hold off;
    
    figure
    hold on;
    title(strcat('accl-',tag(i)))
    xlabel("Time")
    ylabel("Angle")
    %ylim([min(accl(:,1))-10 max(accl(:,1))+10])
    
    n = length(accl);
    plot(time,accl(1:n,9),'b')
    plot(time,accl(1:n,10),'g')
    
    legend({'x','y'})
    hold off;
    
    figure
    hold on;
    title(strcat('accl-',tag(i)))
    xlabel("Time")
    ylabel("Event")
    %ylim([min(accl(:,1))-10 max(accl(:,1))+10])
    
    n = length(accl);
    plot(time,accl(1:n,11),'b')
    
    legend({'2=shake; 1=flick; 0=static'})
end

