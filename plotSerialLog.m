clear;
nextFig = 1;

logFile = 'C:\Users\JohnMcCall\Desktop\serialLog.txt';

endTime = datetime('03/10/2022 22:15:00');

Vdiff_ref = 0.01:0.01:0.10;
durationBinEdges = (0:5:5:60)';

%%
rawText = strtrim(fileread(logFile));

if any(strfind(rawText,[char(13) char(10)]))
    lineDelimiter = '\r\n';
elseif any(rawText == char(13))
    lineDelimiter = '\r';
elseif any(rawText == char(10))
    lineDelimiter = '\n';
end

% Cell column vector containing one row/line of text per cell
rows = textscan(rawText,'%s','delimiter',lineDelimiter);
rows = rows{1};
headerRow = find(strcmpi(rows,'Entering data logging mode')) + 2;

%%
logData = parseTextTable(logFile,'delimiter','\t','headerRow',headerRow);

t_s = logData.time_ms / 1000;
time = endTime - seconds(t_s(end) - t_s);
Vdiff = logData.Vstart - logData.Vhouse;
contactorClosed = logData.state >= 3;
n_closings = sum(diff(contactorClosed)==1);
n_closings_per_day = n_closings / ((t_s(end) - t_s(1)) / (24*3600));

durationCount = nan(length(durationBinEdges),length(Vdiff_ref));
for x = 1:length(Vdiff_ref)
    goesHigh = find(Vdiff(2:end) >= Vdiff_ref(x) & Vdiff(1:end-1) < Vdiff_ref(x));
    goesLow = find(Vdiff(2:end) <= Vdiff_ref(x) & Vdiff(1:end-1) > Vdiff_ref(x));
    
    if ~(isempty(goesHigh) || isempty(goesLow))
        while goesHigh(1) > goesLow(1)
            goesLow(1) = [];
        end
        while length(goesHigh) > length(goesLow)
            goesHigh(end) = [];
        end
        while length(goesLow) > length(goesHigh)
            goesHigh(end) = [];
        end
        duration = goesLow - goesHigh;
        
        durationCount(:,x) = histc(duration,durationBinEdges);

        figure(nextFig); clf; nextFig = nextFig + 1;
        bar(durationBinEdges,durationCount(:,x),'histc');
        grid on
        title(num2str(Vdiff_ref(x)));
    end
end

executionTime_ms = diff(logData.time_ms) - 1000;
figure(nextFig); clf; nextFig = nextFig + 1;
hist(executionTime_ms,0:5)
grid on
title('Execution time (milliseconds)');
pause(1e-2);

figure(nextFig); clf; nextFig = nextFig + 1;
plot(time,logData.Vstart,time,logData.Vhouse)
grid on
legend('Starting','House');
xlabel('Time (seconds)');
pause(1e-2);

figure(nextFig); clf; nextFig = nextFig + 1;
plot(time,logData.Vstart,time,logData.Vhouse)
grid on
legend('Starting','House');
xlabel('Time (seconds)');
pause(0.5);

figure(nextFig); clf; nextFig = nextFig + 1;
clear ax
ax(1) = subplot(2,1,1);
plot(time,logData.Vstart,time,logData.Vhouse)
grid on
legend('Starting','House');
xlabel('Time (seconds)');
pause(1e-2);
ax(2) = subplot(2,1,2);
plot(time,logData.state,time,logData.nextState)
grid on
legend('State','Next state');
xlabel('Time (seconds)');
linkaxes(ax,'x');
pause(1e-2);