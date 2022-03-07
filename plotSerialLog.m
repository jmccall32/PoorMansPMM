clear;
nextFig = 1;

logFile = 'C:\Users\JohnMcCall\Desktop\serial.txt';
headerRow = 177;

Vdiff_ref = 0.04:0.01:0.10;
durationBinEdges = (0:5:5:60)';

%%
logData = parseTextTable(logFile,'delimiter','\t','headerRow',headerRow);

Vdiff = logData.Vstart - logData.Vhouse;

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

figure(nextFig); clf; nextFig = nextFig + 1;
plot(logData.time_ms,logData.Vstart,logData.time_ms,logData.Vhouse)
grid on
legend('Starting','House');

figure(nextFig); clf; nextFig = nextFig + 1;
plot(logData.time_ms,logData.Vstart - logData.Vhouse)
grid on
legend('Starting','House');