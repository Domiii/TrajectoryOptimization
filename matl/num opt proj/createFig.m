% create a new figure
function fig = createFig()
global figN;
if (~exist('figN', 'var'))
  figN = 1;
end
fig = figure(figN); figN = figN+1; clf;
end

% render latex in Matlab
function printLog(logText)
fig = createFig();
set(fig, 'Position', [.1 .1 1200 1000]);
ha = annotation('textbox',[0 0 1 1], 'Interpreter', 'latex');
set(ha, 'String', logText, ...
  'FontName', 'Arial', ...
  'FontSize', 18);
%set(ha, 'String',  '\begin{tabular}{l}  \begin{tabular}{cc} 1.00& 0.00 \\ 2 & 3 \end{tabular} \right] \end{tabular}');
end

% write the given text to the given file
function writeToFile(fname, text)
fileID = fopen(fname, 'wb');
fprintf(fileID, '%s', text);
fclose(fileID);
end

% returns transformFun(i, v(i)) for all v(i) for which filter(v(i)) is true
function [indices, res] = filterTransform(v, filterFun, transformFun)
n = length(v);
res = zeros(n, 1);
indices = zeros(n, 1);
k = 1;
for i = 1:n
  if (filterFun(v(i)))
    res(k) = transformFun(i, v(i));
    indices(k) = i;
    k = k+1;
  end
end
res = res(1:k-1);
indices = indices(1:k-1);
end

function drawPlane(domain, normal, d, color, alpha)
[X,Y] = meshgrid(domain);
Z=(d - normal(1) * X - normal(2) * Y)/normal(3);
s = surf(X,Y,Z);
set(s,'FaceColor',color,'FaceAlpha',alpha);
end