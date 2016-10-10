function testBed()
  % add snopt to path
  snoptDir = '/snopt/';
  addLocalDir(snoptDir);
  
  % add tests to path
  for i = 1:100
    if (~addLocalDir(['/test' num2str(i)]))
      break;
    end
  end
  
  global figN;
  figN = 1;
%   
%   % test global access
%   xxx = 0;
%   tic 
%   for i = 1:1e7
%     xxx = xxx + gxxxx + gxxxx+ gxxxx;
%   end
%   toc
%   xxx
%   
%   % test constant access
%   xxx = 0;
%   tic 
%   for i = 1:1e7
%     xxx = xxx + 122;
%   end
%   toc
%   xxx

  %matlabpool close
  %matlabpool open 4
  
  %test1run();
  [cms,ds,us,lambdas,worldMin,worldMax,groundHeight,restLen,qs,qg] = test2run();
  for i = 1:length(us)
    %us{i}
  end
  lambdas
  
  %function plotSpring(worldMin, worldMax, groundHeight, restLen, cm, d)
  n = max(size(ds));
  colors = lines(n);
  fig = createFig();
  dx = (worldMax(1) - worldMin(1)) / (n+1);
  x = worldMin(1) + dx;
  color = colors(i, :);
  
  % plot start state
  plotSpring([0 worldMin(2)], [n+1 worldMax(2)], groundHeight, restLen, [x, qs(2)], qs(5), [0,0,0], 4);
  
  for i = 2:n-1
    cm = cms{i};
    d = ds{i};
    color = colors(i, :);
    %subplot(ceil(n/2), 2, i, 'replace');
    x = x + dx;
    plotSpring([0 worldMin(2)], [n+1 worldMax(2)], groundHeight, restLen, [x, cm(2)], d, color, 3);
    title([num2str(i) '. d = ' num2str(d)]);
    hold on;
  end
    x = x + dx;
  % plot goal state
  plotSpring(worldMin, worldMax, groundHeight, restLen, [x, qg(2)], qg(5), [0,0,0], 4);
end