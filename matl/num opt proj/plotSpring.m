function plotSpring(worldMin, worldMax, groundHeight, restLen, cm, d, color, wid)
  xs = [cm(1), cm(1)];
  len = restLen + d;
  ys = [cm(2) - .5 * len, cm(2) + .5 * len];
  
  xlim([worldMin(1) worldMax(1)]);
  ylim([worldMin(2) worldMax(2)]);
  
  % draw spring
  line(xs, ys, 'LineWidth', wid, 'Color', color);
  
  % draw ground
  line([worldMin(1) worldMax(1)], [groundHeight groundHeight], 'LineWidth', 5, 'Color', [.9 .9 .9]);
end