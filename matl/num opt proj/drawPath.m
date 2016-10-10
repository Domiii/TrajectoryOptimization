% draw search path
% and make sure that line is above surface
function drawPath(steps, f, lineOptions)
lineZ = evalZ(steps(1,:), steps(2, :), f);

plot3(steps(1, :), steps(2, :), lineZ + 2, lineOptions, ...
  'LineWidth',1,...
  'MarkerEdgeColor','g',...
  'MarkerFaceColor','g',...
  'MarkerSize',2.5);
end

% Evaluate f at all points [xs(i); ys(i)]
function zs = evalZ(xs, ys, f)
nx = length(xs);
zs = zeros(nx, 1);
for i = 1:nx
  zs(i) = f([xs(i); ys(i)]);
end
end