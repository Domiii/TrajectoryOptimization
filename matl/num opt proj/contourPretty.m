
% Pretty version of a contour plot with steps taken on it's surface.
% f must accept a 2-dimensional state vector.
% steps contains points of a segmented line to be drawn ontop of the surface.
% The contour colors are saturated at the min and max of f of all steps.
  function contourPretty(nX, f, steps)
    % compute the bounding box in all three directions
    bboxOvershoot = .1;
    maxCamHeight = 10e8;
    
    lineZ = evalZ(steps(1,:), steps(2, :), f);
    minZ = min(lineZ); maxZ = max(lineZ);
    minX = [min(steps(1, :)); min(steps(2, :))];
    maxX = [max(steps(1, :)); max(steps(2, :))];
    boundDist = maxX - minX;
    minX = minX - boundDist * bboxOvershoot;
    maxX = maxX + boundDist * bboxOvershoot;
    dX = (maxX - minX) / nX;
    
    % setup the grid
    mxs = minX(1):dX(1):maxX(1);
    mys = minX(2):dX(2):maxX(2);
    camPos = minX + .5 * (maxX - minX);   % center the camera
    [xs, ys] = meshgrid(mxs, mys);
    
    % evaluate f at all grid points
    zs = evalZGrid(mxs, mys, f);
    
    % color setting
    minC = minZ; maxC = maxZ;
    %laplacianZs = del2(zs);
    %minC = min(laplacianZs); maxC = max(laplacianZs);
    
    %   [C, handle] = contourf(xs, ys, zs);
    %   set(handle, ...
    %     'ShowText', 'on', ...
    %     'TextStep', get(handle,'LevelStep')*2 ...
    %   );
    
    % draw surface
    %handle = surf(xs, ys, zs, laplacianZs);
    handle = surf(xs, ys, zs);
    set(handle, ...
      'LineWidth',1,...
      'CDataMapping','scaled');
    shading(gca,'interp');
    %shading(gca,'flat');
    
    % show color bar
    colorbar('location','southoutside');
    hold on;
    
    % set color saturation
    %caxis([minC; maxC]);
    
    % look from high above
    set(gca, ...
      'CameraPosition', [camPos(1), camPos(2), maxCamHeight], ...
      'CameraTarget', [camPos(1), camPos(2), 0] ...
      );
    
    % draw the steps on top of things
    drawPath(steps, f, '-rs');
    hold on;
  end

% Evaluate f at all points [xs(i); ys(i)]
  function zs = evalZ(xs, ys, f)
    nx = length(xs);
    zs = zeros(nx, 1);
    for i = 1:nx
      zs(i) = f([xs(i); ys(i)]);
    end
  end

% Evaluate f over the mesh of xs and ys
  function zs = evalZGrid(xs, ys, f)
    nx = length(xs); ny = length(ys);
    zs = zeros(ny, nx);
    for j = 1:ny
      for i = 1:nx
        zs(j, i) = f([xs(i); ys(j)]);
      end
    end
  end