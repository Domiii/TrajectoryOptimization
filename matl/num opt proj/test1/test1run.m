% simple continuous pathfinding
function test1run()

startPos = [-8;-8];
goalPos = [10;10];
goalDistCostFactor = 15;
nDimsX = 2;                             % state space dimensions (x = p & v)
nDimsU = 2;                             % control dimensions (u)
nDims = nDimsX + nDimsU;
T = 10;                                 % max time
nSteps = 50;                            % how many steps to take
xMax = 10;
uMax = 3;


% cost function geometry

bumpSigmas = [...
    [1;5],...
    [2;4],...
    [5;3],...
    [5;6]
    ];
bumpCenters = [...
    [-1; -1],...
    [1;9],...
    [3;8],...
    [7;2]
    ];

bumpHeights = [500; 700; 700; 800];
 

% misc setup
uMaxSq = uMax * uMax;         % min & max actuation
xBoundsL = [-xMax, -xMax]; xBoundsU = [xMax, xMax];   % box bounds on state space
h = T/nSteps;


doRun();

  function doRun()
    % initial trajectory = straight line from start to goal
    % but since start & goal are fixed, we can ommit them
    x0 = zeros(nDimsX, nSteps-1);
    deltaStep = (goalPos - startPos) ./ nSteps;
    for k = 1:nSteps-1
      x0(:, k) = startPos + k * [0; h * 1.5];
    end
    
    % intial open control loop
    u = uMax * ones(nDimsU, nSteps);
    
    % flatten into a single state array, representing the trajectory
    [Q0, boundsLow, boundsUp] = convertTrajToNLP(x0, u, xBoundsL, xBoundsU, uMax);
    
    %Aeq = zeros(nDims, length(Q0));
    %beq = zeros(nDims, 1);
    
    % optimize trajectory
    options=optimset('Algorithm','sqp', 'UseParallel','never', 'GradObj', 'on', 'GradConstr','on');
    %options=optimset('Algorithm','sqp', 'UseParallel','never', 'GradObj', 'on');
    %options=optimset('Algorithm','sqp', 'UseParallel','never');
    %options=optimset('Algorithm','sqp', 'UseParallel','always');
    %'Display', 'iter-detailed' ...
    %);
    tic
    [Qf, fval, exitFlag, output] = fmincon(...
      @costToGo, Q0, [], [], [], [], boundsLow, boundsUp, @constraints, options);
    toc
    
    %Qf
    %exitFlag
    fval
    output
    
    %contourPretty(100, @cost, x0);
    
    contourPretty(100, @cost, getStatesFromTraj(Qf));
    %drawPath(getStatesFromTraj(Q0), @cost, '-bs');
  end


% Convert trajectory optimization to NLP problem
% x0 = Initial trajectory
% u0 = Initial open control loop
% length(x0) == length(u0)+1
  function [Q, boundsLow, boundsUp] = convertTrajToNLP(x0, u0, xBoundsL, xBoundsU, uMax)
    nOut = nSteps * (nDimsX + nDimsU) - nDimsX;
    
    % define NLP variables (all xs and us, flattened into a vector)
    Q = zeros(nOut, 1);
    boundsLow = zeros(nOut, 1);
    boundsUp = zeros(nOut, 1);
    uidx = 1:1+nDimsU-1;
    xidx = 1+nDimsU:1+nDims-1;
    for k = 1:nSteps
      Q(uidx) = u0(:, k);
      boundsLow(uidx) = -uMax * ones(nDimsU, 1);
      boundsUp(uidx) = uMax * ones(nDimsU, 1);
      if (k < nSteps)
        Q(xidx) = x0(:, k);
        boundsLow(xidx) = xBoundsL;
        boundsUp(xidx) = xBoundsU;
      end
      xidx = xidx + nDims; uidx = uidx + nDims;
    end
  end


  % non-linear constraints of problem 1
  function [ci, ce, ciGrad, ceGrad] = constraints(Q)
    % define constraints due to dynamics
    nEConstraints = nDimsX * nSteps;
    ce = zeros(nEConstraints, 1);
    
    % dynamics
    idxCe = 1:nDimsX;
    lastX = startPos;
    for k = 1:nSteps-1
      [x, u] = getq(Q, k);
      ce(idxCe) = x - lastX - h * u;
      idxCe = idxCe + nDimsX;
      lastX = x;
    end
    % add final piece
    ce(idxCe) = goalPos - lastX - h * getU(Q, nSteps);
    
    % inequality constraints
    nIConstraints = nSteps;
    ci = zeros(nIConstraints, 1);
    
    % constrain u
    for k = 1:nSteps
      u = getU(Q, k);
      ci(k) = u' * u - uMaxSq;
    end
    
    if nargout > 2                                % fun called with 4 output arguments
      [ciGrad, ceGrad] = constraintsJacobian(Q);      % Compute the gradient, too
    end
  end

  % Jacobian of constraints
  function [ciGrad, ceGrad] = constraintsJacobian(Q)
    nEConstraints = nDimsX * nSteps;
    n = length(Q);
    ceGrad = zeros(n, nEConstraints);
    
    % add all inequality gradients
    idxCe = 1:nDimsX;
    ident = eye(nDimsX);
    theHs = -h * eye(nDimsU);
    for k = 1:nSteps
      [xIdx, uIdx] = getqIdx(k);
      if (k < nSteps)
        ceGrad(xIdx, idxCe) = ident;          % x
      end
      if (k > 1)
        ceGrad(xIdx-nDims, idxCe) = -ident;   % lastX
      end
      ceGrad(uIdx, idxCe) = theHs;              % u
      idxCe = idxCe + nDimsX;
    end
    
    % Jacobian of inequality constraints
    nIConstraints = nSteps;
    ciGrad = zeros(n, nIConstraints);
    
    % gradient for constraining u
    for k = 1:nSteps
      uIdx = getUIdx(k);
      u = Q(uIdx);
      
      ciGrad(uIdx, k) = 2 * u;
    end
  end


  % gaussian bump
  function g = gaussianBump(x, i)
    sigma = bumpSigmas(:, i);
    center = bumpCenters(:, i);
    hh = bumpHeights(i) ;
    y = -sum(arrayfun(@(xi, centeri, sigmai) (xi - centeri)^2/(2 * sigmai^2), x, center, sigma));
    g = hh * exp(y);
    
    % plateau-style Gaussian bumps:
    %   g1 = bump(x(1), 1, 5, 0);
    %   g2 = bump(x(2), 1, 6, 0);
    %   g = g1 * g2;
    
    %   function g = bump(x, scale, r, center)
    %     %scale = .5 * scale * exp(-1);
    %     if (norm(x) < r)
    %       g = exp(-scale/(r*r - (x + center)^2));
    %     else
    %       g = 0;
    %     end
    %   end
  end

  % gradient of a Gaussian bump
  function grad = gaussianBumpGrad(x, i)
    sigma = bumpSigmas(:, i);
    center = bumpCenters(:, i);
    hh = bumpHeights(i) ;
    y = -sum(arrayfun(@(xi, centeri, sigmai) (xi - centeri)^2/(2 * sigmai^2), x, center, sigma));
    grad = -hh * exp(y) * arrayfun(@(xi, centeri, sigmai) (xi - centeri)/(sigmai^2), x, center, sigma);
  end

% cost function g of
  function g = cost(x)
    % add bumps
    g = 0;
    for i = 1:length(bumpSigmas)
        g = g + gaussianBump(x, i);
    end
    
    % plus some distance to goal measure
    g = g + .5 * norm(x - goalPos);
  end

  function gGrad = costGrad(x)
    % add bumps
    gGrad = zeros(length(x), 1);
    for i = 1:length(bumpSigmas)
        gGrad = gGrad + gaussianBumpGrad(x, i);
    end
    
    % plus some gradient of norm
    gGrad = gGrad + .5 * normGrad(x, goalPos);
  end

  % the gradient of norm(x1 - x2) with respect to x1
  function grad = normGrad(x1, x2)
    grad = (x1 - x2) / norm(x1 - x2);
  end

% cost-to-go is the sum of the cost of all x and total trajectory length
  function [J, jGrad] = costToGo(Q)
    idx = 1;
    J = 0;
    lastX = startPos;
    len = 0;
    for k = 1:nSteps-1
      x = getX(Q, k);
      J = J + cost(x);                % terrain cost
      len = len + norm(x - lastX);    % total path length
      idx = idx + nDimsX + nDimsU;
      lastX = x;
    end
    J = h * J;
    len = len + norm(x - goalPos);    % add final path segment
    J = J + goalDistCostFactor * len;
    if nargout > 1                    % fun called with 2 output arguments
      jGrad = JGrad(Q);               % Compute the gradient, too
    end
  end

% gradient of the cost-to-go function
  function Jgrad = JGrad(Q)
    Jgrad = zeros(nSteps * (nDimsX + nDimsU) - nDimsX, 1);
    idx = 1;
    lastX = startPos;
    for k = 1:nSteps-1
      [xIdxs] = getXIdx(k);
      x = Q(xIdxs);
      Jgrad(xIdxs) = h * costGrad(x);                 % terrain cost
      if (k > 1)
        Jgrad(xIdxs-nDims) = Jgrad(xIdxs-nDims) + goalDistCostFactor * normGrad(lastX, x); % lastX-grad of norm
      end
      Jgrad(xIdxs) = Jgrad(xIdxs) + goalDistCostFactor * normGrad(x, lastX);   % x-grad of norm
      %Jgrad(uIdxs) = h;
      idx = idx + nDimsX + nDimsU;
      lastX = x;
    end
    [xIdxs] = getXIdx(nSteps-1);
    Jgrad(xIdxs) = Jgrad(xIdxs) + goalDistCostFactor * normGrad(x, goalPos);  % add final path segment
    %Jgrad(uIdxs) = h;
  end

  function x = getStatesFromTraj(Q)
    x = zeros(nDimsX, nSteps+1);
    x(:, 1) = startPos;
    x(:, nSteps+1) = goalPos;
    for k = 1:nSteps-1
      x(:, k+1) = getX(Q, k);
    end
  end

% Returns the index set of the k'th position&velocity from trajectory vector Q
  function idxs = getXIdx(k)
    idx = (nDims) * (k - 1) + nDimsU + 1;
    idxs = idx:idx+nDimsX-1;
  end

% Returns the k'th actuation u from trajectory vector Q
  function idxs = getUIdx(k)
    idx = (nDims) * (k - 1) + 1;
    idxs = idx:idx+nDimsU-1;
  end

  function [xidxs, uidxs] = getqIdx(k)
    idx = (nDims) * (k - 1) + nDimsU + 1;
    xidxs = idx:idx+nDimsX-1;
    idx = (nDims) * (k - 1) + 1;
    uidxs = idx:idx+nDimsU-1;
  end

% Returns the k'th position&velocity from trajectory vector Q
  function x = getX(Q, k)
    idx = (nDims) * (k - 1) + nDimsU + 1;
    x = Q(idx:idx+nDimsX-1);
  end

% Returns the k'th actuation u from trajectory vector Q
  function u = getU(Q, k)
    idx = (nDims) * (k - 1) + 1;
    u = Q(idx:idx+nDimsU-1);
  end

% Returns the k'th point q from trajectory vector Q
  function [x, u] = getq(Q, k)
    idx = (nDims) * (k - 1) + 1;
    u = Q(idx:idx+nDimsU-1);
    x = Q(idx+nDimsU:idx+nDims-1);
  end
end
