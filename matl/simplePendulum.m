% Simulates an actuated simple pendulum
% Stabilizes it in the horizontal position, despite random wind
function simplePendulum()
set(gca,'DefaultTextInterpreter', 'latex');   % use latex for output
% pendulum parameters
m = 1; g = -9.8;
l = 1;

damp = 0.5;
uMax = 20;
maxWindVel = 3; windChange = .1;
maxVEstimate = pi;

t = 0;
%h = .01;
h = .1;
T = 200;

stopIt = 0;
actuated = 0;
windy = 0;

maxErr = 10^(-8);

figN = 11;
x0 = [pi/2 + .1; 0];
xGoal = [0; 0];
%x0 = [pi; 0];

win = figure(figN);
clf;
%allXs = zeros(N, 2);
%imagesc([th(1,1) th(end,1)],[thdot(1,1) thdot(1,end)],basin');

acc = odeset('RelTol', maxErr, 'AbsTol', maxErr);
curX = x0;

% setup GUI
stopButton = uicontrol('Style', 'pushbutton', ...
  'Position', [0 0 100 30], ...
  'String', 'Running', ...
  'Callback', @stopButtonClicked);
uButton = uicontrol('Style', 'pushbutton', ...
  'Position', [110 0 100 30], ...
  'String', iif(actuated, 'Actuated', 'Not Actuated'), ...
  'Callback', @uButtonClicked);
windButton = uicontrol('Style', 'pushbutton', ...
  'Position', [220 0 100 30], ...
  'String', iif(windy, 'Windy', 'No Wind'), ...
  'Callback', @windButtonClicked);

pendulObj = 0;
axis([-l l -l l]);
axis manual;
phi = curX(1);
while true
  if (~stopIt)
    % decide on the actuation
    if (actuated)
      % Compute the estimated dynamics
      if (abs(phi) < .01)
        alpha = phi;
      else
        alpha = sin(phi) / phi;
      end
      A = [0, 1;  -g/l * alpha, -damp/(m * l^2)];
      B = [0; 1];
      
      % produce the cost matrices
      % to gain intuition about the cost matrices,
      % see http://www.youtube.com/watch?v=St5L-ekOKGA
      Q = diag([2 .* 1 / (pi)^2, 1/maxVEstimate^2]);
      %R = [1/(2*uMax)^2];
      R =  .5 .* [1/(2*uMax)^2];
      N = B .* .0001;   % we don't want this to have much of an influence at all
      [K, S, E] = lqr(A, B, Q, R, N);
      u = -K * (curX - xGoal);
      u = sign(u) * min(abs(u), uMax);  % clamp to [-uMax, uMax]
    else
      u = 0;
    end
    if (windy)
      wind = wind + (-1 + 2 * rand(1, 1)) * windChange;
      wind = sign(wind) * min(abs(wind), maxWindVel);
    else
      wind = 0;
    end
  end
  
  % simulate dynamics
  [ts, xs] = ode45(@dynamics, [t, t + h], curX, acc);
  %allXs(i:i+len-1, :) = xs;
  t = t + h;
  
  % draw pendulum
  %clf;
  curX = xs(length(xs(:,1)), :)';
  phi = curX(1);
  if (pendulObj && ishandle(pendulObj))
    delete(pendulObj);
  end
  px = [0, l * sin(phi)];
  py = [0, l * cos(phi)];
  pendulObj = line(px, py);
  title(['u = ' num2str(u, '%.03f') ', wind = ' num2str(wind, '%.03f')]);
  pause(.02);
  if (~ishandle(win))
    % window closed
    break;
  end
end

  function stopButtonClicked(hObject,eventdata)
    stopIt = ~stopIt;
    set(stopButton, 'String', iif(stopIt, 'Stopped', 'Running'));
  end

  function uButtonClicked(hObject, eventData)
    actuated = ~actuated;
    set(uButton, 'String', iif(actuated, 'Actuated', 'Not Actuated'));
  end

  function windButtonClicked(hObject, eventData)
    windy = ~windy;
    set(windButton, 'String', iif(windy, 'Windy', 'No Wind'));
    wind = maxWindVel/2;
  end



%imagesc([th(1,1) th(end,1)],[thdot(1,1) thdot(1,end)],basin');
%axis xy; colormap('winter');colorbar;
%plot(xs(:,1), xs(:, 2), '--', 'Color', 'r');
%xlabel('theta'); ylabel('theta dot');

  function xdot = dynamics(t, x)
    % simulate dynamics, with actuation
    if (x(1) > 2 * pi)
      x(1) = x(1) - 2 * pi;
    end
    xdot = [wind * cos(x(1)) + x(2); u - g/l * sin(x(1)) - damp/(m * l^2) * x(2)];
  end

  function res = iif(cond, trueRes, falseRes)
    if (cond)
      res = trueRes;
    else
      res = falseRes;
    end
  end
end