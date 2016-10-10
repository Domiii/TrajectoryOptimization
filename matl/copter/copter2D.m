function copter2D()
    nDims = 2;

    % setup copter geometry
    g = [0; -9.8];          % gravitational acceleration
    dampingFactor = .1;     % damping of linear & angular velocities
    ms = [1, 1];            % mass of rotors
    m = 4;                  % mass of entire copter
    inertia = 6;            % inertia scalar
    nRotors = 2;            % the amount of rotors
    rotPos = zeros(2, 2);   % position of rotors from CM
    rotPos(:, 1) = [-1; 1];
    rotPos(:, 2) = [1; 1];
    
    mInv = 1/m;
    inertiaInv = inv(inertia);
    
    % setup world and simulation parameters
    axis equal;
    worldMin = [0; 0];
    worldMax = [40; 50];
    h = .04;            % simulation time step
    T = [0; 100];       % start and end time
    %damping = 1.01;
    maxErr = 10e-4;
    dynAccuracy = odeset('RelTol', maxErr, 'AbsTol', maxErr);
    
    % control parameters
    fMax = 50;
    us = [fMax; .1 * fMax];              % forces of rotors
    
    % copter state
    initPos = (worldMax - worldMin)/2;      % position of CM
    initVel = [0; 0];                       % linear velocity
    initAngle = 0;                          % angle from vertical
    initAngVel = 0;                         % angular velocity
    
    idxPos1 = 1; idxPos2 = 2; idxPos = idxPos1:idxPos2;
    idxVel1 = 3; idxVel2 = 4; idxVel = idxVel1:idxVel2;
    idxAng = 5; idxAngVel = 6;
    curX = [initPos; initVel; initAngle; initAngVel; 1];
    
    % setup GUI controls
    uicontrol('Style', 'slider',...
        'Min',0,'Max',fMax,'Value',fMax,...
        'Position', [20 10 120 20],...
        'Callback', {@controlF1});
    uicontrol('Style', 'slider',...
        'Min',0,'Max',fMax,'Value',fMax,...
        'Position', [20 35 120 20],...
        'Callback', {@controlF2}); 
    timeText = uicontrol('Style','text',...
        'Position',[400 10 120 20],...
        'String','Time');
    
    % setup drawing parameters
    copterColor = [0 1 0];
    xlim([worldMin(1) worldMax(1)]);
    ylim([worldMin(2) worldMax(2)]);
    
    R = rot2D(initAngle);
    copterControls = cell(nRotors, 1);
    for k=1:nRotors
        v = initPos + R * rotPos(:, k);
        nextLine = imline(gca, [initPos(1) v(1)], [initPos(2) v(2)]);
        copterControls{k} = nextLine;
        setColor(nextLine, copterColor);
    end
    
    
    % run update loop
    for t = T(1):h:T(2)
        % integrate forward in time
        [ts, xs] = ode45(@dynamics, [t, t + h], curX, dynAccuracy);
        curX = xs(length(xs(:,1)), :)';
        
        % compute LQR
        R = rot2D(curX(idxAng));
        A = [...
            0, 1, 0, 0, 0; ...
            0, -d, 0, 0, g; ...
            0, 0, 0, 1, 0; ...
            0, 0, 0, -d, 0
            ];
        
        B = [ [0; 0], [0; 0]; ...
            mInv * R, mInv * R; ...
            inertiaInv * diag(-r1(2), r1(1)), ...
            inertiaInv * diag(-r2(2), r2(1))];
        
        % produce the cost matrices
        % to gain intuition about the cost matrices,
        % see http://www.youtube.com/watch?v=St5L-ekOKGA
        
        % TODO
        QCost = diag([2 .* 1 / (pi)^2, 1/maxVEstimate^2]);
        %R = [1/(2*uMax)^2];
        RCost =  .5 .* [1/(2*uMax)^2];
        [K, S, E] = lqr(A, B, QCost, RCost);
        
        % (very rudimentary) physical boundary handling
        if (curX(idxPos1) > worldMax(1) - 2)
            curX(idxPos1) = worldMax(1) - 2;
            curX(idxVel1) = - .5 * curX(idxVel1);
        end
        if (curX(idxPos2) > worldMax(2) - 2)
            curX(idxPos2) = worldMax(2) - 2;
            curX(idxVel2) = - .5 * curX(idxVel2);
        end
        if (curX(idxPos1) < worldMin(1) + 2)
            curX(idxPos1)= worldMin(1) + 2;
            curX(idxVel1) = - .5 * curX(idxVel1);
        end
        if (curX(idxPos2) < worldMin(2) + 2)
            curX(idxPos2) = worldMin(2) + 2;
            curX(idxVel2) = - .5 * curX(idxVel2);
        end
        
        % update GUI
        for k=1:nRotors
            nextLine = copterControls{k};
            v = curX(idxPos) + R * rotPos(:, k);
            setPosition(nextLine, [curX(idxPos1) v(1)], [curX(idxPos2) v(2)]);
        end
        
        set(timeText,'String', ['Time: ' num2str(t, '%.1f')]);
        pause(.001);
    end
    
    function xDot = dynamics(t, x)
        if (x(idxAngVel) > 2 * pi)
          x(idxAngVel) = x(idxAngVel) - 2 * pi;
        end
        R = rot2D(x(idxAng));
        u1 = [0; us(1)];
        u2 = [0; us(2)];
        r1 = rotPos(:, 1);
        r2 = rotPos(:, 2);
        xDot = [x(idxVel); ...
            g + mInv * R * (u1+u2) - dampingFactor * x(idxVel); ...
            x(idxAngVel);  ...
            inertiaInv * (cross2(r1, u1) + cross2(r2, u2)) - dampingFactor * x(idxAngVel); ...
            0];
    end

    % 2D rotation matrix
    function R = rot2D(phi)
        R = [[cos(phi), -sin(phi)]; [sin(phi), cos(phi)]];
    end

    function v = cross2(a, b)
        v = a(1) * b(2) - a(2) * b(1);
    end

    % called when f1 is changed by user
    function controlF1(hObj, event)
        val = get(hObj,'Value');
        us(1) = val;
    end

    % called when f2 is changed by user
    function controlF2(hObj,event)
        val = get(hObj,'Value');
        us(2) = val;
    end
end