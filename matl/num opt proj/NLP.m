% NLP utility class
classdef NLP
  methods(Static)
    function init()
      global NLPinit gxxxx;
      %if (~exist('NLPinit', 'var'))
        NLPinit = 1;
        
        % init
        gxxxx = 123;
      %end
    end
    
    % Returns the k'th position&velocity from trajectory vector Q
    function x = getX(Q, k)
      global nDimsX nDimsU;
      
      idx = (nDimsX + nDimsU) * (k - 1) + 1;
      x = Q(idx:idx+nDimsX-1);
    end
    
    % Returns the k'th actuation u from trajectory vector Q
    function x = getU(Q, k)
      global nDimsX nDimsU;
      
      idx = (nDimsX + nDimsU) * (k - 1) + 1 + nDimsX;
      x = Q(idx:idx+nDimsU-1);
    end
    
    % Returns the k'th point q from trajectory vector Q
    function [x, u] = getq(Q, k)
      global nDimsX nDimsU;
      
      idx = (nDimsX + nDimsU) * (k - 1) + 1;
      x = Q(idx:idx+nDimsX-1);
      u = Q(idx+nDimsX:idx+nDimsX+nDimsU-1);
    end
  end
end