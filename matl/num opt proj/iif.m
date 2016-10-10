
% inline if
  function result = iif(condition,trueResult,falseResult)
    if condition
      result = trueResult;
    else
      result = falseResult;
    end
  end