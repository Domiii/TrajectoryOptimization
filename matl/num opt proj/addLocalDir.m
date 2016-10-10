function res = addLocalDir(dir)
  newDir = [pwd dir];
  if (~exist(newDir, 'dir'))
    res = 0;
    return;
  end
  if (isempty(regexp(newDir, path, 'ONCE')))
    path(newDir, path);
  end
  res = 1;
end