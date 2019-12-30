function c = FindCameraCenter(P)
  x = det([P(:,2), P(:,3), P(:,4)]);
  y = -det([P(:,1), P(:,3), P(:,4)]);
  z = det([P(:,1), P(:,2), P(:,4)]);
  t = -det([P(:,1), P(:,2), P(:,3)]);
  c = [x/t; y/t; z/t];
end