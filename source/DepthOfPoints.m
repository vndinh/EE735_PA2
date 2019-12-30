function depth = DepthOfPoints(X,P)
  n = size(X,2);
  depth = zeros(1,n);
  c = FindCameraCenter(P);
  c = [c; 1];
  R = P(:,1:3);
  for i = 1:n
    w = R(3,:)*(X(1:3,i)-c(1:3,:));
    depth(i) = (sign(det(R))*w)/(X(4,i)*norm(R(3,:)));
  end
end