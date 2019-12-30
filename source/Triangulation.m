function X = Triangulation(x1,x2,P1,P2)
  % Convert to homogeneous coordinates
  %x1 = [x1; ones(1,size(x1,2))];
  %x2 = [x2; ones(1,size(x2,2))];
  %p1 = eye(3,4);
  %p2 = P;
  % AX = 0
  n = size(x1,2);
  X = zeros(4,n);
  for i = 1:n
    A1 = x1(1,i)*P1(3,:) - P1(1,:);
    A2 = x1(2,i)*P1(3,:) - P1(2,:);
    A3 = x2(1,i)*P2(3,:) - P2(1,:);
    A4 = x2(2,i)*P2(3,:) - P2(2,:);
    A = [A1; A2; A3; A4];
    [~,~,V] = svd(A);
    X(:,i) = V(:,4)./V(4,4);
  end
end