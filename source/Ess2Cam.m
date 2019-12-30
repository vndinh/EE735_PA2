function P = Ess2Cam(E)
  [U,~,V] = svd(E);
  W = [0 1 0; -1 0 0; 0 0 1];
  Ra = U*W*V';
  Rb = U*W'*V';
  t = U(:,3)./max(abs(U(:,3)));
  P = zeros(3,4,4);
  P(:,:,1) = [Ra,t];
  P(:,:,2) = [Ra,-t];
  P(:,:,3) = [Rb,t];
  P(:,:,4) = [Rb,-t];
end