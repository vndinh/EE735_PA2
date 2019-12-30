function h = HomoCoord(p, p2Dor3D)
  [m,n] = size(p);
  if strcmp(p2Dor3D, '2D')
    if m == 2
      h = [p; ones(1,n)];
    elseif m == 3
      h(1,:) = p(1,:)./p(3,:);
      h(2,:) = p(2,:)./p(3,:);
      h(3,:) = p(3,:)./p(3,:);
    end
  elseif strcmp(p2Dor3D, '3D')
    if m == 3
      h = [p; ones(1,n)];
    elseif m == 4
      h(1,:) = p(1,:)./(p(4,:)+eps);
      h(2,:) = p(2,:)./(p(4,:)+eps);
      h(3,:) = p(3,:)./(p(4,:)+eps);
      h(4,:) = p(4,:)./(p(4,:)+eps);
    end
  end
end