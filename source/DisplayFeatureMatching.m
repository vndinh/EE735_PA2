function DisplayFeatureMatching(datapath, i1, i2, Feature, num_Match, Match)
  img_name_1 = sprintf('%04d.jpg',i1);
  img_path_1 = strcat(datapath, img_name_1);
  I1 = imread(img_path_1);
  
  img_name_2 = sprintf('%04d.jpg',i2);
  img_path_2 = strcat(datapath, img_name_2);
  I2 = imread(img_path_2);
  
  f1 = Feature(:,:,i1);
  f2 = Feature(:,:,i2);
  
  mf = Match(i1,:,:,i2);
  mf = squeeze(mf);
  nm = num_Match(i1, i2);
  mf = mf(:,1:nm);
  
  figure('Name', 'Feature Matching');
  
  subplot(1,2,1);
  frame = sprintf('Frame %04d', i1);
  title(frame);
  imshow(I1);
  hold on;
  h1 = vl_plotframe(f1(1:2,mf(1,:)));
  set(h1, 'color', 'r');
  hold off;
  
  subplot(1,2,2);
  frame = sprintf('Frame %04d', i2);
  title(frame);
  imshow(I2);
  hold on;
  h2 = vl_plotframe(f2(1:2,mf(2,:)));
  set(h2, 'color', 'r');
  hold off;
end