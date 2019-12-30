clc; clear;

addpath('./Givenfunctions');
addpath('./vlfeat');
addpath('./vlfeat/vlfeat-0.9.21');
addpath('./vlfeat/vlfeat-0.9.21/toolbox');

datapath = '../data/given_data/';

%% Define constants and parameters
% Constants ( need to be set )
number_of_pictures                  = 32;     % number of input pictures
number_of_iterations_for_5_point    = 1000;
number_of_iterations_for_3_point    = 10000;
number_of_iterations_for_LM         = 0;

% Thresholds ( need to be set )
threshold_of_distance = 1; 

% Assumptions ( need to be set )
max_number_of_features  = 6000; % maximum number of features in 1 image
max_number_of_points    = 3000; % maximum number of reconstructed points

% Matrices
K               = [ 1698.873755 0.000000     971.7497705;
                    0.000000    1698.8796645 647.7488275;
                    0.000000    0.000000     1.000000 ];
                                             
                % = intrinsic parameter matrix. 3 x 3
num_Feature     = zeros(1, number_of_pictures);                         
                % = number of features in each images. 1 x (# picture)
Feature         = zeros(4, max_number_of_features, number_of_pictures);     
                % = SIFT features in each images. 4 x (# feature) x (# picture) containing [ x y s th ]
Descriptor      = zeros(128, max_number_of_features, number_of_pictures);
                % = SIFT descriptors in each images. 128 x (# feature) x (# picture)
num_Match       = zeros(number_of_pictures, number_of_pictures);          
                % = number of matches. (# picture) x (# picture)
Match           = zeros(number_of_pictures, 2, max_number_of_features, number_of_pictures);
                % = corresponding feature index in second picture for a feature of first picture. (# picture) x (# feature) x (# picture)
X               = zeros(6, max_number_of_points);                                 
                % = reconstructed 3D points. 6 x (# point) containing [ x y z r g b ] (float)
X_exist         = zeros(1, max_number_of_points);                           
                % = 1 : X is exist. 0 : not. 1 x (# point)
Feature2X       = zeros(number_of_pictures, max_number_of_features);
                % = feature to 3D pointindex. (# picture) x (# feature)
X2Feature       = zeros(max_number_of_points, number_of_pictures);        
                % = 3D point to feature index. (# point) x (# picture)
Image_selected  = zeros(1, number_of_pictures);                      
                % = 1 : image is selected, 0 : not. (# picture)
R               = zeros(3, 3, number_of_pictures);                                
                % = Camera rotation R of images. 3 x 3 x (# picture)
T               = zeros(3, number_of_pictures);                      
                % = Camera translation t of images. 3 x (# picture)

%% Feature extraction and matching
% Load images and extract features and find correspondences.
% Fill num_Feature, Feature, Descriptor, num_Match and Match
% hints : use vl_sift to extract features and get the descriptors.
%        use vl_ubcmatch to find corresponding matches between two feature sets.
for i = 1:number_of_pictures
  img_name = sprintf('%04d.jpg',i-1);
  img_path = strcat(datapath, img_name);
  I = imread(img_path);
  I = single(rgb2gray(I));
  [f, d] = vl_sift(I);
  nf = length(f);
  num_Feature(i) = nf;
  Feature(:,1:nf,i) = f;
  Descriptor(:,1:nf,i) = d;
end

max_number_of_features = max(num_Feature);
Feature = Feature(:,1:max_number_of_features,:);
Descriptor = Descriptor(:,1:max_number_of_features,:);

% Display Feature Extraction
DisplayFeatureExtraction(datapath, 8, 9, Feature, num_Feature);

for i = 1:number_of_pictures
  for j = 1:number_of_pictures
    nfa = num_Feature(i);
    nfb = num_Feature(j);
    da = Descriptor(:,1:nfa,i);
    db = Descriptor(:,1:nfb,j);
    [matches, scores] = vl_ubcmatch(da, db);
    if i ~= j
      num_Match(i,j) = length(scores);
      Match(i,:,1:length(matches),j) = matches;
    end
  end
end

% Display Feature Matching
DisplayFeatureMatching(datapath, 8, 9, Feature, num_Match, Match);

%% Initialization step (for 2 views)
% Find the best pair of images. 
[max1, ref] = max(num_Match);
[~, pair] = max(max1);
ref = ref(pair);

Image_selected(ref) = 1;
Image_selected(pair) = 1;

% Give ref's R|T = I|0
R(:, :, ref) = eye(3);
T(:, ref) = zeros(3, 1);

% Estimate E using 8,7-point algorithm or calibrated 5-point algorithm and RANSAC
feature_ref = Feature(:,:,ref);
feature_pair = Feature(:,:,pair);
mf_ref_pair = Match(ref,:,:,pair);
mf_ref_pair = squeeze(mf_ref_pair);
nm_ref_pair = num_Match(ref, pair);
mf_ref_pair = mf_ref_pair(:,1:nm_ref_pair);
mp_ref_pair = zeros(4,nm_ref_pair);
mp_ref_pair(1:2,:) = feature_ref(1:2,mf_ref_pair(1,:));
mp_ref_pair(3:4,:) = feature_pair(1:2,mf_ref_pair(2,:));
[E, F] = EstEF(mp_ref_pair, number_of_iterations_for_5_point, K, threshold_of_distance);

% Decompose E into [Rp, Tp]
P = Ess2Cam(E);
x_ref = HomoCoord(mp_ref_pair(1:2,:),'2D');
x_pair = HomoCoord(mp_ref_pair(3:4,:),'2D');
nx_ref = Normalize(x_ref,K);
nx_pair = Normalize(x_pair,K);
nx_ref = HomoCoord(nx_ref,'2D');
nx_pair = HomoCoord(nx_pair,'2D');
max_np = 0;
P_init = eye(3,4);
X3D_init = zeros(4,nm_ref_pair);
D_init = zeros(1,nm_ref_pair);
for i = 1:4
  P1 = eye(3,4);
  P2 = P(:,:,i);
  X3D = Triangulation(nx_ref, nx_pair, P1, P2);
  X3D = HomoCoord(X3D, '3D');
  D = DepthOfPoints(X3D,P2);
  np = sum(D>0);
  if np > max_np
    max_np = np;
    P_init = P2;
    X3D_init = X3D;
    D_init = D;
  end
end

c = 1;
for i = 1:nm_ref_pair
  if D_init(i) > 0
    X(:,c) = [X3D_init(1:3,i); 0; 0; 0];
    c = c + 1;
  end
end
X_init = X(:,1:c);

% (Optional) Optimize Rp and Tp
Rp = P_init(:,1:3);
Tp = P_init(:,4);

% Give pair's R|T = Rp|Tp
R(:, :, pair) = Rp;
T(:, pair) = Tp;

% Save 3D points to PLY
filename = sprintf('../report/view3d/given_data/init_views.ply');
SavePLY(filename, X_init);

% Display Initialization View
figure('Name', 'Initialization View')
ptcloud = pcread(filename);
pcshow(ptcloud);
xlim([-10, 20]);
ylim([-10, 20]);
zlim([0, 20]);

%% Growing step (for more than 2 views)
% If you have done the initialization step, then do this step.
X3D = X3D_init;
X_old = X_init;
P_old = P_init;
while(sum(Image_selected)<number_of_pictures)
  total_match = zeros(1,number_of_pictures);
  selected = find(Image_selected==1); % previously selected images
  for picture = 1 : number_of_pictures
    % Find the best image that has the largest sum of # correspondences to previously selected images
    if picture ~= selected
      total_match(picture) = num_Match(picture,ref)+num_Match(picture,pair);
    end
  end
  [~, new] = max(total_match);
  Image_selected(new) = 1;
  
  % Find the 2D-to-3D correspondences between features of 'new' and features of 'old's.
  mf_ref_pair = Match(ref,:,:,pair);
  mf_ref_pair = squeeze(mf_ref_pair);
  nm_ref_pair = num_Match(ref, pair);
  mf_ref_pair = mf_ref_pair(:,1:nm_ref_pair);

  mf_pair_new = Match(pair,:,:,new);
  mf_pair_new = squeeze(mf_pair_new);
  nm_pair_new = num_Match(pair,new);
  mf_pair_new = mf_pair_new(:,1:nm_pair_new);
  
  k = 1;
  mf_3views_idx = zeros(1,max_number_of_points);
  x_n = zeros(2,max_number_of_points);
  for i = 1:nm_ref_pair
    for j = 1:nm_pair_new
      if mf_ref_pair(2,i) == mf_pair_new(1,j)
        mf_3views_idx(1,k) = i;
        x_n(:,k) = Feature(1:2,mf_pair_new(2,j),new);
        k = k + 1;
      end
    end
  end
  nm_3views = k - 1;
  mf_3views_idx = mf_3views_idx(:,1:nm_3views);
  x_n = x_n(:,1:nm_3views);
  
  % Estimate pose R|T using 6-point DLT or 3-point algorithm.
  x_new = HomoCoord(x_n, '2D');
  x_pair = x_pair(:,mf_3views_idx);
  X3D_match = X3D(:,mf_3views_idx);
  data = zeros(3,6);
  max_cinl = 0;
  P = zeros(3,4);
  for i = 1:number_of_iterations_for_3_point
    three_points = randi(nm_3views,1,3);
    nx = x_new(:,three_points);
    nx = Normalize(nx, K);
    nx = HomoCoord(nx, '2D');
    Xw = X3D_match(1:3,three_points);
    data(1,:) = [nx(:,1)', Xw(:,1)'];
    data(2,:) = [nx(:,2)', Xw(:,2)'];
    data(3,:) = [nx(:,3)', Xw(:,3)'];
    Pn = PerspectiveThreePoint(data);
    [m,~] = size(Pn);
    for j = 0:(m/4-1)
      id = 4*j+1;
      P_new = Pn(id:(id+2),:);
      d1 = K*P_old*X3D_match;
      d2 = K*P_new*X3D_match;
      d1 = HomoCoord(d1, '2D');
      d2 = HomoCoord(d2, '2D');
      d1 = (x_pair-d1).^2;
      d2 = (x_new-d2).^2;
      d1 = sum(d1);
      d2 = sum(d2);
      d = d1 + d2;
      d = sqrt(d);
      cinl = d < threshold_of_distance;
      cinl = sum(cinl);
      if cinl > max_cinl
        max_cinl = cinl;
        P = P_new;
      end
    end
  end
    
  % Optimize R|T
  R(:,:,new) = P(:,1:3);
  T(:,new) = P(:,4);
    
  % Reconstruct 3D points using triangulation
  mp_pair_new = zeros(4,nm_pair_new);
  feature_new = Feature(:,:,new);
  feature_pair = Feature(:,:,pair);
  mp_pair_new(1:2,:) = feature_pair(1:2,mf_pair_new(1,:));
  mp_pair_new(3:4,:) = feature_new(1:2,mf_pair_new(2,:));
  x_pair = HomoCoord(mp_pair_new(1:2,:),'2D');
  x_new = HomoCoord(mp_pair_new(3:4,:),'2D');
  nx_pair = Normalize(x_pair,K);
  nx_new = Normalize(x_new,K);
  nx_pair = HomoCoord(nx_pair,'2D');
  nx_new = HomoCoord(nx_new,'2D');
  X3D = Triangulation(nx_pair, nx_new, P_old, P);
  
  ref = pair;
  pair = new;
  x_ref = x_pair;
  x_pair = x_new;
  P_old = P;
  
  % Save 3D points to PLY
  X3D = HomoCoord(X3D, '3D');
  D = DepthOfPoints(X3D, P);
  c = 1;
  for i = 1:nm_pair_new
    if D(i)>0
      X(:,c) = [X3D(1:3,i); 0; 0; 0];
      c = c + 1;
    end
  end
  Xv = X(:,1:c);
  Xsave = [X_old, Xv];
  X_old = Xsave;
  
  filename = sprintf('../report/view3d/given_data/%02dviews.ply', new);
  SavePLY(filename, Xsave);
end

filename = sprintf('../report/view3d/given_data/final_views.ply');
SavePLY(filename, Xsave);

% Display Final Views
figure('Name', 'Final View')
ptcloud = pcread(filename);
pcshow(ptcloud);
xlim([-10, 20]);
ylim([-10, 20]);
zlim([0, 20]);

