function objHypotheses = getObjectPose(scenePath,sceneData,scenePointCloud,backgroundPointCloud,extBin2Bg,objName,objModel,objNum, Mask)
% Take in scene RGB-D data and information about a target object, and
% predict the 6D pose for that object.
%
% function objHypotheses = getObjectPose(scenePath,sceneData,scenePointCloud,backgroundPointCloud,extBin2Bg,objName,objModel,objNum)
% Input:
%   scenePath            - file path to the folder holding RGB-D data of
%                          the scene
%   sceneData            - data structure holding the contents (frames and 
%                          camera information) of a captured scene (has N
%                          RGB-D frames)
%   scenePointCloud      - data structure holding the point cloud of the scene
%   backgroundPointCloud - data structure holding the background point cloud
%   extBin2Bg            - 4x4 rigid transformation matrix (in homogeneous
%                          coordinates) aligning the background point cloud
%                          to the scene point cloud
%   objName              - name of the target object (aka. object ID)
%   objModel             - data structure holding the point cloud of the
%                          pre-scanned object model
%   objNum               - number of instances of the target object in the
%                          scene
% Output:
%   objHypotheses        - ROS message with predicted 6D object pose (and
%                          other information about the surface point cloud)
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

global visPath;
global savePointCloudVis; 
global saveResultImageVis;
global emptyShelfModels;
global emptyToteModel;
global useGPU;

% Parameters for both shelf and tote scenario
gridStep = 0.002; % grid size for downsampling point clouds
icpWorstRejRatio = 0.9; % ratio of outlier points to ignore during ICP
objHypotheses = [];


% Parameters specific to shelf or tote scenario
if strcmp(sceneData.env,'tote')
%  frames = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18];
%   frames = [1 3 5 7 9 10 12 14 16 18];
%   frames = [5 14];
  viewBounds = [-0.3, 0.3; -0.4, 0.4; -0.05, 0.2];
  pushBackAxis = [0; 0; -1];
else
%  frames = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15];
%   frames = [1 5 8 11 15];
%   frames = 8;
  viewBounds = [-0.01, 0.40; -0.17, 0.17; -0.06, 0.20];
  pushBackAxis = [1; 0; -1];
end



% Parse segmentation masks and save confidence threshold used
% [objMasks,segmThresh,segmConfMaps] = getObjectMasks(scenePath,objName,frames);
% dlmwrite(fullfile(scenePath,'masks',sprintf('%s.thresh.txt',objName)),segmThresh);

% Load segmentation confidence maps
objMasks = {};
segmConfMaps = {};
segSum = zeros(480,640);

segConf = double(Mask)./255;
size(segConf(segConf>0.5))
segSum = segSum + segConf;
segmConfMaps{1} = segConf;

% Compute segmentation threshold
segmThresh = 0.15; % manual set threshold
segConf = segmConfMaps{1};
objMasks{1} = segConf > segmThresh;


% Create segmented point cloud of object
frames = 1; %now only 1 viewpoints

%%% pts in bin coordinate %%%
[objSegmPts,objSegmConf] = getSegmentedPointCloud(sceneData,frames,objMasks,segmConfMaps);

% If no segmentation, return dummy pose
if size(objSegmPts,2) < 200
  fprintf('    [Pose Estimation] %s: 0.000000\n',objName);
  for instanceIdx = 1:objNum
    currObjHypothesis = getEmptyObjectHypothesis(scenePath,objName,instanceIdx);
    objHypotheses = [objHypotheses,currObjHypothesis];
  end
  return;
end

% Handle objects without depth
if strcmp(objName,'dasani_water_bottle') && strcmp(sceneData.env,'shelf') || ...
   strcmp(objName,'rolodex_jumbo_pencil_cup') || ...
   strcmp(objName,'platinum_pets_dog_bowl')
  [predObjPoseBin,surfCentroid,surfRangeWorld] = getObjectPoseNoDepth(visPath,objSegmPts,objName,frames,sceneData,objMasks);
  predObjPoseWorld = sceneData.extBin2World * predObjPoseBin;
  predObjConfScore = mean(objSegmConf);
  for instanceIdx = 1:objNum
    if savePointCloudVis
      surfaceAxisPtsX = [surfRangeWorld(1,1):0.001:surfRangeWorld(1,2)];
      surfaceAxisPtsX = [surfaceAxisPtsX;repmat(surfCentroid(2),1,size(surfaceAxisPtsX,2));repmat(surfCentroid(3),1,size(surfaceAxisPtsX,2))];
      surfaceAxisPtsY = [surfRangeWorld(2,1):0.001:surfRangeWorld(2,2)];
      surfaceAxisPtsY = [repmat(surfCentroid(1),1,size(surfaceAxisPtsY,2));surfaceAxisPtsY;repmat(surfCentroid(3),1,size(surfaceAxisPtsY,2))];
      surfaceAxisPtsZ = [surfRangeWorld(3,1):0.001:surfRangeWorld(3,2)];
      surfaceAxisPtsZ = [repmat(surfCentroid(1),1,size(surfaceAxisPtsZ,2));repmat(surfCentroid(2),1,size(surfaceAxisPtsZ,2));surfaceAxisPtsZ];
      pcwrite(pointCloud([surfaceAxisPtsX,surfaceAxisPtsY,surfaceAxisPtsZ]','Color',[repmat([255,0,0],size(surfaceAxisPtsX,2),1);repmat([0,255,0],size(surfaceAxisPtsY,2),1);repmat([0,0,255],size(surfaceAxisPtsZ,2),1)]),fullfile(visPath,sprintf('vis.objSurf.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
    end
    
    %%% world cordiniate centroid %%%
    surfCentroidWorld = sceneData.extBin2World(1:3,1:3) * surfCentroid + repmat(sceneData.extBin2World(1:3,4),1,size(surfCentroid,2));
    surfRangeWorld = sceneData.extBin2World(1:3,1:3) * surfRangeWorld + repmat(sceneData.extBin2World(1:3,4),1,size(surfRangeWorld,2));
    if saveResultImageVis
      visualizeResults(predObjPoseWorld,[0 0 0],surfCentroidWorld,surfRangeWorld,predObjPoseWorld,predObjConfScore,scenePath,objName,instanceIdx,sceneData,[],objMasks,objModel.Location');
    end
    currObjHypothesis = getObjectHypothesis(predObjPoseWorld,[0 0 0],surfCentroidWorld,surfRangeWorld,predObjPoseWorld,predObjConfScore,scenePath,objName,instanceIdx);
    objHypotheses = [objHypotheses,currObjHypothesis];
  end
  return;
end

% Do 3D background subtraction
% pcwrite(pointCloud(objSegmPts'),fullfile(visPath,sprintf('vis.seg.%s',objName)),'PLYFormat','binary');
% pcwrite(backgroundPointCloud,fullfile(visPath,sprintf('vis.bg.%s',objName)),'PLYFormat','binary');


if useGPU
    [indicesNN,distsNN] = multiQueryKNNSearchImplGPU(backgroundPointCloud,objSegmPts');
else
    [indicesNN,distsNN] = multiQueryKNNSearchImpl(backgroundPointCloud,objSegmPts',1);
end

objSegmPts(:,find(sqrt(distsNN) < 0.005)) = []; %0.005
objSegmConf(:,find(sqrt(distsNN) < 0.005)) = [];


if strcmp(sceneData.env,'shelf')
  objSegmPtsBg = extBin2Bg(1:3,1:3) * objSegmPts + repmat(extBin2Bg(1:3,4) + [0;0;0.01],1,size(objSegmPts,2));
  bgRot = vrrotvec2mat([0 1 0 -atan(0.025/0.20)]);
  objSegmPtsBgRot = bgRot(1:3,1:3) * objSegmPtsBg;
%   pcwrite(pointCloud(objSegmPtsBgRot'),fullfile(visPath,sprintf('vis.objTestRot.%s',objName)),'PLYFormat','binary');
%   pcwrite(pointCloud(objSegmPts'),fullfile(visPath,sprintf('vis.objTestPre.%s',objName)),'PLYFormat','binary');
  objSegmPts(:,find(objSegmPtsBg(3,:) < -0.025 | objSegmPtsBgRot(3,:) < 0)) = [];
%   pcwrite(pointCloud(objSegmPts'),fullfile(visPath,sprintf('vis.objTestPost.%s',objName)),'PLYFormat','binary');
end

% Remove points outside the bin/tote
%{
ptsOutsideBounds = find((objSegmPts(1,:) < viewBounds(1,1)) | (objSegmPts(1,:) > viewBounds(1,2)) | ...
                        (objSegmPts(2,:) < viewBounds(2,1)) | (objSegmPts(2,:) > viewBounds(2,2)) | ...
                        (objSegmPts(3,:) < viewBounds(3,1)) | (objSegmPts(3,:) > viewBounds(3,2)));
objSegmPts(:,ptsOutsideBounds) = [];
objSegmConf(:,ptsOutsideBounds) = [];
%}

% % Grab center of segmented points and remove outliers
% obsCenter = median(sortrows(objSegPts',3)',2);
% eucDistGuess = sqrt(sum((objSegPts'-repmat(obsCenter',size(objSegPts,2),1)).^2,2));
% [sortDist, sortIdx] = sort(eucDistGuess);
% inlierPts = objCamPts(:,sortIdx(1:ceil((size(eucDistGuess,1) * icpWorstRejRatio))));

  
% Do k-means clustering to find centroids per object instance (duplicates)
instancePts = {};
instanceConf = {};
[kIdx, kCtr] = kmeans(objSegmPts(1:2,:)',objNum);
for instanceIdx = 1:objNum
  instanceInd = find(kIdx==instanceIdx);
  instancePts{instanceIdx} = objSegmPts(:,instanceInd);
  instanceConf{instanceIdx} = objSegmConf(instanceInd);
end

% Load object model and downsample it
objModelPts = objModel.Location;
objModelCloud = pointCloud(objModelPts);
objModelCloud = pcdownsample(objModelCloud,'gridAverage',gridStep);

% Loop through each instance of the object and estimate its pose
for instanceIdx = 1:objNum
  
  % Load object model
  objModelPts = objModelCloud.Location';

  % Remove outliers from the segmented point cloud using PCA
  
  %%% bin coridinate seg pts %%%
  currObjSegmPts = instancePts{instanceIdx};   
  if ~strcmp(objName,'barkely_hide_bones')
    if 0 %saveResultImageVis
      pcwrite(pointCloud(currObjSegmPts'),fullfile(visPath,sprintf('vis.objObs.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
    end
    try
      currObjSegmPts = denoisePointCloud(currObjSegmPts);
    end
    if 0 %saveResultImageVis
      pcwrite(pointCloud(currObjSegmPts'),fullfile(visPath,sprintf('vis.objCut.%s.%d',objName,instanceIdx)),'PLYFormat','binary');     
    end
  end
  fullCurrObjSegmPts = currObjSegmPts;
  
  % Print segmentation confidence
  predObjConfScore = mean(instanceConf{instanceIdx});
  if strcmp(objName,'dasani_water_bottle') && strcmp(sceneData.env,'tote')
%     predObjConfScore = predObjConfScore/4;
  end
  if size(currObjSegmPts,2) < 100 % If object not found
    fprintf('    [Pose Estimation] %s: 0.000000\n',objName);
    currObjHypothesis = getEmptyObjectHypothesis(scenePath,objName,instanceIdx);
    objHypotheses = [objHypotheses,currObjHypothesis];
    continue;
  else
    fprintf('    [Pose Estimation] %s: %f\n',objName,predObjConfScore);
  end
    
  % Downsample segmented point cloud to same resolution as object model
  currObjSegCloud = pointCloud(currObjSegmPts');
  currObjSegCloud = pcdownsample(currObjSegCloud,'gridAverage',gridStep);
  currObjSegCloud = pcdenoise(currObjSegCloud,'NumNeighbors',4);
  
  if size(currObjSegCloud.Location',2) < 200 % If object not found
    currObjHypothesis = getEmptyObjectHypothesis(scenePath,objName,instanceIdx);
    objHypotheses = [objHypotheses currObjHypothesis];
    continue;
  end
  
  % Compute surface mean 
  
  %%% object centroid  and bounding box in bin cooridinate %%%
  surfCentroid = mean(currObjSegmPts,2);
  currObjSegPtsRangeX = currObjSegmPts(:,find(((currObjSegmPts(2,:)>(surfCentroid(2)-0.005)) & (currObjSegmPts(2,:)<(surfCentroid(2)+0.005)) & ...
                                              (currObjSegmPts(3,:)>(surfCentroid(3)-0.005)) & (currObjSegmPts(3,:)<(surfCentroid(3)+0.005)))));
  currObjSegPtsRangeY = currObjSegmPts(:,find(((currObjSegmPts(1,:)>(surfCentroid(1)-0.005)) & (currObjSegmPts(1,:)<(surfCentroid(1)+0.005)) & ...
                                              (currObjSegmPts(3,:)>(surfCentroid(3)-0.005)) & (currObjSegmPts(3,:)<(surfCentroid(3)+0.005)))));
  currObjSegPtsRangeZ = currObjSegmPts(:,find(((currObjSegmPts(2,:)>(surfCentroid(2)-0.005)) & (currObjSegmPts(2,:)<(surfCentroid(2)+0.005)) & ...
                                              (currObjSegmPts(1,:)>(surfCentroid(1)-0.005)) & (currObjSegmPts(1,:)<(surfCentroid(1)+0.005)))));
  if isempty(currObjSegPtsRangeX)
    currObjSegPtsRangeX = currObjSegmPts;
  end
  if isempty(currObjSegPtsRangeY)
    currObjSegPtsRangeY = currObjSegmPts;
  end
  if isempty(currObjSegPtsRangeZ)
    currObjSegPtsRangeZ = currObjSegmPts;
  end
  surfRangeX = [min(currObjSegPtsRangeX(1,:)),max(currObjSegPtsRangeX(1,:))];
  surfRangeY = [min(currObjSegPtsRangeY(2,:)),max(currObjSegPtsRangeY(2,:))];
  surfRangeZ = [min(currObjSegPtsRangeZ(3,:)),max(currObjSegPtsRangeZ(3,:))];
  surfRangeWorld = [surfRangeX;surfRangeY;surfRangeZ];
                                            
  
  % Convert surface centroid to world coordinates
  
  %%% centroid in world cooridinate %%%
  surfCentroid = sceneData.extBin2World(1:3,1:3) * surfCentroid + repmat(sceneData.extBin2World(1:3,4),1,size(surfCentroid,2));
  surfRangeWorld = sceneData.extBin2World(1:3,1:3) * surfRangeWorld + repmat(sceneData.extBin2World(1:3,4),1,size(surfRangeWorld,2));
  % Recompute PCA pose over denoised and downsampled segmented point cloud
  currObjSegmPts = currObjSegCloud.Location';
  [coeffPCA,scorePCA,latentPCA] = pca(currObjSegmPts');
  if size(latentPCA,1) < 3
    latentPCA = [latentPCA;0];
  end
  coeffPCA = [coeffPCA(:,1),coeffPCA(:,2),cross(coeffPCA(:,1),coeffPCA(:,2))]; % Follow righthand rule
  surfPCAPoseBin = [[coeffPCA median(currObjSegmPts,2)]; 0 0 0 1];
   
  %%% pca pose in bin cooridinate %%%
  surfPCAPoseWorld = sceneData.extBin2World * surfPCAPoseBin;
  
  % If object is deformable, return PCA as pose
  if strcmp(objName,'cherokee_easy_tee_shirt') || strcmp(objName,'kyjen_squeakin_eggs_plush_puppies') || strcmp(objName,'womens_knit_gloves') || strcmp(objName,'cloud_b_plush_bear')
    predObjPoseWorld = surfPCAPoseWorld;
    if saveResultImageVis
      visualizeResults(surfPCAPoseWorld,latentPCA,surfCentroid,surfRangeWorld,predObjPoseWorld,predObjConfScore,scenePath,objName,instanceIdx,sceneData,fullCurrObjSegmPts,objMasks,objModel.Location');
    end
    currObjHypothesis = getObjectHypothesis(surfPCAPoseWorld,latentPCA,surfCentroid,surfRangeWorld,predObjPoseWorld,predObjConfScore,scenePath,objName,instanceIdx);
    objHypotheses = [objHypotheses,currObjHypothesis];
    continue;
  end

  % Push objects back prior to ICP
  pushBackVal = max([objModel.XLimits(2),objModel.YLimits(2),objModel.ZLimits(2)]);
  surfPCAPoseBin(1:3,4) = surfPCAPoseBin(1:3,4) + pushBackAxis.*pushBackVal;
  
  % Attempt 6 different rotations using PCA directions
%   possibleRotations = cell(1,6);
%   possibleRotations{1} = eye(3);
%   possibleRotations{2} = vrrotvec2mat([1,0,0,pi/2]);
%   possibleRotations{3} = vrrotvec2mat([1,0,0,pi]);
%   possibleRotations{4} = vrrotvec2mat([1,0,0,3*pi/2]);
%   possibleRotations{5} = vrrotvec2mat([0 1 0 pi/2]);
%   possibleRotations{6} = vrrotvec2mat([0,1,0,3*pi/2]);
  bestRotScore = inf;
  bestRt = surfPCAPoseBin;
%   for rotIdx = 1:length(possibleRotations)
%     tmpRt = initRtPCA * [possibleRotations{rotIdx} [0;0;0]; [0,0,0,1]];
%     tmpObjModelPts = tmpRt(1:3,1:3) * objModelPts + repmat(tmpRt(1:3,4),1,size(objModelPts,2));
%     tmpObjModelCloud = pointCloud(tmpObjModelPts');
% %     tmpObjSegCloud = pointCloud(currObjSegPts');
% %     [tform,movingReg,rmse] = pcregrigid(tmpObjSegCloud,tmpObjModelCloud,'InlierRatio',0.8,'MaxIterations',1);
%     [tmpIndices,tmpDists1] = multiQueryKNNSearchImplGPU(tmpObjModelCloud,currObjSegPts');
%     tmpObjSegCloud = pointCloud(currObjSegPts');
%     [tmpIndices,tmpDists2] = multiQueryKNNSearchImplGPU(tmpObjSegCloud,tmpObjModelPts');
%     tmpScore = mean([tmpDists1,tmpDists2]);
%     if tmpScore < bestRotScore
%       bestRotScore = tmpScore;
%       bestRt = tmpRt;
%     end
%   end
%   initRtPCA = bestRt;
  if strcmp(sceneData.env,'shelf') && (strcmp(objName,'creativity_chenille_stems') || strcmp(objName,'dr_browns_bottle_brush') || strcmp(objName,'peva_shower_curtain_liner') || strcmp(objName,'kleenex_paper_towels'))
    surfPCAPoseBin(1:3,1:3) = eye(3);
  end
  
  %surfPCAPoseBin = eye(4);
  
  % Apply rigid transform computed prior to ICP
  
  %%% object model pts in bin cooridinate %%%
  tmpObjModelPts = surfPCAPoseBin(1:3,1:3) * objModelPts + repmat(surfPCAPoseBin(1:3,4),1,size(objModelPts,2));
  
  if 0 %savePointCloudVis
    pcwrite(pointCloud(tmpObjModelPts','Color',repmat(uint8([0 0 255]),size(tmpObjModelPts,2),1)),fullfile(visPath,sprintf('vis.objPre.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
  end
  
  % Perform ICP to align segmented point cloud to object model
  if size(currObjSegmPts,2) > 3
    tmpObjModelCloud = pointCloud(tmpObjModelPts');
    objSegCloud = pointCloud(currObjSegmPts');
    if useGPU
        [tform,movingReg,icpRmse] = pcregrigidGPU(objSegCloud,tmpObjModelCloud,'InlierRatio',icpWorstRejRatio,'MaxIterations',200,'Tolerance',[0.0001 0.0009],'Verbose',false,'Extrapolate',true);
    else
        [tform,~,icpRmse] = pcregrigid(objSegCloud,tmpObjModelCloud,'InlierRatio',icpWorstRejRatio,'MaxIterations',200,'Tolerance',[0.0001 0.0009],'Verbose',false,'Extrapolate',true);
    end
    icpRt1 = inv(tform.T');
    %icpRt1 = eye(4);
    
    tmpObjModelPts = tmpObjModelCloud.Location';
    tmpObjModelPts = icpRt1(1:3,1:3) * tmpObjModelPts + repmat(icpRt1(1:3,4),1,size(tmpObjModelPts,2));
    tmpObjModelCloud = pointCloud(tmpObjModelPts');
    if useGPU
        [tform,movingReg,icpRmse] = pcregrigidGPU(objSegCloud,tmpObjModelCloud,'InlierRatio',icpWorstRejRatio/2,'MaxIterations',200,'Tolerance',[0.0001 0.0009],'Verbose',false,'Extrapolate',true); %'Tolerance',[0.0001 0.0009] 'MaxIterations',200 icpWorstRejRatio/2
    else
        [tform,movingReg,icpRmse] = pcregrigid(objSegCloud,tmpObjModelCloud,'InlierRatio',icpWorstRejRatio/2,'MaxIterations',200,'Tolerance',[0.0001 0.0009],'Verbose',false,'Extrapolate',true);
    end
    icpRt2 = inv(tform.T');
    icpRt = icpRt2*icpRt1;
  else
    icpRt = eye(4);
  end
  
  %%% object pose final in bin cooridinate %%%
  predObjPoseBin = icpRt* surfPCAPoseBin;
  

  %% send ros mark to visualize object pose in rviz

%{  
% specify data to publish
markerPub = rospublisher('/visualization_marker','visualization_msgs/Marker');
marker = rosmessage(markerPub);
   
%- set the frame information and names of marker
marker.Header.FrameId = '/camera_rgb_optical_frame';
marker.Ns = 'basic_shapes';
marker.Text = 'cube';
   
%- set the time
marker.Header.Stamp = rostime('now','system');
   
%- set the ID of the shape
marker.Id = 0;
   
%- set the scale of the shape
marker.Scale.X = 1;
marker.Scale.Y = 1;
marker.Scale.Z = 1;
   
%- set position and orientation
Pos = rosmessage('geometry_msgs/Point');
Pos.X = 0;
Pos.Y = 0;
Pos.Z = 0;                
Ori = rosmessage('geometry_msgs/Quaternion');
Ori.W = 1;
marker.Pose.Position = Pos; 
marker.Pose.Orientation = Ori;
   
%- set color
Color = rosmessage('std_msgs/ColorRGBA');
Color.R = 1;
Color.G = 0.5;
Color.B = 0;
Color.A = 1;
marker.Color = Color;
   
%- set the type of the shape
type = [1 2 3]; % 1: cube, 2: sphere, 3: cylinder
   
marker

while (1)
  for i=1:3
      %- set the time
      marker.Header.Stamp = rostime('now','system');
      marker.Type = type(i); 
      send(markerPub,marker);
      pause(1);
  end            
end
%}
  
  %%
  if 0 %savePointCloudVis
    tmpObjModelPts = predObjPoseBin(1:3,1:3) * objModelPts + repmat(predObjPoseBin(1:3,4),1,size(objModelPts,2));
    pcwrite(pointCloud(tmpObjModelPts','Color',repmat(uint8([0 255 0]),size(tmpObjModelPts,2),1)),fullfile(visPath,sprintf('vis.objPost.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
  end

  % Save pose as a rigid transform from model to world space
  
  
  %%% pose in world cooridinate %%%
  predObjPoseWorld = sceneData.extBin2World * predObjPoseBin;
  
  if 0 %saveResultImageVis
    visualizeResults(surfPCAPoseWorld,latentPCA,surfCentroid,surfRangeWorld,predObjPoseWorld,predObjConfScore,scenePath,objName,instanceIdx,sceneData,fullCurrObjSegmPts,objMasks,objModel.Location');
  end
  
  sceneData.extCam2World{1}
  sceneData.extWorld2Bin(1)
    
  currObjHypothesis = getObjectHypothesis(surfPCAPoseWorld,latentPCA,surfCentroid,surfRangeWorld,predObjPoseWorld,predObjConfScore,scenePath,objName,instanceIdx,sceneData.extCam2World{1},sceneData.extWorld2Bin(1));
  objHypotheses = [objHypotheses,currObjHypothesis];
end

end

