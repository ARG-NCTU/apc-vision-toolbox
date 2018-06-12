
%% real time mask prediction callback

%%

function sceneData = msgcallback(subscriber, msg)

global visPath;
global savePointCloudVis; 
global saveResultImageVis;
global objNames
global objModels
global emptyShelfModels;
global emptyToteModel;
global useGPU;


choosen_obj = {'kleenex_paper_towels'};
object = msg.Object;
Mask = readImage(msg.Probmask);


%% only do pose estimation for selected class
if strcmp(choosen_obj, object) == 1
    
sub_depth = rossubscriber('/camera/depth_registered/sw_registered/image_rect');
depth_msg = receive(sub_depth,2);
depth = readImage(depth_msg);


sub_rgb = rossubscriber('/camera/rgb/image_rect_color');
rgb_msg = receive(sub_rgb,2);
rgb = readImage(rgb_msg);

sub_raw_depth =  rossubscriber('/camera/depth/image_raw');
raw_depth_msg = receive(sub_raw_depth,2);
raw_depth = readImage(raw_depth_msg);

% Load RGB-D frames for scene
scenePath = '/home/joinet/text-pick-n-place-baseline/data/sample/scene-0000'

sceneData = loadScene(scenePath, rgb, depth, raw_depth);
DoCalib = 0;

if DoCalib == 1
    sceneData = loadCalib(reqMsg.CalibrationFiles,sceneData);
end

Original_point_in_bin = [0,0,0];
Original_point_in_World = sceneData.extBin2World(1:3,1:3) * Original_point_in_bin' + repmat(sceneData.extBin2World(1:3,4),1,size(Original_point_in_bin',2))
World2Cam = inv(sceneData.extCam2World{1})
size(Original_point_in_World)
Original_point_in_Camera = World2Cam(1:3,1:3) * Original_point_in_World + repmat(World2Cam(1:3,4),1,size(Original_point_in_World,2))


 % Fill holes in depth frames for scene
for frameIdx = 1:length(sceneData.depthFrames)
    sceneData.depthFrames{frameIdx} = fillHoles(sceneData.depthFrames{frameIdx});
    
end

% Load scene point cloud
fprintf('    [Processing] Loading scene point clouds\n');
scenePointCloud = getScenePointCloud(sceneData);


% Load and align empty bin/tote point cloud to observation
binIds = 'ABCDEFGHIJKL';

if 0 %strcmp(sceneData.env,'shelf')
    backgroundPointCloud = emptyShelfModels{strfind(binIds,sceneData.binId)};
else
    backgroundPointCloud = emptyToteModel;
end

if 1 %useGPU
    [tform,backgroundPointCloud] = pcregrigidGPU(backgroundPointCloud,scenePointCloud,'InlierRatio',0.8,'MaxIterations',200,'Tolerance',[0.0001,0.0009],'Verbose',false,'Extrapolate',true);
else
    [tform,backgroundPointCloud] = pcregrigid(backgroundPointCloud,scenePointCloud,'InlierRatio',0.8,'MaxIterations',200,'Tolerance',[0.0001,0.0009],'Verbose',false,'Extrapolate',true);
end
extBin2Bg = inv(tform.T');


% Parse object names
objList = choosen_obj

for objIdx = 1:length(objList)
%     tic;
    objName = objList{objIdx};
    objNum = sum(ismember(objList,objName));
    objModel = objModels{find(ismember(objNames,objName))};
    if isempty(objModel)
        fprintf('Error: object model not loaded for %s!\n',objName);
    end

    % Do 6D pose estimation for each object and save the results
    objPoses = getObjectPose(scenePath,sceneData,scenePointCloud,backgroundPointCloud,extBin2Bg,objName,objModel,objNum, Mask);
    showdetails(objPoses)
    for duplicateIdx = 1:length(objPoses)
        %respMsg.Objects = [respMsg.Objects; objPoses(duplicateIdx)];
        
    end
%     toc;
end


fprintf('    [Visualization] Drawing predicted object poses\n');


%% not using matlab visualize
%%
% Load results (predicted 6D object poses)

% Get set of colors for visualizations (from vis-utils)
load('colorPalette.mat');
modelsPath = './models/objects'; % Directory holding pre-scanned object models% Get set of colors for visualizations (from vis-utils)


tmpDataPath = '/home/joinet/text-pick-n-place-baseline/data/sample/scene-0000/';

resultFiles = dir(fullfile(tmpDataPath,'results/*.result.txt'));
results = cell(1,length(resultFiles));
confScores = [];
for resultIdx = 1:length(resultFiles)
    tmpResultFile = resultFiles(resultIdx).name;
    tmpResultFilenameDotIdx = strfind(tmpResultFile,'.');
    tmpResult.objName = tmpResultFile(1:(tmpResultFilenameDotIdx(1)-1));
    tmpResult.objNum = str2double(tmpResultFile((tmpResultFilenameDotIdx(1)+1):(tmpResultFilenameDotIdx(2)-1)));
    tmpResult.objPoseWorld = eye(4);
    tmpResult.objPoseWorld(1:3,4) = dlmread(fullfile(tmpDataPath,'results',tmpResultFile),'\t',[1,0,1,2])';
    objPoseRotQuat = dlmread(fullfile(tmpDataPath,'results',tmpResultFile),'\t',[4,0,4,3]);
    tmpResult.objPoseWorld(1:3,1:3) = quat2rotm([objPoseRotQuat(4),objPoseRotQuat(1:3)]);
    tmpResult.confScore = dlmread(fullfile(tmpDataPath,'results',tmpResultFile),'\t',[28,0,28,0]);
    confScores = [confScores;tmpResult.confScore];
    results{resultIdx} = tmpResult;
end

% Sort results by confidence scores
[~,sortIdx] = sortrows(confScores,1);

% Get random colors per object in the scene
randColorIdx = randperm(length(colorPalette),length(results));

% Create canvas for visualization
canvas = sceneData.colorFrames{1};
canvasSeg = uint8(ones(size(canvas))*255);
canvasPose = canvas;
canvasPose = insertText(canvasPose,[10 10],'Confidence  :  Object','Font','LucidaSansDemiBold','FontSize',12,'TextColor','white','BoxColor','black');

% Loop through each object to save and visualize predicted object pose
uniqueObjList = unique(sceneData.objects);
textPosY = 32;
for resultIdx = 1:length(resultFiles)
    currResult = results{sortIdx(resultIdx)};
    objColor = colorPalette{randColorIdx(resultIdx)};

    % Load pre-scanned object point cloud
    objPointCloud = pcread(fullfile(modelsPath,sprintf('%s.ply',currResult.objName)));

    % Draw projected object model and bounding box
    [canvasSeg,canvasPose] = showObjectPose(currResult.objName, canvasSeg, canvasPose, sceneData.colorFrames{1}, sceneData.depthFrames{1}, sceneData.extCam2World{1}, sceneData.colorK, objPointCloud, currResult.objPoseWorld, objColor,sceneData.extBin2World);                             
    canvasPose = insertText(canvasPose,[10 textPosY],sprintf('  %f  :  %s',currResult.confScore,currResult.objName),'Font','LucidaSansDemiBold','FontSize',12,'TextColor',objColor,'BoxColor','black');
    textPosY = textPosY + 22;
end

imshow(canvasPose); title('Predicted 6D Object Poses');





   
    
end





end