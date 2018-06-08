% Starts Matlab ROS service pose_estimation
%
% Matlab custom ROS messages needs to be compiled before running this
% script (see make.m)
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

cd(fileparts(which('startService.m')));

% User configurations (change me)
rgbdUtilsPath = '/home/joinet/text-pick-n-place-baseline/rgbd-utils'; % Directory of RGB-D toolbox utilities
vis_utilpath = '/home/joinet/text-pick-n-place-baseline/vis-utils';
global visPath; visPath = fullfile(pwd,'visualizations'); % Directory to save visualization files
global savePointCloudVis; savePointCloudVis = true; % Option to save point cloud visualizations (can influence pose estimation speed)
global saveResultImageVis; saveResultImageVis = true;% Option to save pose estimation result visualizations (can influence pose estimation speed)
global useGPU; useGPU = true; % Option to use GPU (CUDA support only)

addpath(genpath(rgbdUtilsPath));
addpath(genpath(pwd));
addpath(genpath(vis_utilpath));
mkdir(visPath);


% Check installation of Robotics Addons for ROS Custom Messages
if ~exist('rosgenmsg')
    fprintf('Please download and install add-ons for Matlab Robotics System Toolbox then restart this script.\n')
    roboticsAddons
    return;
end
addpath('matlab_gen/msggen');

% Start Matlab ROS
try
    rosinit
catch
    rosshutdown
    rosinit
end

% Load pre-scanned object models
global objNames;
global objModels;
objNames = {'barkely_hide_bones', ...
            'cherokee_easy_tee_shirt', ...
            'clorox_utility_brush', ...
            'cloud_b_plush_bear', ...
            'cool_shot_glue_sticks', ...
            'command_hooks', ...
            'crayola_24_ct', ...
            'creativity_chenille_stems', ...
            'dasani_water_bottle', ...
            'dove_beauty_bar', ...
            'dr_browns_bottle_brush', ...
            'easter_turtle_sippy_cup', ...
            'elmers_washable_no_run_school_glue', ...
            'expo_dry_erase_board_eraser', ...
            'fiskars_scissors_red', ...
            'fitness_gear_3lb_dumbbell', ...
            'folgers_classic_roast_coffee', ...
            'hanes_tube_socks', ...
            'i_am_a_bunny_book', ...
            'jane_eyre_dvd', ...
            'kleenex_paper_towels', ...
            'kleenex_tissue_box', ...
            'kyjen_squeakin_eggs_plush_puppies', ...
            'laugh_out_loud_joke_book', ...
            'oral_b_toothbrush_green', ...
            'oral_b_toothbrush_red', ...
            'peva_shower_curtain_liner', ...
            'platinum_pets_dog_bowl', ...
            'rawlings_baseball', ...
            'rolodex_jumbo_pencil_cup', ...
            'safety_first_outlet_plugs', ...
            'scotch_bubble_mailer', ...
            'scotch_duct_tape', ...
            'soft_white_lightbulb', ...
            'staples_index_cards', ...
            'ticonderoga_12_pencils', ...
            'up_glucose_bottle', ...
            'woods_extension_cord', ...
            'womens_knit_gloves'};
objModels = cell(1,length(objNames));
fprintf('Loading pre-scanned object models...\n');
for objIdx = 1:length(objNames)
  try
    objModels{objIdx} = pcread(sprintf('models/objects/%s.ply',objNames{objIdx}));
    fprintf('    %s\n',objNames{objIdx});
  end
end

% Load pre-scanned empty tote and shelf bins
global emptyShelfModels;
global emptyToteModel;
emptyShelfModels = cell(1,12);
binIds = 'ABCDEFGHIJKL';
fprintf('Loading pre-scanned empty tote and shelf bins...\n');
for binIdx = 1:length(binIds)
    emptyShelfModels{binIdx} = pcread(sprintf('models/bins/bin%s.ply',binIds(binIdx)));
end
emptyToteModel = pcread('models/bins/tote.ply');

% Setup CUDA KNNSearch Kernel Function
if useGPU
    global KNNSearchGPU;
    tic;
    fprintf('Setting up CUDA kernel functions...\n');
    KNNSearchGPU = parallel.gpu.CUDAKernel('KNNSearch.ptx','KNNSearch.cu');
    toc;
end


% Start ROS service
%{
server = rossvcserver('/pose_estimation', 'pose_estimation/EstimateObjectPose', @serviceCallback);
fprintf('Ready.\n');
%}

% start listen prediction topic 
sub = rossubscriber('/mask_prediction_with_class',@msgcallback);
mask_msg = receive(sub,10);
fprintf('Ready.\n');
