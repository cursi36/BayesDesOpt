% Example to optimize for design of a single arm
function BayesDesOpt_runSingleRobot
clear all;
close all;
warning('off','all')

addpath('../SmoothSurf')
addpath('..')

%% Initializations
%Initilize robots
DH_tabs{1} = [
    1 pi/2 0 pi/2;
    0 pi/2 0 -pi/2;
    0 0 3 pi/2;
    0 0 0 -pi/2;
    0 0 3 0];
joint_types{1} = 'prrrr';
joint_limits{1} = [0 10;-45 45;-45 45;-45 45;-45 45];
T_inits{1} = eye(4);

dual_arm_copy = false;

Robots = InitializeRobots(DH_tabs,joint_types,joint_limits,T_inits,dual_arm_copy);

%% Define Optimization Variables
%% Set info of parameters to ptimize for each robot independently if dual_arm_copy = false;
%Set index of links to optimize ad their bounds
LinkLength_optInfos{1}.d_idx = [1 2 3 4 5]; %d of DH table
LinkLength_optInfos{1}.a_idx = [1 2 3 4 5]; % a of DH table
LinkLength_optInfos{1}.d_bounds{1} = [3 25]; %bounds for ds
LinkLength_optInfos{1}.d_bounds{2} = [3 25]; %bounds for ds
LinkLength_optInfos{1}.d_bounds{3} = [3 25]; %bounds for ds
LinkLength_optInfos{1}.d_bounds{4} = [3 25]; %bounds for ds
LinkLength_optInfos{1}.d_bounds{5} = [3 25]; %bounds for ds

LinkLength_optInfos{1}.a_bounds{1} = [3 25]; %bounds for as
LinkLength_optInfos{1}.a_bounds{2} = [3 25]; %bounds for as
LinkLength_optInfos{1}.a_bounds{3} = [3 25]; %bounds for as
LinkLength_optInfos{1}.a_bounds{4} = [3 25]; %bounds for as
LinkLength_optInfos{1}.a_bounds{5} = [3 25]; %bounds for as

%Set index of joint to optimize ad their bounds. The bounds can be 'p' or
%'r'.
JointType_optInfos{1}.idx = [2 5];
JointType_optInfos{1}.bounds{1} = {'p' 'r'};
JointType_optInfos{1}.bounds{2} = {'p' 'r'};

%joint limits in case th joint is prismatic
JointType_optInfos{1}.jointlimits.p{1} = [0 1];
JointType_optInfos{1}.jointlimits.p{2} = [0 1];

%joint limits in case th joint is revolute
JointType_optInfos{1}.jointlimits.r{1} = [0 180];
JointType_optInfos{1}.jointlimits.r{2} = [0 180];

JointType_optInfos{1}.thetas_idx = [3];
JointType_optInfos{1}.thetas_bounds{1} = [0 180];

JointType_optInfos{1}.alphas_idx = [];
JointType_optInfos{1}.alphas_bounds{1} = [];

%Set distance between arms to be optimized
Distance_optInfo.idx = []; %x,y,z

[optVars,Indexes] = setOptimizationVariables(LinkLength_optInfos,JointType_optInfos,Distance_optInfo,dual_arm_copy);

%% START OPTIMIZATION
Accept_rate = 0.7;

for i = 1:length(Accept_rate)
    close all;
    
    accept_rate = Accept_rate(i);
    disp("*******")
    disp("accept rate "+num2str(accept_rate))
    
    Npnts_WS = 150*1e03; %number of points for WS computation
    %%minimizes function
    NumSeed = 100;
    Res = bayesopt(@(x)getCostFunction(x,Robots,Indexes,accept_rate,Npnts_WS,false),...
        optVars,'Verbose',0,...
        'IsObjectiveDeterministic',true,'AcquisitionFunctionName','expected-improvement-per-second-plus',...
        'ExplorationRatio',0.6,...
        'MaxObjectiveEvaluations',100,'NumSeedPoints',NumSeed,'UseParallel',false);
    
    %plot results:
    folder_base = "Results_Matlab_1/";
    folder = folder_base+"accept_"+num2str(accept_rate*100)+"/";
    mkdir (folder);
    x = Res.XAtMinObjective;
    
    Robots = generateRobots(x,Robots,Indexes);
    dual_arm_copy = Indexes{1}.dual_arm_copy;
    
    [dtsPs,Vs,SafetyMeasure] = getWSVolumes(Robots,dual_arm_copy,accept_rate,Npnts_WS,true);
    
    
    saveas(figure(2),folder+"V_cloud.fig")
    saveas(figure(3),folder+"V_shp.fig")
    saveas(figure(4),folder+"V_patch.fig")
    save(folder+"ResBayesOpt","Res")
end

end

%Cost function to minimize
function Cost = getCostFunction(x,Robots,Indexes,acceptRate,Npnts_WS,plot_en)
Robots = generateRobots(x,Robots,Indexes);
dual_arm_copy = Indexes{1}.dual_arm_copy;

[~,~,Vs,Safety] = getWSVolumes(Robots,dual_arm_copy,acceptRate,Npnts_WS,plot_en);

if length(Vs) > 1
    V = Vs(3);
else
    V = Vs(1);
    
end

%with safety measure.
SafetyVolume = max(V*Safety,1e-16);
Cost = -log10(SafetyVolume);

% disp("| Volume | "+num2str(Vs(3))+...
%     " | Saefty | "+num2str(Safety));

end


function x = roundValues(x, Resolution)

% unit_scale = 1;
% for i = 1:4
%
%     div_rem = rem(x{1,i}/unit_scale*2,2);
%
%     if div_rem <= 1
%
%         if div_rem >= 0.5
%             x{1,i} = ceil(x{1,i}/unit_scale * 2) / 2;
%         else
%             x{1,i} = floor(x{1,i}/unit_scale * 2) / 2;
%         end
%     end
%
%     if div_rem > 1
%         if  div_rem >= 1.5
%             %round to x+0.5
%             x{1,i} = ceil(x{1,i}/unit_scale * 2) / 2;
%
%         elseif div_rem < 1.5
%             %round to x-0.5
%             x{1,i} = floor(x{1,i}/unit_scale * 2) / 2;
%
%         end
%     end
%
%     x{1,i} = x{1,i}*unit_scale;
% end
end




