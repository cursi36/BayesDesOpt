% Example to optimize for design of two dual arms with different
% structures
function BayesDesOpt_run
clear all;
close all;
warning('off','all')

addpath('../SmoothSurf')
addpath('../')
addpath("Data/")

%% Initializations
% configFile = "Data/optConf_dualArm.xml"; %DualArm opt
% configFile = "Data/optConf_singleArm.xml"; %SingleArm opt
configFile = "Data/optConf_dualArm_copy.xml"; %DualArm copy opt
[solver,Robot_infos,dual_arm_copy,JointType_optInfos,LinkLength_optInfos,Distance_optInfo] = readSolverConfig(configFile);


for i = 1:length(Robot_infos)
    DH_tabs{i} = Robot_infos{i}.DH_tab;
    joint_types{i} = Robot_infos{i}.joint_types;
    joint_limits{i} = Robot_infos{i}.joint_limits;
    T_inits{i} = Robot_infos{i}.T_init;
    
    if dual_arm_copy == true
        DH_tabs{i+1} = DH_tabs{i};
        joint_types{i+1} = Robot_infos{i}.joint_types;
        joint_limits{i+1} = Robot_infos{i}.joint_limits;
        T_inits{i+1} = Robot_infos{i+1}.T_init;
        break;
    end
    
end

Robots = InitializeRobots(DH_tabs,joint_types,joint_limits,T_inits,dual_arm_copy);

[optVars,Indexes] = setOptimizationVariablesBO(LinkLength_optInfos,JointType_optInfos,Distance_optInfo,dual_arm_copy);


%% START OPTIMIZATION
Accept_rate = 0.6;

for i = 1:length(Accept_rate)
    close all;
    
    accept_rate = Accept_rate(i);
    disp("*******")
    disp("accept rate "+num2str(accept_rate))
    
    Npnts_WS = 150*1e03; %number of points for WS computation
    %%minimizes function
    nvars = length(optVars);
    
    NumSeed = 1000;
    Res = bayesopt(@(x)costFunction(x,Robots,Indexes,accept_rate,Npnts_WS,false),...
        optVars,'Verbose',0,...
        'IsObjectiveDeterministic',true,'AcquisitionFunctionName','expected-improvement-per-second-plus',...
        'ExplorationRatio',0.6,...
        'MaxObjectiveEvaluations',1,'NumSeedPoints',NumSeed,'UseParallel',true);
    x_res = Res.XAtMinObjective;
    x = x_res{:,:};
    
    %plot results:
    Robots = generateRobots(x,Robots,Indexes);
    dual_arm_copy = Indexes{1}.dual_arm_copy;
    
    [dtsPs,Vs,SafetyMeasure] = getWSVolumes(Robots,dual_arm_copy,accept_rate,Npnts_WS,true);
    
    folder_base = "Results_Matlab_1/";
    folder = folder_base+"accept_"+num2str(accept_rate*100)+"/";
    mkdir (folder);
    
    
    saveas(figure(2),folder+"V_cloud.fig")
    saveas(figure(3),folder+"V_shp.fig")
    saveas(figure(4),folder+"V_patch.fig")
    save(folder+"ResBayesOpt","Res")
end

end

function [Cost] = costFunction(x,Robots,Indexes,acceptRate,Npnts_WS,plot_en)

x = x{:,:};
Cost = getCostFunction(x,Robots,Indexes,acceptRate,Npnts_WS,plot_en);

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




