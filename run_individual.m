close all;

%加载路径
addpath(genpath('FeatureExtractor'));
addpath(genpath('MotionModel'));
addpath(genpath('ObservationModel'));
addpath(genpath('sampler'));
addpath(genpath('UpdateJudger'));
addpath(genpath('Utils'));

%配置视频序列路径
vedios=configSeqs;
for index_vedio=1:length(vedios)
    vedio=vedios{index_vedio};
    dataPath = vedio.path;
    config;
    
    % Load data
    disp('Loading data...');
    %读入图像
    fullPath = [dataPath, 'img\'];
    d = dir([fullPath, '*.jpg']);
    if size(d, 1) == 0
        d = dir([fullPath, '*.png']);
    end
    if size(d, 1) == 0
        d = dir([fullPath, '*.bmp']);
    end
    %读入注释
      if strcmp(vedio.name,'Jogging') == 0
        rects = importdata([dataPath, '\groundtruth_rect.txt']);
     else
         rects = importdata([dataPath, '\groundtruth_rect.2.txt']);
     end
    %提取视频序列第一帧boundbox
    p = rects(1,:);
    seq.init_rect = [p(1), p(2), p(3), p(4), 0];
    %提取视频序列第一帧
    im = imread([fullPath, d(1).name]);
    data = zeros(size(im, 1), size(im, 2), size(d, 1));
    seq.s_frames = cell(size(d, 1), 1);
    for i = 1 : size(d, 1)
        seq.s_frames{i} = [fullPath, d(i).name];
    end
    %读入option信息
    seq.opt = opt;
    %结果resul.res为最终数据
    result = run_Diagnose(seq, '', false,vedio);
    results{1}=result;
    save(['.\results\',vedio.name '_BASE_C.mat'],'results') ;
end