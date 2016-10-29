function results = run_Diagnose(seq, res_path, bSaveImage,vedio)

%读入配置信息
configGlobalParam;
config;

pathDraw = '.\tmp\imgs\';

rng(0);
seq.opt = opt;
%第一帧目标位置
rect=seq.init_rect;
%p为目标中心位置x,y,与res的h,w
p = [rect(1)+rect(3)/2, rect(2)+rect(4)/2, rect(3), rect(4)]; % Center x, Center y, height, width
frame = imread(seq.s_frames{1});
%如果为灰度图像，则复制灰度维为RGB
if size(frame,3)==1
    frame = repmat(frame,[1,1,3]);
end
%将RGB格式转化为gray格式
frame = rgb2gray(frame);
%调整目标图片尺寸
if (seq.opt.useNormalSize)
    %计算高度变化系数
    scaleHeight = size(frame, 1) / seq.opt.normalHeight;
    %计算宽度变化系数
    scaleWidth = size(frame, 2) / seq.opt.normalWidth;
    p(1) = p(1) / scaleWidth;
    p(3) = p(3) / scaleWidth;
    p(2) = p(2) / scaleHeight;
    p(4) = p(4) / scaleHeight;
end

duration = 0;
tic;
reportRes = [];
for f = 1:size(seq.s_frames, 1)
    disp(f)
    frame = imread(seq.s_frames{f});
    if size(frame,3)==1
        frame = repmat(frame,[1,1,3]);
    end
    %如果分辨率过高resize图片
    if (seq.opt.useNormalSize)
        %           frame = imresize(frame, [seq.opt.normalHeight, seq.opt.normalWidth]);
        frame = mexResize(frame, [seq.opt.normalHeight, seq.opt.normalWidth], 'auto');
    end
    frame = im2double(frame);
    
    %如果非第一帧
    if (f ~= 1)
        %第3步
        %运动模型，提取出可能存在目标的候选区域
        tmpl    = globalParam.MotionModel(tmpl, prob, seq.opt);
        %特征提取，提取所有候选区域的特征
        [feat, seq.opt] = globalParam.FeatureExtractor(frame, tmpl, seq.opt);
        %观测模型，计算每一特征的概率
        prob    = globalParam.ObservationModelTest(feat, model);
        %筛选出概率最大位置，作为PF输出结果
        [maxProb, maxIdx] = max(prob);
        pf_p = tmpl(maxIdx, :);
        
        %选择PForNF
        if f>5
            if maxProb>0.8
                %if (norm(pf_v-nf_v,1)>20 || norm((sign(pf_a)-sign(nf_a)))~=0)
                if (norm(pf_v-nf_v,1)>30 )
                    p=nf_p
                    %                     maxProb=0.8;
                else
                    p=pf_p;
                end
            else
                p=nf_p;
            end
        else
            p=pf_p;
        end
        
        model.lastOutput = p;
        model.lastProb = maxProb;
        
        %计算PF的v与a
        pf_v=pf_p(1,1:2)-front_p(1,1:2);
        if f==2
            pf_a=0;
            init_pfa=pf_v;
        else
            pf_a=pf_v-init_pfa;
            init_pfa=pf_v;
        end
        
        %添加物理特征
        if(f==3)
            init_v=model.lastOutput(1,1:2);
            init_w=model.lastOutput(1,3);
            init_h=model.lastOutput(1,4);
        end
        if(f>3)
            nf_v=model.lastOutput(1,1:2)-init_v;
            init_v=model.lastOutput(1,1:2);
            nf_w=model.lastOutput(1,3)-init_w;
            init_w=model.lastOutput(1,3);
            nf_h=model.lastOutput(1,4)-init_h;
            init_h=model.lastOutput(1,4);
        end
        
        if(f==4)
            init_a=nf_v;
            init_aw=nf_w;
            init_ah=nf_h;
        end
        if(f>4)
            nf_a=nf_v-init_a;
            init_a=nf_v;
            nf_aw=nf_w-init_aw;
            init_aw=nf_w;
            nf_ah=nf_h-init_ah;
            init_ah=nf_h;
        end
        
        if f>5
            nf_p_v=front_p(1,1:2)+nf_v+nf_a;
            nf_p_w=front_p(1,3)+nf_w+nf_aw;
            nf_p_h=front_p(1,4)+nf_h+nf_ah;
            %此处需修改
            nf_p=[nf_p_v,front_p(1,3),front_p(1,4)];
        end
        
        maxProb
        %保存当前帧p
        front_p=p;
        
        if (strcmp(func2str(globalParam.ConfidenceJudger), 'UpdateDifferenceJudger'))
            w1 = max(round(tmpl(:, 1) - tmpl(:, 3) / 2), round(p(:, 1) - p(:, 3) / 2));
            w2 = min(round(tmpl(:, 1) + tmpl(:, 3) / 2), round(p(:, 1) + p(:, 3) / 2));
            h1 = max(round(tmpl(:, 2) - tmpl(:, 4) / 2), round(p(:, 2) - p(:, 4) / 2));
            h2 = min(round(tmpl(:, 2) + tmpl(:, 4) / 2), round(p(:, 2) + p(:, 4) / 2));
            interArea = max(w2 - w1, 0) .* max(h2 - h1, 0);
            jointArea = (round(tmpl(:, 3)) .* round(tmpl(:, 4)) + round(p(3)) * round(p(4))) - interArea;
            overlapRatio = interArea ./ jointArea;
            idx = (overlapRatio < seq.opt.UpdateDifferenceJudger.overlap);
            model.secondProb = max(prob(idx));
        end
    else
        %第1步
        %tmpl为400个后候选区域
        tmpl = globalParam.MotionModel(p, 1, seq.opt);
        %其概率值初始化均为1
        prob = ones(1, size(tmpl, 1));
        front_p=p;
    end
    
    %模型的建立与更新
    if (f == 1)
        %第2步
        %建立模型
        pathSave = [pathDraw vedio.name  '\'];
        %采样
        tmplPos = globalParam.PosSampler(p, seq.opt);
        tmplNeg = globalParam.NegSampler(p, seq.opt);
        %特征提取
        [dataPos, seq.opt] = globalParam.FeatureExtractor(frame, tmplPos, seq.opt);
        [dataNeg, seq.opt] = globalParam.FeatureExtractor(frame, tmplNeg, seq.opt);
        %建立模型
        model   = globalParam.ObservationModelTrain(dataPos, dataNeg, seq.opt);
        if (seq.opt.useFirstFrame)
            assert(~strcmp(func2str(globalParam.ObservationModelTrain), 'SOSVMTrain'), ...
                'SOSVM does not support useFirstFrame option!!');
            dataPosFirstFrame = dataPos;
        end
    else
        %第4步
        %模型更新
        if (globalParam.ConfidenceJudger(model, seq.opt))
            tmplPos = globalParam.PosSampler(p, seq.opt);
            tmplNeg = globalParam.NegSampler(p, seq.opt);
            [dataPos, seq.opt] = globalParam.FeatureExtractor(frame, tmplPos, seq.opt);
            [dataNeg, seq.opt] = globalParam.FeatureExtractor(frame, tmplNeg, seq.opt);
            %                 disp(f);
            if (seq.opt.useFirstFrame)
                dataPos.feat = [dataPosFirstFrame.feat, dataPos.feat];
                dataPos.tmpl = [zeros(size(dataPosFirstFrame.tmpl)); dataPos.tmpl];
            end
            model   = globalParam.ObservationModelTrain(dataPos, dataNeg, seq.opt, model);
        end
    end
    
    
    %
    figure(1),imagesc(frame);
    %         pause(0.1);
    imshow(frame);
    text(10, 15, ['#' num2str(f)], 'Color','y', 'FontWeight','bold', 'FontSize',24);
    rectangle('position', [p(1) - p(3) / 2, p(2) - p(4) / 2, p(3), p(4)], ...
        'EdgeColor','r', 'LineWidth',2);
    drawnow;
    if (seq.opt.useNormalSize)
        p(1) = p(1) * scaleWidth;
        p(3) = p(3) * scaleWidth;
        p(2) = p(2) * scaleHeight;
        p(4) = p(4) * scaleHeight;
    end
    rect = [p(1) - p(3) / 2, p(2) - p(4) / 2, p(3), p(4)];
    reportRes = [reportRes; round(rect)];
    
    
    if ~exist(pathSave)
        mkdir(pathSave)         % 若不存在，在当前目录中产生一个子目录‘Figure’
    end
    imwrite(frame2im(getframe(gcf)), [pathSave  num2str(f) '.png']);
end
if (strcmp(func2str(globalParam.ObservationModelTrain), 'SOSVMTrain'))
    mexSOSVMLearn([], [], 'delete');
end


duration = duration + toc;
fprintf('%d frames took %.3f seconds : %.3fps\n',f,duration,f/duration);
results.res=reportRes;
results.type='rect';
results.fps = f/duration;
results.len = size(seq.s_frames, 1);
end