function [v,a] = calNF_Vel_Accel(f, p, model,vedio)
%速度与加速度初始为0
v=0;
a=0;

%添加物理特征
if(f==3)
    init_v=model.lastOutput(1,1:2);
    init_w=model.lastOutput(1,3);
    init_h=model.lastOutput(1,4);
    lastOutput
end
if(f>3)
    v=model.lastOutput(1,1:2)-init_v;
    init_v=model.lastOutput(1,1:2);
    w=model.lastOutput(1,3)-init_w;
    init_w=model.lastOutput(1,3);
    h=model.lastOutput(1,4)-init_h;
    init_h=model.lastOutput(1,4);
end

if(f==4)
    init_a=v;
    init_aw=w;
    init_ah=h;
end
if(f>4)
    a=v-init_a;
    init_a=v;
    aw=w-init_aw;
    init_aw=w;
    ah=h-init_ah;
    init_ah=h;
end