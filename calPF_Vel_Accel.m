function [pf_v,pf_a] = calPF_Vel_Accel(f,pf_p, p)

%º∆À„PFµƒv”Îa
pf_v=pf_p(1,1:2)-p(1,1:2);

if f==2
    pf_a=0;
    a_init=pf_v;
else
    pf_a=pf_v-a_init;
    a_init=pf_v;
end
