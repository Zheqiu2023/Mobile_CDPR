function result = CDPR_Near_Distance_cal(CDPR1_State_p,CDPR1_Input_p,CDPR1_Reference_Path,CDPR1_Reference_vel,CDPR1_Reference_Acc,K)
%   计算CDPR1参考轨迹与当前轨迹最近的点
num_ref=size(CDPR1_Reference_Path,2);
dis_ref=zeros(1,num_ref);
%CDPR1_Reference_Path=CDPR1_Reference_Path';
for i=1:num_ref
    dis_ref(:,i)=norm(CDPR1_Reference_Path(:,i)-CDPR1_State_p(1:6,1));
end
near_index=find(dis_ref==min(dis_ref));

if size(near_index,2)>1
    near_index=near_index(1,1);
end
CDPR1_Reference_regin=[];
CDPR1_Reference_v=[];
CDPR1_Reference_a=[];

ref_reg1=CDPR1_Reference_Path(:,near_index);
ref_vel=CDPR1_Reference_vel(:,near_index);
ref_acc=CDPR1_Reference_Acc(:,near_index);
CDPR1_Reference_regin=[CDPR1_Reference_regin ref_reg1];
CDPR1_Reference_v=[CDPR1_Reference_v ref_vel];
CDPR1_Reference_a=[CDPR1_Reference_a ref_acc];
%ref_reg2=ref_reg1; dt=0.1;

for i=1:K
    if near_index+i <= num_ref
        %CDPR1_Reference_Path(:,near_index+i)=ref_reg2+CDPR1_Input_p*dt;
        CDPR1_Reference_regin=[CDPR1_Reference_regin CDPR1_Reference_Path(:,near_index+i)];
        CDPR1_Reference_v=[CDPR1_Reference_v CDPR1_Reference_vel(:,near_index+i)];
        CDPR1_Reference_a=[CDPR1_Reference_a CDPR1_Reference_Acc(:,near_index+i)];
        %ref_reg2=CDPR1_Reference_Path(:,near_index+i);
    else
        CDPR1_Reference_regin=[CDPR1_Reference_regin CDPR1_Reference_Path(:,end)];
        CDPR1_Reference_v=[CDPR1_Reference_v CDPR1_Reference_vel(:,end)];
        CDPR1_Reference_a=[CDPR1_Reference_a CDPR1_Reference_a(:,end)];
    end
end

result = [CDPR1_Reference_regin;CDPR1_Reference_v;CDPR1_Reference_a];
