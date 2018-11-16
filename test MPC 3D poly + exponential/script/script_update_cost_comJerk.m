%% update cost function
% min com pos to ref
H_c=Pu_c.'*Pu_c;
% min com vel to ref
H_dc=Pu_dc.'*Pu_dc;

% min com Jerk
H_dddc=eye(N);

% min com acc
H_ddc=Pu_ddc.'*Pu_ddc;

% min zmp vel
Pu_dz=Pu_dz_mean(2:end,:)-Pu_dz_mean(1:end-1,:);
H_dz=Pu_dz.'*Pu_dz;

xf_dz=xf_dz_mean(2:end,:)-xf_dz_mean(1:end-1,:);
yf_dz=yf_dz_mean(2:end,:)-yf_dz_mean(1:end-1,:);

%%
% add min(zeta_mean-foot_step)
% add variable part of min(com_vel-com_vel_ref)
% xtranslate_step=(fronttoankle+backtoankle)/2-backtoankle;
xtranslate_step=0;
ytranslate_step=(exttoankle+inttoankle)/2-inttoankle;

cop_ref_type='ankle_center';
%'ankle_center' : polyhedron centered on the ankle
%'foot_center' : polyhedron centered on the middle of the foot  
switch(cop_ref_type)
    case 'ankle_center'
        xtranslate_step=0;
        ytranslate_step=0;
    case 'foot_center'
        xtranslate_step=(fronttoankle+backtoankle)/2-backtoankle;
        ytranslate_step=(exttoankle+inttoankle)/2-inttoankle;
    case 'waist_center'
end

% if i<=phase_sampling_length(3)
%     w1=10^-1;
% else
%     w1=10^-7;
% end
H_Pu=w1*H_dddc+w2*H_dc+w5*H_ddc+w6*H_dz;

%     no_double_support=any(phase_type_sampling_reduce~='b',2);
no_double_support=(sum(Px_step_ref==1,2)==1);
no_double_support=no_double_support(preview_windows,:);

%     right_support=any(phase_type_sampling_reduce=='r',2);
%     left_support=any(phase_type_sampling_reduce=='l',2);

firstSS=min(find(phase_type=='r',1),find(phase_type=='l',1));
if phase_type(firstSS)=='r'
    if phase_type_sampling_reduce(1)=='r' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce~='b',1)]))=='r'
        left_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        right_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif phase_type_sampling_reduce(1)=='l' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce~='b',1)]))=='l'
        right_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        left_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support=false(size(no_double_support));
        left_support=false(size(no_double_support));
    end
elseif phase_type(firstSS)=='l'
    if phase_type_sampling_reduce(1)=='r' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce~='b',1)]))=='r'
        right_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        left_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif phase_type_sampling_reduce(1)=='l' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce~='b',1)]))=='l'
        left_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        right_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support=false(size(no_double_support));
        left_support=false(size(no_double_support));
    end
end


if isempty(Pu_step)
    xH=H_Pu+w3*H_z_mean;

    xf=w2*Pu_dc.'*(xf_dc-xf_dc_ref)...
        +w3*Pu_z_mean.'*(xf_z_mean-(xf_step+xtranslate_step))...
        +w5*Pu_ddc.'*xf_ddc...
        +w6*Pu_dz.'*xf_dz;

    yf=w2*Pu_dc.'*(yf_dc-yf_dc_ref)...
        +w3*Pu_z_mean.'*(yf_z_mean-(yf_step+(left_support-right_support)*ytranslate_step))...
        +w5*Pu_ddc.'*yf_ddc...
        +w6*Pu_dz.'*yf_dz;
else
    xH=[H_Pu+w3*H_z_mean -w3*Pu_z_mean.'*Pu_step;...
        (-w3*Pu_z_mean.'*Pu_step).' w3*H_step];

    xf=[w2*Pu_dc.'*(xf_dc-xf_dc_ref)+...
        w3*Pu_z_mean.'*(xf_z_mean-(xf_step+xtranslate_step))+...
        w5*Pu_ddc.'*xf_ddc+...
        w6*Pu_dz.'*xf_dz; ...
        -w3*Pu_step.'*(xf_z_mean-(xf_step+xtranslate_step))];

    yf=[w2*Pu_dc.'*(yf_dc-yf_dc_ref)+...
        w3*Pu_z_mean.'*(yf_z_mean-(yf_step+(left_support-right_support)*ytranslate_step))+...
        +w5*Pu_ddc.'*yf_ddc+...
        w6*Pu_dz.'*yf_dz;...
        -w3*Pu_step.'*(yf_z_mean-(yf_step+(left_support-right_support)*ytranslate_step))];
end

yH=xH;

H=blkdiag(xH,yH);

f=[xf;yf];
    
%%
%add Cost com heigth and min jerk
zf_c=Px_c*[zc(i);zdc(i);zddc(i)];

zzmp_ref_reduce=zzmp_ref(preview_windows,:);
hcom_ref_reduce=hcom_ref(preview_windows,:);
hcom_ref_max_reduce=hcom_ref_max(preview_windows,:);
%     if i<10
%         hcom_ref_reduce=0.8;
%     elseif i<120
%         hcom_ref_reduce=0.95;%h_com+h_com_max;%hcom_ref(preview_windows,:);
%     else
%         hcom_ref_reduce=0.8;
%     end

% zf=w4*Pu_c.'*(zf_c-zzmp_ref_reduce-hcom_ref_max_reduce);
zf=w4*Pu_c.'*(zf_c-hcom_ref_max_reduce);

H=blkdiag(H,w1*H_dddc+w4*H_c); % add min jerk
% H=blkdiag(H,w4*H_c+10^-7*H_dddc); % add min jerk
f=[f;zf];