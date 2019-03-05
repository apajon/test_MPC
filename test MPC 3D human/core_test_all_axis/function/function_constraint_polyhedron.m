function [A,b]=function_constraint_polyhedron(...
    Pu_c,Pu_step,...
    xf_c,xf_step,...
    yf_c,yf_step,...
    zf_c,zf_step,...%zzmp_ref_reduce
    rot_successive,polyhedron_lim)
%compute linear matrix to verify in a quadprog optimization
%Ax<b
%% constraint kinematics com height (polyhedron)
    Pu_diff_c_p=[Pu_c zeros(size(Pu_step,1),size(Pu_step,2))]-[zeros(size(Pu_c,1),size(Pu_c,2)) Pu_step];
    z_Pu_diff_c=Pu_c;

    xf_diff_c_p=xf_c-xf_step;
    yf_diff_c_p=yf_c-yf_step;
    zf_diff_C_p=zf_c-zf_step;

%     A_diff_c_p_no_z=zeros(N*size(rot_successive,1),size(Pu_diff_c_p,2));
%     zA_diff_c_p=zeros(N*size(rot_successive,1),size(z_Pu_diff_c,2));
%     xzb_diff_c_p=zeros(N*size(rot_successive,1),1);
%     yzb_diff_c_p=zeros(N*size(rot_successive,1),1);
    A_diff_c_p_no_z=[];
    zA_diff_c_p=[];
    xzb_diff_c_p=[];
    yzb_diff_c_p=[];
    if isempty(z_Pu_diff_c)==0
        for j=1:size(rot_successive,1)
            sin_sampled=rot_successive(j,1);
            cos_sampled=rot_successive(j,2);
            polyhedron_lim_sampled=polyhedron_lim(j,1);

            
%             A_diff_c_p_no_z((1:N)+N*(j-1),:)=sin_sampled*Pu_diff_c_p;
%             zA_diff_c_p((1:N)+N*(j-1),:)=cos_sampled*z_Pu_diff_c;
            A_diff_c_p_no_z=[A_diff_c_p_no_z;sin_sampled*Pu_diff_c_p(:,:)];
            zA_diff_c_p=[zA_diff_c_p;cos_sampled*z_Pu_diff_c(:,:)];
            
%             xzb_diff_c_p((1:N)+N*(j-1),:)=polyhedron_lim_sampled-sin_sampled*xf_diff_c_p-cos_sampled*zf_diff_C_p;
%             yzb_diff_c_p((1:N)+N*(j-1),:)=polyhedron_lim_sampled-sin_sampled*yf_diff_c_p-cos_sampled*zf_diff_C_p;
            xzb_diff_c_p=[xzb_diff_c_p;polyhedron_lim_sampled-sin_sampled*xf_diff_c_p(:,:)-cos_sampled*zf_diff_C_p(:,:)];
            yzb_diff_c_p=[yzb_diff_c_p;polyhedron_lim_sampled-sin_sampled*yf_diff_c_p(:,:)-cos_sampled*zf_diff_C_p(:,:)];
        end
    end

    A=[A_diff_c_p_no_z zeros(size(A_diff_c_p_no_z)) zA_diff_c_p;...
        zeros(size(A_diff_c_p_no_z)) A_diff_c_p_no_z zA_diff_c_p];

    b=[xzb_diff_c_p;yzb_diff_c_p];