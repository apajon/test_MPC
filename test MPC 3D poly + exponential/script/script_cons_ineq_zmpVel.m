%% Constraints inequalities
    %TODO orientation
    
     polyhedron_type='waist_center';
     %'ankle_center' : polyhedron centered on the ankle
     %'foot_center' : polyhedron centered on the middle of the foot
     %'waist_center' : polyhedron centered on the middle of the waist    
     switch(polyhedron_type)
         case 'ankle_center'
             translation_x=0;
             translation_y=0;
         case 'foot_center'
             translation_x=(fronttoankle+backtoankle)/2-backtoankle;
             translation_y=-((exttoankle+inttoankle)/2-inttoankle);
         case 'waist_center'
             translation_x=0;
             translation_y=0.06845;
     end
%% Constraint ZMP in convex hull
%     no_double_support=any(phase_type_sampling_reduce~='b',2);
    no_double_support=(sum(Px_step_ref==1,2)==1);
    no_double_support=no_double_support(preview_windows,:);
    
%     right_support=any(phase_type_sampling_reduce=='r',2);
%     left_support=any(phase_type_sampling_reduce=='l',2);
    if phase_type(2)=='r'
        if phase_type_sampling_reduce(1)=='r' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce=='r'|phase_type_sampling_reduce=='l',1)]))=='r'
            left_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
            right_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        elseif phase_type_sampling_reduce(1)=='l' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce=='r'|phase_type_sampling_reduce=='l',1)]))=='l'
            right_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
            left_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        else
            right_support=false(size(no_double_support));
            left_support=false(size(no_double_support));
        end
    elseif phase_type(2)=='l'
        if phase_type_sampling_reduce(1)=='r' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce=='r'|phase_type_sampling_reduce=='l',1)]))=='r'
            right_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
            left_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        elseif phase_type_sampling_reduce(1)=='l' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce=='r'|phase_type_sampling_reduce=='l',1)]))=='l'
            left_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
            right_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        else
            right_support=false(size(no_double_support));
            left_support=false(size(no_double_support));
        end
    end
    
    if i>1 && (phase_type_sampling_reduce(1)=='b'|phase_type_sampling_reduce(1)=="start"|phase_type_sampling_reduce(1)=="stop")
        if phase_type_sampling(i-1)=='r'
            right_support(1)=true;
        elseif phase_type_sampling(i-1)=='l'
            left_support(1)=true;
        end
    end
    
    double_support=[];
    if any(Px_step_ref(1+(i-1),1:2:end)==0.5,2)||any(Px_step_ref(N+(i-1),1:2:end)==0.5,2)
        double_support=any(Px_step_ref(preview_windows,1:2:end)==0.5,2);
    end
    
    [A_zmp,b_zmp]=function_constraint_convexhull(...
    Pu_z_up,Pu_z_down,Pu_step,...
    xf_z_up,xf_z_down,xf_step,...
    yf_z_up,yf_z_down,yf_step,...
    no_double_support,double_support,right_support,left_support,...
    fronttoankle,backtoankle,inttoankle,exttoankle,sole_margin);
        
    %% Constraint foot step stretching
    A_step_stretch=[];
    b_step_stretch=[];
    xb_step_stretch=[];yb_step_stretch=[];
    
    if size(Pu_step,2)>=1
        A_step_stretch=eye(size(Pu_step,2));
        A_step_stretch(2:end,1:end-1)=A_step_stretch(2:end,1:end-1)-eye(size(Pu_step,2)-1);
        A_step_stretch=[zeros(size(Pu_step,2),N) A_step_stretch];
        A_step_stretch=[A_step_stretch;-A_step_stretch];
        
        A_step_stretch=blkdiag(A_step_stretch*0,A_step_stretch);
        
        %%%
        phase_type_nodouble_reduce=phase_type_reduce(any(phase_type_reduce~='b'&phase_type_reduce~="start"&phase_type_reduce~="stop",2));
%         phase_type_nodouble_reduce=phase_type_nodouble_reduce(any(phase_type_nodouble_reduce~="start",2));
%         phase_type_nodouble_reduce=phase_type_nodouble_reduce(any(phase_type_nodouble_reduce~="stop",2));
        
        if phase_type_reduce(end)=="stop" && Pu_step(end,end)==0.5
            if phase_type_nodouble_reduce(end)=='r'
                phase_type_nodouble_reduce(end+1)='l';
            else
                phase_type_nodouble_reduce(end+1)='r';
            end
        end
        
        phase_type_nodouble_reduce(1)=[];
        %%%
        xb_step_stretch=[ones(size(Pu_step,2),1)*xankmax;...
            ones(size(Pu_step,2),1)*(-xankmin)];
        xb_step_stretch([1 end/2+1])=xb_step_stretch([1 end/2+1])+[xstep(end);-xstep(end)];
 
        yb_step_stretch=[any(phase_type_nodouble_reduce=='l',2)*yankmax-any(phase_type_nodouble_reduce=='r',2)*yankmin;...
            any(phase_type_nodouble_reduce=='l',2)*(-yankmin)-any(phase_type_nodouble_reduce=='r',2)*(-yankmax)];
        yb_step_stretch([1 end/2+1])=yb_step_stretch([1 end/2+1])+[ystep(end);-ystep(end)];
    end
    
    b_step_stretch=[xb_step_stretch*0;yb_step_stretch];
    
    % Constraint concatenation
    if i>=length(xvcom_ref)-39
        A=[A_zmp;A_step_stretch];
        b=[b_zmp;b_step_stretch]; 
    else
        A=[A_zmp;A_step_stretch];
        b=[b_zmp;b_step_stretch]; 
    end
       
    
    A=[A zeros(size(A,1),size(H_c,2))];    
    %% Constraint vertical com motion
    %zeta_down (zddc - g) < (zc-zp) < zeta_up (zddc + g)
    A_verti_motion_up=Pu_c-diag(zeta_up_ref(preview_windows,:))*Pu_ddc;
    b_verti_motion_up=zeta_up_ref(preview_windows,:).*g+zzmp_ref_reduce-(zf_c-zeta_up_ref(preview_windows,:).*Px_ddc*[zc(i);zdc(i);zddc(i)]);

    A_verti_motion_down=Pu_c-diag(zeta_down_ref(preview_windows,:))*Pu_ddc;
    b_verti_motion_down=zeta_down_ref(preview_windows,:).*g+zzmp_ref_reduce-(zf_c-zeta_down_ref(preview_windows,:).*Px_ddc*[zc(i);zdc(i);zddc(i)]);

    
    
    if 0%i==29
%         toto=15;
%         A_verti_motion=[A_verti_motion_up(1:end-toto,:);-A_verti_motion_down(1:end-toto,:)];
%         b_verti_motion=[b_verti_motion_up(1:end-toto,:);-b_verti_motion_down(1:end-toto,:)];
        A_verti_motion=[A_verti_motion_up(2:end,:)*0;-A_verti_motion_down(2:end,:)*0];
        b_verti_motion=[b_verti_motion_up(2:end,:)*0;-b_verti_motion_down(2:end,:)*0];
    else
        A_verti_motion=[A_verti_motion_up;-A_verti_motion_down];
        b_verti_motion=[b_verti_motion_up;-b_verti_motion_down];
    end

    %% COM accel z > -g
    A_verti_acc=-Pu_ddc;
    b_verti_acc=g+Px_ddc*[zc(i);zdc(i);zddc(i)];    
    
    %%
    A_verti=[A_verti_motion;A_verti_acc];
    b_verti=[b_verti_motion;b_verti_acc];
    
    A=[A;zeros(size(A_verti,1),size(A_zmp,2)) A_verti];
    b=[b;b_verti];
    
    
    %% constraint kinematics com height (polyhedron)
    switch kinematic_limit
        case ''
            [A_diff_c_p,b_diff_c_p]=function_constraint_polyhedron(...
            Pu_c,Pu_step,...
            xf_c,xf_step,...
            yf_c,yf_step,...
            zf_c,zzmp_ref_reduce,...
            rot_successive,polyhedron_lim);
        case 'hexagon'
            [A_diff_c_p,b_diff_c_p]=function_constraint_polyhedron_hexagon(...
            Pu_c,Pu_step,...
            xf_c,xf_step,...
            yf_c,yf_step,...
            zf_c,zzmp_ref_reduce,...%zzmp_ref_reduce
            plan_hexagon,h_com+h_com_min);
        case 'hexagonTranslation'
            [A_diff_c_p,b_diff_c_p]=function_constraint_polyhedron_hexagon_translation(...
            Pu_c,Pu_step,...
            xf_c,xf_step,...
            yf_c,yf_step,...
            zf_c,zzmp_ref_reduce,...%zzmp_ref_reduce
            plan_hexagon,z_leg_min+z_decalage_tot,translation_x,translation_y,...
            left_support,right_support);
        otherwise
%             left_support
%             right_support
            error('choose a type of kinematic_limit')
    end
    

    

    
    if i>=length(xvcom_ref)-39
        A=[A;A_diff_c_p];
        b=[b;b_diff_c_p];
    else
        A=[A;A_diff_c_p];
        b=[b;b_diff_c_p];
    end


%     A_height=[Pu_c;-Pu_c];
%     b_height=[0.9-zf_c;-0.7+zf_c];
% 
%     A=[A;zeros(size(A_height,1),size(A_zmp,2)) A_height];
%     b=[b;b_height];

%% Constraint Last Capture point in convex hull
    no_double_support_capture=(sum(Px_step_ref==1,2)==1);
    no_double_support_capture=no_double_support_capture(preview_windows,:);
    no_double_support_capture(1:end-1,:)=[no_double_support_capture(1:end-1,:)==2];

    if phase_type_sampling_reduce(1)=='r' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce=='r'|phase_type_sampling_reduce=='l',1)]))=='r'
        right_support_capture=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        left_support_capture=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif phase_type_sampling_reduce(1)=='l' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce=='r'|phase_type_sampling_reduce=='l',1)]))=='l'
        right_support_capture=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        left_support_capture=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support_capture=false(size(no_double_support_capture));
        left_support_capture=false(size(no_double_support_capture));
    end
        
%     [A_Capture,b_Capture]=function_constraint_convexhull(...
%     Pu_Capture_up,Pu_Capture_down,Pu_step,...
%     xf_Capture_up,xf_Capture_down,xf_step,...
%     yf_Capture_up,yf_Capture_down,yf_step,...
%     no_double_support_capture,[],right_support_capture,left_support_capture,...
%     fronttoankle,backtoankle,inttoankle,exttoankle,sole_margin);
    switch(COM_form)
        case {'com jerk','zmp vel'}
                [A_Capture,b_Capture]=function_constraint_convexhull(...
                    Pu_c+Pu_dc/omega_temp,Pu_c+Pu_dc/omega_temp,Pu_step,...
                    xf_c+xf_dc/omega_temp,xf_c+xf_dc/omega_temp,xf_step,...
                    yf_c+yf_dc/omega_temp,yf_c+yf_dc/omega_temp,yf_step,...
                    no_double_support_capture,[],right_support,left_support,...
                    fronttoankle,backtoankle,inttoankle,exttoankle,sole_margin);
        case 'poly expo'
                [A_Capture,b_Capture]=function_constraint_convexhull(...
                    Pu_c+3/2*Pu_dc/omega_temp+1/2*Pu_ddc/omega_temp^2,Pu_c+3/2*Pu_dc/omega_temp+1/2*Pu_ddc/omega_temp^2,Pu_step,...
                    xf_c+3/2*xf_dc/omega_temp+1/2*xf_ddc/omega_temp^2,xf_c+3/2*xf_dc/omega_temp+1/2*xf_ddc/omega_temp^2,xf_step,...
                    yf_c+3/2*yf_dc/omega_temp+1/2*yf_ddc/omega_temp^2,yf_c+3/2*yf_dc/omega_temp+1/2*yf_ddc/omega_temp^2,yf_step,...
                    no_double_support_capture,[],right_support,left_support,...
                    fronttoankle,backtoankle,inttoankle,exttoankle,sole_margin);
        otherwise
            error('Bad COM_form')
    end

    
    A_Capture=[A_Capture zeros(size(A_Capture,1),size(H_c,2))];  
    
    if size(A_Capture,1)>0
        A_Capture([3 4 7 8],:)=[];
        b_Capture([3 4 7 8],:)=[];
    end
    
    % Constraint concatenation
    if i>=length(xvcom_ref)-39 %|| i==40 %|| (phase_type_sampling_reduce(end-3)~=phase_type_sampling_reduce(end)&&phase_type_sampling_reduce(end-3)~="b"&&phase_type_sampling_reduce(end)~="b")
        A=[A;A_Capture*0];
        b=[b;b_Capture*0]; 
    else
        A=[A;A_Capture];
        b=[b;b_Capture]; 
    end
       
%% constraint kinematics Last capture point height (polyhedron)
    if isempty(Pu_step) %deal with indices of empty matrix
        Pu_step_temp=ones(1,0);
    else
        Pu_step_temp=Pu_step(end,:);
    end
    
%     switch kinematic_limit
%         case ''
%             [A_diff_capture_p_down,b_diff_capture_p_down]=function_constraint_polyhedron(...
%             Pu_Capture_down(end,:),Pu_step_temp,...
%             xf_Capture_down(end,:),xf_step(end,:),...
%             yf_Capture_down(end,:),yf_step(end,:),...
%             zf_Capture_down(end,:),zzmp_ref_reduce(end,:),...
%             rot_successive,polyhedron_lim);
%         case 'hexagon'
%             [A_diff_capture_p_down,b_diff_capture_p_down]=function_constraint_polyhedron_hexagon(...
%             Pu_Capture_down(end,:),Pu_step_temp,...
%             xf_Capture_down(end,:),xf_step(end,:),...
%             yf_Capture_down(end,:),yf_step(end,:),...
%             zf_Capture_down(end,:),zzmp_ref_reduce(end,:),...%zzmp_ref_reduce
%             plan_hexagon,h_com+h_com_min);
%         case 'hexagonTranslation'
%             [A_diff_capture_p_down,b_diff_capture_p_down]=function_constraint_polyhedron_hexagon_translation(...
%             Pu_Capture_down(end,:),Pu_step_temp,...
%             xf_Capture_down(end,:),xf_step(end,:),...
%             yf_Capture_down(end,:),yf_step(end,:),...
%             zf_Capture_down(end,:),zzmp_ref_reduce(end,:),...%zzmp_ref_reduce
%             plan_hexagon,z_leg_min+z_decalage_tot,translation_x,translation_y,...
%             left_support(end,:),right_support(end,:));
%         otherwise
%             error('choose a type of kinematic_limit')
%     end
%  
%     if i>=length(xvcom_ref)-39
%         A=[A;A_diff_capture_p_down];
%         b=[b;b_diff_capture_p_down];
%     else
%         A=[A;A_diff_capture_p_down];
%         b=[b;b_diff_capture_p_down];
%     end
%     
%     switch kinematic_limit
%         case ''
%             [A_diff_capture_p_up,b_diff_capture_p_up]=function_constraint_polyhedron(...
%             Pu_Capture_up(end,:),Pu_step_temp,...
%             xf_Capture_up(end,:),xf_step(end,:),...
%             yf_Capture_up(end,:),yf_step(end,:),...
%             zf_Capture_up(end,:),zzmp_ref_reduce(end,:),...
%             rot_successive,polyhedron_lim);
%         case 'hexagon'
%             [A_diff_capture_p_up,b_diff_capture_p_up]=function_constraint_polyhedron_hexagon(...
%             Pu_Capture_up(end,:),Pu_step_temp,...
%             xf_Capture_up(end,:),xf_step(end,:),...
%             yf_Capture_up(end,:),yf_step(end,:),...
%             zf_Capture_up(end,:),zzmp_ref_reduce(end,:),...%zzmp_ref_reduce
%             plan_hexagon,h_com+h_com_min);
%         case 'hexagonTranslation'
%             [A_diff_capture_p_up,b_diff_capture_p_up]=function_constraint_polyhedron_hexagon_translation(...
%             Pu_Capture_up(end,:),Pu_step_temp,...
%             xf_Capture_up(end,:),xf_step(end,:),...
%             yf_Capture_up(end,:),yf_step(end,:),...
%             zf_Capture_up(end,:),zzmp_ref_reduce(end,:),...%zzmp_ref_reduce
%             plan_hexagon,z_leg_min+z_decalage_tot,translation_x,translation_y,...
%             left_support(end,:),right_support(end,:));
%         otherwise
%             error('choose a type of kinematic_limit')
%     end
%     
%     
% 
%     
%     if i>=length(xvcom_ref)-39
%         A=[A;A_diff_capture_p_up];
%         b=[b;b_diff_capture_p_up];
%     else
%         A=[A;A_diff_capture_p_up];
%         b=[b;b_diff_capture_p_up];
%     end
     
    switch(COM_form)
        case {'com jerk','zmp vel'}
            switch kinematic_limit
                case ''
                    [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron(...
                    Pu_c(end,:)+Pu_dc(end,:)/omega_temp,Pu_step_temp,...
                    xf_c(end,:)+xf_dc(end,:)/omega_temp,xf_step(end,:),...
                    yf_c(end,:)+yf_dc(end,:)/omega_temp,yf_step(end,:),...
                    zf_c(end,:)+zf_dc(end,:)/omega_temp,zzmp_ref_reduce(end,:),...
                    rot_successive,polyhedron_lim);
                case 'hexagon'
                    [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron_hexagon(...
                    Pu_c(end,:)+Pu_dc(end,:)/omega_temp,Pu_step_temp,...
                    xf_c(end,:)+xf_dc(end,:)/omega_temp,xf_step(end,:),...
                    yf_c(end,:)+yf_dc(end,:)/omega_temp,yf_step(end,:),...
                    zf_c(end,:)+zf_dc(end,:)/omega_temp,zzmp_ref_reduce(end,:),...%zzmp_ref_reduce
                    plan_hexagon,h_com+h_com_min);
                case 'hexagonTranslation'
                    [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron_hexagon_translation(...
                    Pu_c(end,:)+Pu_dc(end,:)/omega_temp,Pu_step_temp,...
                    xf_c(end,:)+xf_dc(end,:)/omega_temp,xf_step(end,:),...
                    yf_c(end,:)+yf_dc(end,:)/omega_temp,yf_step(end,:),...
                    zf_c(end,:)+zf_dc(end,:)/omega_temp,zzmp_ref_reduce(end,:),...%zzmp_ref_reduce
                    plan_hexagon,z_leg_min+z_decalage_tot,translation_x,translation_y,...
                    left_support(end,:),right_support(end,:));
                otherwise
                    error('choose a type of kinematic_limit')
            end
        case 'poly expo'
                switch kinematic_limit
                    case ''
                        [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron(...
                        Pu_c(end,:)+3/2*Pu_dc(end,:)/omega_temp+1/2*Pu_ddc(end,:)/omega_temp^2,Pu_step_temp,...
                        xf_c(end,:)+3/2*xf_dc(end,:)/omega_temp+1/2*xf_ddc(end,:)/omega_temp^2,xf_step(end,:),...
                        yf_c(end,:)+3/2*yf_dc(end,:)/omega_temp+1/2*yf_ddc(end,:)/omega_temp^2,yf_step(end,:),...
                        zf_c(end,:)+3/2*zf_dc(end,:)/omega_temp+1/2*zf_ddc(end,:)/omega_temp^2,zzmp_ref_reduce(end,:),...%zzmp_ref_reduce
                        rot_successive,polyhedron_lim);
                    case 'hexagon'
                        [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron_hexagon(...
                        Pu_c(end,:)+3/2*Pu_dc(end,:)/omega_temp+1/2*Pu_ddc(end,:)/omega_temp^2,Pu_step_temp,...
                        xf_c(end,:)+3/2*xf_dc(end,:)/omega_temp+1/2*xf_ddc(end,:)/omega_temp^2,xf_step(end,:),...
                        yf_c(end,:)+3/2*yf_dc(end,:)/omega_temp+1/2*yf_ddc(end,:)/omega_temp^2,yf_step(end,:),...
                        zf_c(end,:)+3/2*zf_dc(end,:)/omega_temp+1/2*zf_ddc(end,:)/omega_temp^2,zzmp_ref_reduce(end,:),...%zzmp_ref_reduce
                        plan_hexagon,h_com+h_com_min);
                    case 'hexagonTranslation'
                        [A_diff_capture_p,b_diff_capture_p]=function_constraint_polyhedron_hexagon_translation(...
                        Pu_c(end,:)+3/2*Pu_dc(end,:)/omega_temp+1/2*Pu_ddc(end,:)/omega_temp^2,Pu_step_temp,...
                        xf_c(end,:)+3/2*xf_dc(end,:)/omega_temp+1/2*xf_ddc(end,:)/omega_temp^2,xf_step(end,:),...
                        yf_c(end,:)+3/2*yf_dc(end,:)/omega_temp+1/2*yf_ddc(end,:)/omega_temp^2,yf_step(end,:),...
                        zf_c(end,:)+3/2*zf_dc(end,:)/omega_temp+1/2*zf_ddc(end,:)/omega_temp^2,zzmp_ref_reduce(end,:),...%zzmp_ref_reduce
                        plan_hexagon,z_leg_min+z_decalage_tot,translation_x,translation_y,...
                        left_support(end,:),right_support(end,:));
                    otherwise
                        error('choose a type of kinematic_limit')
                end
        otherwise
            error('Bad COM_form')
    end

    
    if i>=length(xvcom_ref)-39 %|| i==40
        A=[A;A_diff_capture_p*0];
        b=[b;b_diff_capture_p*0];
    else
        A=[A;A_diff_capture_p];
        b=[b;b_diff_capture_p];
    end
    
%% Constraint vertical com motion on horizon
    %zeta_down (zddc - g) < (zc-zp) < zeta_up (zddc + g)
    switch(COM_form)
        case {'com jerk','zmp vel'}
            A_verti_motion_up_horizon=Pu_c(end,:)+Pu_dc(end,:)/omega_temp;
            b_verti_motion_up_horizon=zeta_up_ref(N+(i-1),:).*g+zzmp_ref_reduce(end,:)-(zf_c(end,:)+Px_dc(end,:)*[zc(i);zdc(i);zddc(i)]/omega_temp);

            A_verti_motion_down_horizon=Pu_c(end,:)+Pu_dc(end,:)/omega_temp;
            b_verti_motion_down_horizon=zeta_down_ref(N+(i-1),:).*g+zzmp_ref_reduce(end,:)-(zf_c(end,:)+Px_dc(end,:)*[zc(i);zdc(i);zddc(i)]/omega_temp);

            A_verti_motion_horizon=[A_verti_motion_up_horizon;-A_verti_motion_down_horizon];
            b_verti_motion_horizon=[b_verti_motion_up_horizon;-b_verti_motion_down_horizon];        
        case 'poly expo'
            A_verti_motion_up_horizon=Pu_c(end,:)+3/2*Pu_dc(end,:)/omega_temp+1/2*Pu_ddc(end,:)/omega_temp^2;
            b_verti_motion_up_horizon=zeta_up_ref(N+(i-1),:).*g+zzmp_ref_reduce(end,:)-(zf_c(end,:)+3/2*zf_dc(end,:)/omega_temp+1/2*zf_ddc(end,:)/omega_temp^2);

            A_verti_motion_down_horizon=Pu_c(end,:)+3/2*Pu_dc(end,:)/omega_temp+1/2*Pu_ddc(end,:)/omega_temp^2;
            b_verti_motion_down_horizon=zeta_down_ref(N+(i-1),:).*g+zzmp_ref_reduce(end,:)-(zf_c(end,:)+3/2*zf_dc(end,:)/omega_temp+1/2*zf_ddc(end,:)/omega_temp^2);

            A_verti_motion_horizon=[A_verti_motion_up_horizon;-A_verti_motion_down_horizon];
            b_verti_motion_horizon=[b_verti_motion_up_horizon;-b_verti_motion_down_horizon];
        otherwise
            error('Bad COM_form')
    end

    
    
    if 0
        A=[A;zeros(size(A_verti_motion_horizon,1),size(A_zmp,2)) A_verti_motion_horizon*0];
        b=[b;b_verti_motion_horizon*0];
    else
        A=[A;zeros(size(A_verti_motion_horizon,1),size(A_zmp,2)) A_verti_motion_horizon];
        b=[b;b_verti_motion_horizon];
    end
    
%% constraint kinematics COM height (polyhedron) beforte and after DSP
    double_support_before=[any([phase_type_sampling_reduce(1:end-1)=='b']+[phase_type_sampling_reduce(2:end)~='b']==2,2);false];
    double_support_after=[false;any([phase_type_sampling_reduce(1:end-1)=='b']+[phase_type_sampling_reduce(2:end)~='b']==2,2)];
    
%     if phase_type_sampling_reduce(1)~='b' || (phase_type_sampling_reduce(1)~='b'&&phase_type_sampling_reduce(2)~='b')
%         
%         double_support_after=find(double_support_after);
%         
%         double_support_before=logical(double_support_before+[double_support_before(2:end);false]);
%         
%         length_double_support_after=length(double_support_after);
%         for j=1:length_double_support_after
%             double_support_after=[double_support_after(1:length_double_support_after-j) double_support_after(length_double_support_after-j+1) double_support_after(length_double_support_after-j+1:end)];
%         end
%     end

    
    if isempty(Pu_step) %deal with indices of empty matrix
        Pu_step_temp=ones(sum(double_support_after),0);
    else
        Pu_step_temp=Pu_step(double_support_after,:);
    end
    
    switch kinematic_limit
        case ''
            [A_diff_c_p_DSP_before,b_diff_c_p_DSP_before]=function_constraint_polyhedron(...
            Pu_c(double_support_before,:),Pu_step_temp,...
            xf_c(double_support_before,:),xf_step(double_support_after,:),...
            yf_c(double_support_before,:),yf_step(double_support_after,:),...
            zf_c(double_support_before,:),zzmp_ref_reduce(double_support_after,:),...
            rot_successive,polyhedron_lim);
        case 'hexagon'
            [A_diff_c_p_DSP_before,b_diff_c_p_DSP_before]=function_constraint_polyhedron_hexagon(...
            Pu_c(double_support_before,:),Pu_step_temp,...
            xf_c(double_support_before,:),xf_step(double_support_after,:),...
            yf_c(double_support_before,:),yf_step(double_support_after,:),...
            zf_c(double_support_before,:),zzmp_ref_reduce(double_support_after,:),...%zzmp_ref_reduce
            plan_hexagon,h_com+h_com_min);
        case 'hexagonTranslation'
%             [A_diff_c_p_DSP_before,b_diff_c_p_DSP_before]=function_constraint_polyhedron_hexagon_translation(...
%             Pu_c(double_support_before,:),Pu_step_temp,...
%             xf_c(double_support_before,:),xf_step(double_support_after,:),...
%             yf_c(double_support_before,:),yf_step(double_support_after,:),...
%             zf_c(double_support_before,:),zzmp_ref_reduce(double_support_after,:),...%zzmp_ref_reduce
%             plan_hexagon,z_leg_min+z_decalage_tot,translation_x,translation_y,...
%             left_support(double_support_after,:),right_support(double_support_after,:));
        
            [A_diff_c_p_DSP_before,b_diff_c_p_DSP_before]=function_constraint_polyhedron_hexagon_translation(...
            Pu_c(double_support_before,:),Pu_step_temp,...
            xf_c(double_support_before,:),xf_step(double_support_after,:),...
            yf_c(double_support_before,:),yf_step(double_support_after,:),...
            zf_c(double_support_before,:),zzmp_ref_reduce(double_support_after,:),...%zzmp_ref_reduce
            plan_hexagon,z_leg_min+z_decalage_tot,translation_x,translation_y,...
            left_support(double_support_after,:),right_support(double_support_after,:));
        otherwise
            error('choose a type of kinematic_limit')
    end
    
    

    
    if i>=length(xvcom_ref)-39
        A=[A;A_diff_c_p_DSP_before];
        b=[b;b_diff_c_p_DSP_before];
    else
        A=[A;A_diff_c_p_DSP_before];
        b=[b;b_diff_c_p_DSP_before];
    end
    
    double_support_before=[any([phase_type_sampling_reduce(1:end-1)=='b']+[phase_type_sampling_reduce(2:end)~='b']==2,2);false];
    double_support_after=[false;any([phase_type_sampling_reduce(1:end-1)=='b']+[phase_type_sampling_reduce(2:end)~='b']==2,2)];
    
    if isempty(Pu_step) %deal with indices of empty matrix
        Pu_step_temp=ones(sum(double_support_before),0);
    else
        Pu_step_temp=Pu_step(double_support_before,:);
    end
    
    switch kinematic_limit
        case ''
            [A_diff_c_p_DSP_after,b_diff_c_p_DSP_after]=function_constraint_polyhedron(...
            Pu_c(double_support_after,:),Pu_step_temp,...
            xf_c(double_support_after,:),xf_step(double_support_before,:),...
            yf_c(double_support_after,:),yf_step(double_support_before,:),...
            zf_c(double_support_after,:),zzmp_ref_reduce(double_support_before,:),...
            rot_successive,polyhedron_lim);
        case 'hexagon'
            [A_diff_c_p_DSP_after,b_diff_c_p_DSP_after]=function_constraint_polyhedron_hexagon(...
            Pu_c(double_support_after,:),Pu_step_temp,...
            xf_c(double_support_after,:),xf_step(double_support_before,:),...
            yf_c(double_support_after,:),yf_step(double_support_before,:),...
            zf_c(double_support_after,:),zzmp_ref_reduce(double_support_before,:),...%zzmp_ref_reduce
            plan_hexagon,h_com+h_com_min);
         case 'hexagonTranslation'
            [A_diff_c_p_DSP_after,b_diff_c_p_DSP_after]=function_constraint_polyhedron_hexagon_translation(...
            Pu_c(double_support_after,:),Pu_step_temp,...
            xf_c(double_support_after,:),xf_step(double_support_before,:),...
            yf_c(double_support_after,:),yf_step(double_support_before,:),...
            zf_c(double_support_after,:),zzmp_ref_reduce(double_support_before,:),...%zzmp_ref_reduce
            plan_hexagon,z_leg_min+z_decalage_tot,translation_x,translation_y,...
            left_support(double_support_before,:),right_support(double_support_before,:));
        otherwise
            error('choose a type of kinematic_limit')
    end
    

    
    if i>=length(xvcom_ref)-39
        A=[A;A_diff_c_p_DSP_after];
        b=[b;b_diff_c_p_DSP_after];
    else
        A=[A;A_diff_c_p_DSP_after];
        b=[b;b_diff_c_p_DSP_after];
    end
