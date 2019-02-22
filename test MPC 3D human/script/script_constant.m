%% Constant
%% Mechanics
g=9.81; %m.s-1
% h_com=0.8;
% h_com=0.748964+0.095; %m
%h_com=0.78; %m


%%
h_com=0.780678;

switch robot_type
    case 'hrp4'
        run('robot_description/hrp4.m')
    case 'human'
        run('robot_description/human.m')
    otherwise
        msg='Bad choice of robot_type \n';
        errormsg=[msg];
        error(errormsg,[])   
end

omega_temp=sqrt(g/h_com);
zeta_temp=1/omega_temp^2;

%% Phase duration
switch phase_duration_type
    case 1
        run('phase_duration/phase_duration_01.m')
    otherwise
        msg='Bad choice of phase_duration_type \n';
        errormsg=[msg];
        error(errormsg,[])   
end

preview_windows_duration=phase_duration_r+phase_duration_l+phase_duration_b*2;

T_r=phase_duration_r/N_r;
T_l=phase_duration_l/N_l;
T_b=phase_duration_b/N_b;
T_start=phase_duration_start/N_start;
T_stop=phase_duration_stop/N_stop;
%T=0.1;

%% Foot limits
switch robot_type
    case 'hrp2'
        backtoankle=0.1; %from back to ankle of foot
        fronttoankle=0.13; %from  front to ankle of foot
        exttoankle=0.075; %from exterior to ankle of foot
        inttoankle=0.055; %from interior to ankle of foot
        
        sole_margin=0.02;
    case 'hrp4'
        
    case 'human'
        
    otherwise
        msg='Bad choice of robot_type \n';
        errormsg=[msg];
        error(errormsg,[])   
end


%% COM height limits to the floor
switch robot_type
    case 'hrp4'
        switch(walking_type)
            case 1
                h_com_max=+0.0317;
                h_com_min=-0.25;
        %         h_com_max=+0.0317;
        %         h_com_min=-0.25;
            case 2
                h_com_max=+0.0317;
                h_com_min=-0.25;
            case 3
                h_com_max=+0.05;
                h_com_min=-0.1;
            case 4
                h_com_max=+0.005;
                h_com_min=-0.25;
            case 5
                h_com_max=+0.05;
                h_com_min=-0.25;
        end
    case 'human'
        h_com_max=+0.05;
        h_com_min=-0.25;
    otherwise
        msg='Bad choice of robot_type \n';
        errormsg=[msg];
        error(errormsg,[])   
end
