classdef classdef_create_robot<handle
    %%
    properties (SetAccess = protected)

        h_com


        h_com_max
        h_com_min

        xcom_0
        ycom_0
        zcom_0

        xstep_r_0
        ystep_r_0

        xstep_l_0
        ystep_l_0

        backtoankle %from back to ankle of foot
        fronttoankle %from  front to ankle of foot
        exttoankle %from exterior to ankle of foot
        inttoankle %from interior to ankle of foot

        sole_margin


        xankmax%stepping forward max
        xankmin%stepping forward min (if negative, it means stepping backward max)
        yankmin%width min between ankles
        yankmax%width max between ankles
    end
    %%
    methods
        %%
        function obj=classdef_create_robot(robot_type)
            robot_type_namefile=['config/robot_description/' robot_type '.m'];
            if isfile(robot_type_namefile)
                run(robot_type_namefile)
            else
                msg='Bad choice of robot_type \n';
                errormsg=[msg];
                error(errormsg,[])  
            end
%             switch robot_type
%                 case 'hrp2'
%                     run('config/robot_description/hrp2.m')
%                 case 'hrp4'
%                     run('config/robot_description/hrp4.m')
%                 case 'human'
%                     run('config/robot_description/human.m')
%                 otherwise
%                     msg='Bad choice of robot_type \n';
%                     errormsg=[msg];
%                     error(errormsg,[])   
%             end
            
            obj.h_com=h_com;

            obj.h_com_max=h_com_max;
            obj.h_com_min=h_com_min;

            obj.xcom_0=xcom_0;
            obj.ycom_0=ycom_0;
            obj.zcom_0=zcom_0;

            obj.xstep_r_0=xstep_r_0;
            obj.ystep_r_0=ystep_r_0;

            obj.xstep_l_0=xstep_l_0;
            obj.ystep_l_0=ystep_l_0;

            obj.backtoankle=backtoankle;
            obj.fronttoankle=fronttoankle;
            obj.exttoankle=exttoankle;
            obj.inttoankle=inttoankle;

            obj.sole_margin=sole_margin;

            obj.xankmax=xankmax;
            obj.xankmin=xankmin;
            obj.yankmin=yankmin;
            obj.yankmax=yankmax;
        end
    end
end