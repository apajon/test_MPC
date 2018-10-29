%% Optimization QP

[QP_result_all{i},tata,converge_sampling(i)]=quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);

switch(COM_form)
    case 'com jerk'
        xdddc_storage=[xdddc_storage;QP_result_all{i}(1)];
        ydddc_storage=[ydddc_storage;QP_result_all{i}(size(A_zmp,2)/2+1)];
        zdddc_storage=[zdddc_storage;QP_result_all{i}(size(A_zmp,2)+1)];

        %% Results COM
        xc(i+1,1)=xf_c(1,:)+Pu_c(1,1)*xdddc_storage(end);
        xdc(i+1,1)=xf_dc(1,:)+Pu_dc(1,1)*xdddc_storage(end);
        xddc(i+1,1)=xf_ddc(1,:)+Pu_ddc(1,1)*xdddc_storage(end);

        yc(i+1,1)=yf_c(1,:)+Pu_c(1,1)*ydddc_storage(end);
        ydc(i+1,1)=yf_dc(1,:)+Pu_dc(1,1)*ydddc_storage(end);
        yddc(i+1,1)=yf_ddc(1,:)+Pu_ddc(1,1)*ydddc_storage(end);

        zc(i+1,1)=zf_c(1,:)+Pu_c(1,1)*zdddc_storage(end);
        zdc(i+1,1)=zf_dc(1,:)+Pu_dc(1,1)*zdddc_storage(end);
        zddc(i+1,1)=zf_ddc(1,:)+Pu_ddc(1,1)*zdddc_storage(end);


        %%
        if false
            run('script/script_display_online.m')
        end 
    case 'zmp vel'
        xdddc_storage=[xdddc_storage;QP_result_all{i}(1)];
        ydddc_storage=[ydddc_storage;QP_result_all{i}(size(A_zmp,2)/2+1)];
        zdddc_storage=[zdddc_storage;QP_result_all{i}(size(A_zmp,2)+1)];

        %%
        xc(i+1,1)=xf_c(1,:)+Pu_c(1,1)*xdddc_storage(end);
        xdc(i+1,1)=xf_dc(1,:)+Pu_dc(1,1)*xdddc_storage(end);
        xddc(i+1,1)=xf_ddc(1,:)+Pu_ddc(1,1)*xdddc_storage(end);

        yc(i+1,1)=yf_c(1,:)+Pu_c(1,1)*ydddc_storage(end);
        ydc(i+1,1)=yf_dc(1,:)+Pu_dc(1,1)*ydddc_storage(end);
        yddc(i+1,1)=yf_ddc(1,:)+Pu_ddc(1,1)*ydddc_storage(end);

        zc(i+1,1)=zf_c(1,:)+Pu_c(1,1)*zdddc_storage(end);
        zdc(i+1,1)=zf_dc(1,:)+Pu_dc(1,1)*zdddc_storage(end);
        zddc(i+1,1)=zf_ddc(1,:)+Pu_ddc(1,1)*zdddc_storage(end);

end    

if (phase_type_sampling_reduce(1)~='b'&&phase_type_sampling_reduce(1)~="start"&&phase_type_sampling_reduce(1)~="stop") && (phase_type_sampling_reduce(2)=='b'||phase_type_sampling_reduce(2)=="start"||phase_type_sampling_reduce(2)=="stop")
    xstep=[xstep;QP_result_all{i}(N+1)];
    ystep=[ystep;QP_result_all{i}(size(A_zmp,2)/2+N+1)];
    zstep=[zstep;zzmp_ref_reduce(3)];
end  