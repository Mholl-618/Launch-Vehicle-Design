%Orbtial Parameter output script
    %With r and v input, outputs the 6 orbital parameters and relevant data
    %for cross checking hand calcs 
clear all, clc

r = [37.863 28.224+6378 0]
v = [2.9740 1.3723 0]

[semi_major,h, h_mag, r_mag, v_mag,e, e_mag, dot_vr,i,norm_vector,norm_mag,...
    ascending_node_longitude,n_dot_e,omega_calc,omega_final,e_dot_r,...
    calc_true_anomaly,true_anomaly,v_dot_r] = orbital_param(r,v)

peri = semi_major*(1-e_mag);
appo = semi_major*(1+e_mag);
