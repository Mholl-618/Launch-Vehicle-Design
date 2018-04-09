%Orbtial Parameter output script
    %With r and v input, outputs the 6 orbital parameters and relevant data
    %for cross checking hand calcs 
clear all, clc

r = [1445.7 7027.5 0]
v = [8.593 2.172 0]

[semi_major,h, h_mag, r_mag, v_mag,e, e_mag, dot_vr,i,norm_vector,norm_mag,...
    ascending_node_longitude,n_dot_e,omega_calc,omega_final,e_dot_r,...
    calc_true_anomaly,true_anomaly,v_dot_r] = orbital_param(r,v)

peri = semi_major*(1-e_mag);
appo = semi_major*(1+e_mag);
