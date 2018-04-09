function [trans_matrix_321] = trans_inve_orbital(EulerAngle)
%tansforming from inertial to vehicle frame
cap_ohm = EulerAngle(1);
ohm = EulerAngle(2);
i = EulerAngle(3);
trans_matrix_321 = [ (cos(cap_ohm)*cos(ohm))-(sin(ohm)*cos(i)*cos(ohm)) (-cos(cap_ohm)*sin(ohm))-(sin(cap_ohm)*cos(i)*cos(ohm)) sin(cap_ohm)*sin(i)
                     (sin(cap_ohm)*cos(ohm))+(cos(cap_ohm)*cos(i)*cos(ohm)) (-sin(cap_ohm)*sin(ohm))+(cos(cap_ohm)*cos(i)*cos(ohm)) -cos(cap_ohm)*sin(i)
                     sin(i)*sin(ohm)                                         sin(i)*cos(ohm)                                       cos(i)             ]'
end