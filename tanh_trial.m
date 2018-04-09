            
x = 10e3:1:200e3;
percent = 25;
shape_val = (percent/100)*5;
var_pass = (tanh(shape_val-(shape_val*((150e3-x)/140e3))))*(pi/2); %final alt of 150
fpa = cos(var_pass);
plot(x,var_pass)