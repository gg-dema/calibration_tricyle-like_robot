syms x y theta; % state
syms delta_t;  % time
syms k_s k_t;  % k_steering, k_traction 
syms enc_s enc_t; % data from encoder  --> consider MAX VALUE inside
                  % in any case, it's a constant, so ...

syms base offset; % kinematic params

s = k_s * enc_s;
t = k_t * enc_t;

delta_theta = t * (sin(s)/base);
delta_x = (t * (sin(delta_theta))/delta_theta);
delta_y = t * (1 - cos(delta_theta) / delta_theta);
theta_p = theta + delta_t * (delta_theta); 
x_p = x + offset + delta_t * (delta_x);
y_p = y + delta_t * (delta_y);

jac = simplify(jacobian([delta_theta, delta_x, delta_y], [base, offset, k_s, k_t]))