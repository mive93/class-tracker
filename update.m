function [x_1, P_1] = update(x, P, z, R, H)

y = z - H*x
s = H*P*H' + R
k = P*H'/s
x_1 = x + k*y
P_1 = (eye(4)-K*H)*P

end

