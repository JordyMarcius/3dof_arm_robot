%matlab script to calculate inverse jacobian

syms q1 q2 q3
syms L1 L2 L3

%forward kinematic
x = L2*cos(q1)*cos(q2) + L3*cos(q1)*cos(q2+q3);
y = L2*sin(q1)*cos(q2) + L3*sin(q1)*cos(q2+q3);
z = L1 + L2*sin(q2) + L3*sin(q2+q3);

J = [diff(x,q1) diff(x,q2) diff(x,q3);
    diff(y,q1) diff(y,q2) diff(y,q3);
    diff(z,q1) diff(z,q2) diff(z,q3)]

inv_J = inv(J)