m = 4.76;
lf = 0.35;
lr = 0.35;
cf = 150;
cr = 150;
Iz = 0.0687;
vx = 5;

A = [0  1   0   0 
     0      (-(cf + cr))/(m * vx)              (cf + cr)/(m)           ((lr * cr)-(lf*cf))/(m*vx);
     0      0              0           1;
     0 ((lr * cr)-(lf * cf))/(Iz * vx)       ((lf * cf)-(lr *cr))/(Iz)  (-((((lf)^2)*cf) + (((lr)^2)*cr)))/(Iz * vx)];
B = [     0;
     cf/m;
          0;
        (lf * cf)/Iz];
 
C = [0
    (((lr * cr) - (lf * cf)) / (m * vx)) - (vx)
    0
    (-(((lf * lf) * cf) + ((lr * lr) * cr))) / (Iz * vx)];
D = [0;
     0];
Q = [1 0 0 0
     0 0 0 0
     0 0 0 0
     0 0 0 0];
 R = 1;
 
 K = lqr(A,B,Q,R);
 disp(A);