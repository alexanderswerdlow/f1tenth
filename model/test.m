m = 1140.0;
lf = 1.165;
lr = 1.165;
cf = 155494.663;
cr = 155494.663;
Iz = 1436.24;
vx = 5;

A = [0      (-(cf + cr))/(m * vx)              (cf + cr)/(m)           ((lr * cr)-(lf*cf))/(m*vx);
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 ((lr * cr)-(lf * cf))/(Iz * vx)       ((lf * cf)-(lr *cr))/(Iz)  (-((((lf)^2)*cf) + (((lr)^2)*cr)))/(Iz * vx)];
B = [     0;
     cf/m;
          0;
        (lf * cf)/Iz];
C = [0 0 0 0;
     0 0 0 0];
D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

poles = eig(A)

co = ctrb(sys_ss);
controllability = rank(co)

Q = [1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 1]
R = 1;
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')