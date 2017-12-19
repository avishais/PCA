R = BaxterKinematics_B1();

T = [0.964367 0.264522 0.00502167 242.674
    -0.208123 0.74676 0.631692 -221.033
    0.163346 -0.610228 0.775203 22.0178
    0 0 0 1 ];

Q = zeros(14,1);
Q(1:7) = [ -0.0440872, -1.14723, -0.45213, 1.03529, -2.48689, -0.26732, -1.78304  ];
% Q(2) = -1;

% Q(1) = -pi/4;
% Q(2) = pi/2;
% Q(3) = pi/6;
% Q(4) = -pi/5;
% Q(5) = pi/3;
% Q(6) = 1;
% Q(7) = -1.2;
% Q(1:7) = [1.31652 0.86554 -2.62739 -2.53096 -0.110503 1.87921 -0.779376];
% 
% Q(4+7) = -pi/2;
% Q(1+7) = deg2rad(0);
% Q(2+7) = deg2rad(-0);
% Q(3+7) = deg2rad(-0);
% Q(4+7) = deg2rad(-0);
% Q(5+7) = deg2rad(-0);
% Q(6+7) = deg2rad(-0);
% Q(7+7) = deg2rad(-0);


T1 = R.FK(Q(1:7),1);
% T2 = R.FK(Q(1:7),2)
T1(1:3,4)*1e-3
R.print2file(Q);

% P = FK_sym(Q(1:7))
% 
% disp([atan2(P(2,1),P(1,1)) atan2(T1(2,1),T1(1,1)) ]);

% T = BaxterFK(rad2deg(Q(1:7)));
% T{8};

%%
% R = BaxterKinematics_B1();
% 
% TL = [eye(3) [200;0;0]; 0 0 0 1];
% 
% q0 = rand(14,1)*2*pi - pi;
% % q = GDproject(q0, TL);
% q = Csolve(q0, TL);
% R.print2file(q);
% %%
% To = [-1, 0, 0, 0;
%     0, -1, 0, 0;
%     0, 0, 1, 0;
%     0, 0, 0, 1];
% T1 = R.FK(q(1:7),1)
% T2 = R.FK(q(8:14),2)