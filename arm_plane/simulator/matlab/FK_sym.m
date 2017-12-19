function Q = FK_sym()

PhyPro;

syms q1 q2 q3 q4 q5 q6 q7
q = [q1 q2 q3 q4 q5 q6 q7];


T{1} = Tz(q(1)) * Tt([l2a; 0; l2b]);
T{2} = Ty(q(2)) * Tt([l3; 0; 0]);
T{3} = Tx(q(3)) * Tt([l4b; 0; -l4a]);
T{4} = Ty(q(4)) * Ty(pi/2) * Tt([l5; 0; 0]);
T{5} = Tx(q(5)) * Tt([l6b; 0; -l6a]);
T{6} = Ty(q(6)) * Tt([l7; 0; 0]);
T{7} = Tx(q(7)) * Tt([lEE; 0; 0]);

Q = Tt([l1x; l1y; l1z]);
for i = 1:7
    Q = Q * T{i}; 
%     P(i,:) = Q * [0;0;0;1];
end

% P = P(:,1:3);

Q = simplify(Q);
for i = 1:7
    Q = subs(Q, ['q' num2str(i)], ['q[' num2str(i-1) ']']);
end

end

function T = Tz(x)

T = [cos(x) -sin(x) 0 0; sin(x) cos(x) 0 0; 0 0 1 0; 0 0 0 1];

end

function T = Ty(x)

T = [cos(x) 0 sin(x) 0; 0 1 0 0; -sin(x) 0 cos(x) 0; 0 0 0 1];

end

function T = Tx(x)

T = [1 0 0 0; 0 cos(x) -sin(x) 0; 0 sin(x) cos(x) 0; 0 0 0 1];

end

function T = Tt(v)

T = [[eye(3) v]; 0 0 0 1];

end