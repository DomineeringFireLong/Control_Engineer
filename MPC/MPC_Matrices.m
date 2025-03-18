function  [E , H]=MPC_Matrices(A,B,Q,R,F,N)%模型预测,根据当前状态和模型，以及性能指标转化为优化问题

n=size(A,1);   % A 是 n x n 矩阵, 得到 n
p=size(B,2);   % B 是 n x p 矩阵, 得到 p
M=[eye(n);zeros(N*n,n)]; % 初始化 M 矩阵. M 矩阵是 (N+1)n x n的， 
                         % 它上面是 n x n 个 "I", 这一步先把下半部
                         % 分写成 0 
C=zeros((N+1)*n,N*p); % 初始化 C 矩阵, 这一步令它有 (N+1)n x NP 个 0
% 定义M 和 C 
tmp=eye(n);  %定义一个n x n 的 I 矩阵
%　更新Ｍ和C,用x(k)和uk...uk+N来估计x(k--k+N|k)向量
for i=1:N % 循环，i 从 1到 N
    rows =i*n+(1:n); %定义当前行数，从i x n开始，共n行 
    C(rows,:)=[tmp*B,C(rows-n, 1:end-p)]; %将c矩阵填满[[0,...],[b,...],[ab,b,..]...,[aN-1b,...,b]]
    tmp= A*tmp; %每一次将tmp左乘一次A  
    M(rows,:)=tmp; %将M矩阵写满 [I,A,A2,A3,..AN-1]T
end 
% 定义Q_bar和R_bar
Q_bar = kron(eye(N),Q);%大块的Q，由小块Q对角构成
Q_bar = blkdiag(Q_bar,F);
R_bar = kron(eye(N),R); 

% 计算G, E, H 性能指标函数化简后的指标函数
G=M'*Q_bar*M; % G: n x n    e^2
E=C'*Q_bar*M; % E: NP x n   多出来的交叉项
H=C'*Q_bar*C+R_bar; % NP x NP  u^2
%小块指标j->模型->大块指标J=x(k)'*G*x(k)+U(k)'*H*U(k)+2*x(k)'*E*U(k);

end