function [vPose]=kalmanfilter(rSpeed,rOmega,kPose,n,dt,vOmegaSpeed)
theta=kPose(3,n-1);
% vPose = kPose(:,n-1)+dt*[cos(theta+dt*rOmega/2) 0;sin(theta+dt*rOmega/2) 0;0 1]*[rSpeed;rOmega];
vPose(1)= kPose(1,n-1)+dt*[cos(theta+dt*rOmega/2) 0]*[rSpeed;rOmega];
vPose(2)= kPose(2,n-1)+dt*[sin(theta+dt*rOmega/2) 0]*[rSpeed;rOmega];
vPose(3)= kPose(3,n-1)+dt*[0 1]*[rSpeed;rOmega];
F=[1 dt;0 1];   % add state transition matrix
G=[dt^2/2;dt];    % Add input vector
GQ=[std(vOmegaSpeed(1,:))];
% change noise on Q
Q=  [dt^3 0;
    0 dt^3];
Xe=[theta 0]';
Pe=eye(2)*0.9^2;
U=rOmega;
Xp=F*Xe+G*U;
Pp=F*Pe*F'+G*GQ*G'+Q;
%Pp=F*Pe*F'+Q;
H=[1 0];               % Add observation model
Z=[1];
R=0.0109;
bAccept=0;
Xp=F*Xe+G*U;
Pp=F*Pe*F'+G*GQ*G'+Q;
%Pp=F*Pe*F'+Q;
Zp=H*Xp;
Z=vPose(3);
vInno=Z-H*Xp;
S=(H*Pp*H'+R);
invS=inv(S);
d_sqr = vInno'*invS*vInno;
bAccept=or(bAccept,d_sqr<5.991);% add expression to accept observation
if(bAccept>0)
    K=Pp*H'.*invS;                  %  add equations for accept
    Xe=Xp+K*vInno;
    I=eye(2);
    Pe=(I-K*H)*Pp*(I-K*H)' + K*R*K';
else
    Xe=Xp;
    Pe=Pp;
end
vPose(3)=Xe(1);
end

    