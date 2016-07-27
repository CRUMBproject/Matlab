function [theta1, theta2, theta3, theta4, theta5] = modelo_cd(x, y, z)
syms q1 q2 q3 q4 q5 L2 h2 L3 d2 px py pz ax ay az nx ny nz ox oy oz

%Para calcular modelo cinemático directo simbólico, comentar L3, d2, q y
%descomentar q simbólica.
%Para que simplifique la expresión final descomentar línea 55.
%Modelo cinemático inverso sólo útil para cálculos numéricos.

% L2=0.04825;
  L3=0.1423;
% L4=0.07585;
% h2=0.1450;
% d2=sqrt(L2^2+h2^2);
  d2=0.1528;
L45=0.07585+0.04625;
% beta=atan(h2/L2)
beta=1.2496;

alpha=[0 -pi/2 pi 0 pi/2];
a=[0 0 d2 L3 0];
d=[0 0 0 0 0];
  q=[q1 q2-beta q3-beta q4+pi/2 q5];
%q=[pi/8 (0)-beta (0)-beta (0)+(pi/2) 0];



i=1;
T01=[cos(q(i)) -sin(q(i)) 0 a(i);
    sin(q(i))*cos(alpha(i)) cos(q(i))*cos(alpha(i)) -0 -0*d(i);
    sin(q(i))*0 cos(q(i))*0 cos(alpha(i)) cos(alpha(i))*d(i);
    0 0 0 1];

i=2;
T12=[cos(q(i)) -sin(q(i)) 0 a(i);
    sin(q(i))*0 cos(q(i))*0 -sin(alpha(i)) -sin(alpha(i))*d(i);
    sin(q(i))*sin(alpha(i)) cos(q(i))*sin(alpha(i)) 0 0*d(i);
    0 0 0 1];
i=3;
T23=[cos(q(i)) -sin(q(i)) 0 a(i);
    sin(q(i))*cos(alpha(i)) cos(q(i))*cos(alpha(i)) -0 -0*d(i);
    sin(q(i))*0 cos(q(i))*0 cos(alpha(i)) cos(alpha(i))*d(i);
    0 0 0 1];
i=4;
T34=[cos(q(i)) -sin(q(i)) 0 a(i);
    sin(q(i))*cos(alpha(i)) cos(q(i))*cos(alpha(i)) -0 -0*d(i);
    sin(q(i))*0 cos(q(i))*0 cos(alpha(i)) cos(alpha(i))*d(i);
    0 0 0 1];
i=5;
T45=[cos(q(i)) -sin(q(i)) 0 a(i);
    sin(q(i))*0 cos(q(i))*0 -sin(alpha(i)) -sin(alpha(i))*d(i);
    sin(q(i))*sin(alpha(i)) cos(q(i))*sin(alpha(i)) 0 0*d(i);
    0 0 0 1];

T05b=T01*T12*T23*T34*T45
% T05=simplify(T05b)


%************************************************************************************
%Modelo cinemático inverso. (para valores numéricos)

%px=T05b(1,4);
%py=T05b(2,4);
%pz=T05b(3,4);


 px=x;
 py=y;
 pz=z;

ax=T05b(1,3);
ay=T05b(2,3);
az=T05b(3,3);

ox=T05b(1,2);
oy=T05b(2,2);
oz=T05b(3,2);

nx=T05b(1,1);
ny=T05b(2,1);
nz=T05b(3,1);

L3=0.1423;
d2=0.1528;

theta1=atan2(py,px)
 
k=-L3^2+pz^2+d2^2+(px*cos(theta1)+py*sin(theta1))^2;
k1=2*d2*px*cos(theta1)+2*py*d2*sin(theta1);
k2=2*pz*d2;

 
theta2b(1)=atan2(k1,k2)-atan2(k,sqrt(k1^2+k2^2-k^2));
theta2(1)=theta2b(1)+781/625;

theta2b(2)=atan2(k1,k2)-atan2(k,-sqrt(k1^2+k2^2-k^2));
theta2(2)=theta2b(2)+781/625

theta23=asin((-pz-d2*sin(theta2b))/L3);
theta3=theta2-theta23

L=az*cos(theta2-theta3)+ax*sin(theta2-theta3)*cos(theta1)+ay*sin(theta2-theta3)*sin(theta1);
theta4=acos(-L)-(pi/2)

theta5=asin(nx*sin(theta1) - ny*cos(theta1))

%Posición real respecto a 0

prx=px+L45*ax;
pry=py+L45*ay;
prz=pz+L45*az;

end