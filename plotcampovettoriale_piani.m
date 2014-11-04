[x,y]=meshgrid(-5:0.1:5,-5:0.1:5);

[X,Y,Z]=meshgrid(-10:2.5:10);


z=x+y;
z1=-2*x+y;

E1= -(X+Y-Z);
E2= -(-2*X+Y-Z);


dx1=1+0*X;
dy1=1+0*Y;
dz1=-1+0*Z;

dx2=-2+0*X;
dy2=1+0*Y;
dz2=-1+0*Z;

dx=(E1.*dx1+E2.*dx2);
dy=(E1.*dy1+E2.*dy2);
dz=(E1.*dz1+E2.*dz2);


hold on;
quiver3(X,Y,Z,dx,dy,dz);
mesh(x,y,z);
mesh(x,y,z1);
%mesh(x,-y,-z2);