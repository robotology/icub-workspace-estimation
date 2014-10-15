
r1 = r(:,1:3);
k = 1.5;

Xedge = linspace(min(r1(:,1)*k),max(r1(:,1)*k),32);
Xbin=Xedge(2)-Xedge(1);
Xedge=[Xedge(1)-3*Xbin,Xedge(1)-2*Xbin,Xedge(1)-Xbin, Xedge, Xedge(end)+Xbin, Xedge(end)+2*Xbin, Xedge(end)+3*Xbin];

Yedge = linspace(min(r1(:,2)*k),max(r1(:,2)*k),32);
Ybin=Yedge(2)-Yedge(1);
Yedge=[Yedge(1)-3*Ybin,Yedge(1)-2*Ybin,Yedge(1)-Ybin, Yedge, Yedge(end)+Ybin, Yedge(end)+2*Ybin, Yedge(end)+3*Ybin];

Zedge = linspace(min(r1(:,3)*k),max(r1(:,3)*k),16);
Zbin=Zedge(2)-Zedge(1);
Zedge=[Zedge(1)-3*Zbin,Zedge(1)-2*Zbin,Zedge(1)-Zbin, Zedge, Zedge(end)+Zbin, Zedge(end)+2*Zbin, Zedge(end)+3*Zbin];

[c] = histcn(r1,Xedge,Yedge,Zedge);
cs = smooth3(c,'gaussian',5);

isovalue=0.15;
figure(121)
% isosurface(cs,isovalue);
xlabel('X');
ylabel('Y');
zlabel('Z');
hiso = patch(isosurface(cs,isovalue),'FaceColor',rand(3,1),'EdgeAlpha',0.2,'FaceAlpha',0.6)