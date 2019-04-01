function findcorridor(vScan,xScan,yScan)
Phi=-135*pi/180:pi/180:135*pi/180;
D=vScan';
NTh=270;
Delta=0.1;
[C,Th_axis,R_axis]=RWhough(Phi,D,NTh,Delta);
theta = [];
rho = [];
enx=[];
eny=[];
for l=1:length(Th_axis)
    for m=1:length(R_axis)
        if R_axis(m)>5 && C(m,l)>50
            C(m,l) = 0;
        end
    end
end
for k=1:2
    max_val=max(max(C));
    max_ind =find(C==max_val);
    [row,col]=ind2sub(size(C),max_ind-1);
    C(:,max(col-15,1):min(col+15,length(Th_axis)))=0;
    
    for i=1:length(row)
        PlotHoughLine(R_axis(row(i)), Th_axis(col(i)),'-r');
        theta(end+1) = Th_axis(col(i));
        rho(end+1) = R_axis(row(i));    
    end
end

for h = 1:length(theta) 
    linep=[];
    r=rho(h)
    th=theta(h)
    if r<15
        for j =1:271
            if round(yScan(j))==round((-cos(th)/sin(th))*xScan(j) + (r / sin(th)))
                linep(end+1) = j;
            end
        end
        for k = 2:numel(linep)
            if sqrt((yScan(linep(k))-yScan(linep(k-1)))^2+ (xScan(linep(k))-xScan(linep(k-1)))^2) >1.65 &&...
                    sqrt((yScan(linep(k))-yScan(linep(k-1)))^2+ (xScan(linep(k))-xScan(linep(k-1)))^2) <2.15
                enx(end+1)=(xScan(linep(k))+xScan(linep(k-1)))/2;
                eny(end+1)=(yScan(linep(k))+yScan(linep(k-1)))/2;
                m=(-cos(th)/sin(th));
                b=r / sin(th);
            end
        end
    end
end
if ~isempty(enx)==1
    plot(enx,eny,'*-')
end
end