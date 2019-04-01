function rAlpha=VHFtest_3(xScan,yScan,kt)
%kt is the anglediff between current position and target position
dsafe=6.11;
obstacle = zeros(length(xScan),2);
for i =1:length(xScan)
    obstacle(i,1:2)=[xScan(i) yScan(i)];
end
%step=rSpeed;
b=2.5;
a=1+b*(dsafe);
c=15;
f=15;
n=270/f;%each sector cover 5 degree
mag=zeros(n,1);
his=zeros(n,1); %histogram for each sector
threshold = 3000;%3500
smax=14;
smin=1;
rsafe=100;
for i=1:length(obstacle)-1
    d=sqrt(obstacle(i,1)^2+obstacle(i,2)^2);
    if(d<dsafe)
        m=c^2*(a-b*d);
        beta=i;
        rangle=round(rsafe/d);
        k=ceil(beta/f);%The number of sector
%         if ((f*k>beta-rad2deg(rangle))&&(f*k<beta+rad2deg(rangle)))
%             h=1;
%         else
%             h=0;
%         end
        mag(k)=max(mag(k),m);
    end
end    
q=1;
c=[];
while (q<=n)
    if (mag(q)<threshold)
        qr=q;
        while(q<=n&&mag(q)<threshold)
            ql=q;
            q=q+1;
        end
        if(ql-qr>=smax)
            if (kt<ql*f-136&&kt>qr*f-140)
                c(end+1)=kt;
            else
%                 c(end+1)=(ql*5-135+((ql+qr)/2)*5-139)/2;
%                 c(end+1)=(((ql+qr)/2)*5-135+qr*5-139)/2;
%                 c(end+1)=(ql*5-135+qr*5-139)/2;
                   c(end+1)=(ql-smax/2)*f-135;
                   c(end+1)=(qr+smax/2)*f-139;

            end
        elseif(ql-qr>=smin)
            c(end+1)=(ql*f-135+qr*f-135)/2;
        end
    else
        q=q+1;
    end
end
loss=[];
for i=1:length(c)
    loss(end+1)=f*deg2rad(abs(c(i)-kt));
    c(i)=deg2rad(c(i));
end
disp(c)
if ~isempty(find(c==kt))==1
    rAlpha=0;
elseif ~isempty(find(c>0&c<3.14/2))==1
    t=find(c>0);
    rAlpha=2.2*c(t(1));
else
    t=find(loss==min(loss));
    rAlpha=2.2*c(t(1));    
end
% t=find(loss==min(loss));
% rAlpha=2.2*c(t(1));

     
            

%disp(mag)
% rAlpha=0;
end

