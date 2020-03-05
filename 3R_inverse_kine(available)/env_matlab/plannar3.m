classdef plannar3
    %PLANNAR3 三自由度平面机械臂
    properties
        %默认平面机械臂参数
        N=3;                   %连杆数
        base=[300,300];        %基座坐标
        link1=150;
        link2=80;
        link3=70;             %连杆长度
        qlim1=[-pi/2,pi/2];
        qlim2=[-pi/2,pi/2];
        qlim3=[-pi/2,pi/2];
        out=readfis('correct');
        ToRad=pi/180.0;
    end
    
    methods
        function obj = plannar3(varargin)
            if length(varargin)>0
                for i=1:1:length(varargin)
                    switch inputname(i)
                        case 'link1'
                            obj.link1=varargin{i};
                        case 'link2'
                            obj.link2=varargin{i};
                        case 'link3'
                            obj.link3=varargin{i};
                        case 'qlim1'
                            obj.qlim1=varargin{i};
                        case 'qlim2'
                            obj.qlim2=varargin{i};
                        case 'qlim3'
                            obj.qlim2=varargin{i};
                        case 'base'
                            obj.base=varargin{i};   
                    end
                end
            end
        end
        
        function h =plot(obj,th,clf)
            h=hggroup;
%             ax=gca;
%             ax.YLim = [0 600];
%             ax.XLim = [0 600];
            grid on;
            hold on;
            axis([0 600 0 600]);
            %正运动学
            T01=[1,0,obj.base(1);0,1,obj.base(2);0,0,1]*[cos(th(1)),-sin(th(1)),0;
                sin(th(1)),cos(th(1)),0;
                0,0,1]*[1,0,obj.link1;0,1,0;0,0,1];
            T12=T01*[cos(th(2)),-sin(th(2)),0;
                sin(th(2)),cos(th(2)),0;
                0,0,1]*[1,0,obj.link2;0,1,0;0,0,1];
            T23=T12*[cos(th(3)),-sin(th(3)),0;
                sin(th(3)),cos(th(3)),0;
                0,0,1]*[1,0,obj.link3;0,1,0;0,0,1];
            scatter(obj.base(1),obj.base(2),100,[1 0 0],'filled','parent',h);
            scatter(T01(1,3),T01(2,3),100,[0 1 0],'filled','parent',h);
            scatter(T12(1,3),T12(2,3),100,[0.1 0.5 0.3],'filled','parent',h);
            scatter(T23(1,3),T23(2,3),100,[0 0 0],'filled','parent',h);
            g=plot([obj.base(1) T01(1,3)],[obj.base(2) T01(2,3)],'b','parent',h);
            set(g,'LineWidth',2);
            g=plot([T01(1,3) T12(1,3)],[T01(2,3) T12(2,3)],'b','parent',h);
            set(g,'LineWidth',2);
            g=plot([T12(1,3) T23(1,3)],[T12(2,3) T23(2,3)],'b','parent',h);
            set(g,'LineWidth',2);
            
%             drawnow;
            if clf
                delete(h);
            end
        end
        
        function xy = fk(obj,th)
            %正运动学
            T01=[1,0,obj.base(1);0,1,obj.base(2);0,0,1]*[cos(th(1)),-sin(th(1)),0;
                sin(th(1)),cos(th(1)),0;
                0,0,1]*[1,0,obj.link1;0,1,0;0,0,1];
            T12=T01*[cos(th(2)),-sin(th(2)),0;
                sin(th(2)),cos(th(2)),0;
                0,0,1]*[1,0,obj.link2;0,1,0;0,0,1];
            T23=T12*[cos(th(3)),-sin(th(3)),0;
                sin(th(3)),cos(th(3)),0;
                0,0,1]*[1,0,obj.link3;0,1,0;0,0,1];
            xy(1)=T23(1,3);
            xy(2)=T23(2,3);
        end
        
        function th=GA(obj,x,y)  %遗传算法求逆解
%             p=[th1,th2,th3]
            size=150;       %100个个体 种群大小
            G=150;          %繁衍150代
            codeL=10;       %基因长度
            %角度范围
            th_max=[obj.qlim1(2),obj.qlim2(2),obj.qlim3(2)];
            th_min=[obj.qlim1(1),obj.qlim2(1),obj.qlim3(1)];
            
            E=round(rand(size,3*codeL));    %Initial Code
            
            for k=1:1:G
                
                for s=1:1:size
                    m=E(s,:);
                    th1=0;th2=0;th3=0;
                    %解码二进制
                    m1=m(1:1:codeL);
                    for i=1:1:codeL
                        th1=th1+m1(i)*2^(i-1);
                    end
                    th1=(th_max(1)-th_min(1))*th1/1023+th_min(1);
                    m2=m(codeL+1:1:2*codeL);
                    for i=1:1:codeL
                        th2=th2+m2(i)*2^(i-1);
                    end
                    th2=(th_max(2)-th_min(2))*th2/1023+th_min(2);
                    m3=m(2*codeL+1:1:3*codeL);
                    for i=1:1:codeL
                        th3=th3+m3(i)*2^(i-1);
                    end
                    th3=(th_max(3)-th_min(3))*th3/1023+th_min(3);
                    %适应度函数
                    F(s)=exp(norm([x,y]-fk(obj,[th1,th2,th3]))/norm(obj.base-fk(obj,[th1,th2,th3])));
                    %越小越好
                end
                
                %****** Step 1 : Evaluate BestJ ******
                [Oderfi,Indexfi]=sort(F);     %Arranging f small to bigger
                Bestfi=Oderfi(1);           %Let Bestfi=max(fi)
                BestS=E(Indexfi(1),:);      %Let BestS=E(m), m is the Indexfi belong to max(fi)
                %****** Step 2 : Select and Reproduct Operation******
                fi_sum=sum(Oderfi);
                fi_size=((fi_sum-Oderfi)/fi_sum).*size;
                fi_s=floor(fi_size);        %Selecting Bigger fi value
                kk=1;
                for i=1:1:size
                    for j=1:1:fi_s(i)
                        TempE(kk,:)=E(Indexfi(i),:);
                        kk=kk+1;
                    end
                end
                %************ Step 3 : Crossover Operation ************
                pc=0.6;
                n=ceil(30*rand);
                for i=1:2:(size-1)
                    temp=rand;
                    if pc>temp
                        for j=n:1:30
                            TempE(i,j)=E(i+1,j);
                            TempE(i+1,j)=E(i,j);
                        end
                    end
                end
                TempE(1,:)=BestS;
                
                %************ Step 4: Mutation Operation **************
                pm=0.1;     %Big mutation
                for i=1:1:size
                    for j=1:1:3*codeL
                        temp=rand;
                        if pm>temp                %Mutation Condition
                            if TempE(i,j)==0
                                TempE(i,j)=1;
                            else
                                TempE(i,j)=0;
                            end
                        end
                    end
                end
                TempE(1,:)=BestS;
                E=TempE;  
            end
            m=E(1,:);
            th1=0;th2=0;th3=0;
                    %解码二进制
                    m1=m(1:1:codeL);
                    for i=1:1:codeL
                        th1=th1+m1(i)*2^(i-1);
                    end
                    th1=(th_max(1)-th_min(1))*th1/1023+th_min(1);
                    m2=m(codeL+1:1:2*codeL);
                    for i=1:1:codeL
                        th2=th2+m2(i)*2^(i-1);
                    end
                    th2=(th_max(2)-th_min(2))*th2/1023+th_min(2);
                    m3=m(2*codeL+1:1:3*codeL);
                    for i=1:1:codeL
                        th3=th3+m3(i)*2^(i-1);
                    end
                    th3=(th_max(3)-th_min(3))*th3/1023+th_min(3);
            th=[th1,th2,th3];
        end   
        
        function dth=GAdth(obj,x,y,th0)
%             p=[th1,th2,th3]
            ToRad=pi/180.0;
            size=100;       %100个个体 种群大小
            G=150;          %繁衍150代
            codeL=10;       %基因长度
            %[x,x,x,x,...x|x,x,x,x,...,x|x,x,x,x,...,x]
            %对于每一段基因，第一位代表正负，1正0负，后九位二进制代表角度
            E=round(rand(size,3*codeL));    %Initial Code
            
            for k=1:1:G
                for s=1:1:size
                    dth1=0;dth2=0;dth3=0;
                    m=E(s,:);
                    %解码二进制
                    m1=m(1:1:codeL);
                    if m1(1)==1
                        sign=1;
                    else
                        sign=-1;
                    end
                    for i=2:1:codeL
                        dth1=dth1+m1(i)*2^(3-i)*ToRad;
                    end
                    dth1=sign*dth1;
                    m2=m(codeL+1:1:2*codeL);
                    if m2(1)==1
                        sign=1;
                    else
                        sign=-1;
                    end
                    for i=2:1:codeL
                        dth2=dth2+m2(i)*2^(3-i)*ToRad;
                    end
                    dth2=sign*dth2;
                    m3=m(2*codeL+1:1:3*codeL);
                    if m3(1)==1
                        sign=1;
                    else
                        sign=-1;
                    end
                    for i=2:1:codeL
                        dth3=dth3+m3(i)*2^(3-i)*ToRad;
                    end
                    dth3=sign*dth3;
                    %适应度函数
                    F(s)=exp(norm([x,y]-fk(obj,th0+[dth1,dth2,dth3])));
                    %越小越好
                end
                
                %****** Step 1 : Evaluate BestJ ******
                [Oderfi,Indexfi]=sort(F);     %Arranging f small to bigger
                Bestfi=Oderfi(1);           %Let Bestfi=max(fi)
                BestS=E(Indexfi(1),:);      %Let BestS=E(m), m is the Indexfi belong to max(fi)
                %****** Step 2 : Select and Reproduct Operation******
                fi_sum=sum(Oderfi);
                fi_size=((fi_sum-Oderfi)/fi_sum).*size;
                fi_s=floor(fi_size);        %Selecting Bigger fi value
                kk=1;
                for i=1:1:size
                    for j=1:1:fi_s(i)
                        TempE(kk,:)=E(Indexfi(i),:);
                        kk=kk+1;
                    end
                end
                %************ Step 3 : Crossover Operation ************
                pc=0.6;
                n=ceil(30*rand);
                for i=1:2:(size-1)
                    temp=rand;
                    if pc>temp
                        for j=n:1:30
                            TempE(i,j)=E(i+1,j);
                            TempE(i+1,j)=E(i,j);
                        end
                    end
                end
                TempE(1,:)=BestS;
                
                %************ Step 4: Mutation Operation **************
                pm=0.1;     %Big mutation
                for i=1:1:size
                    for j=1:1:3*codeL
                        temp=rand;
                        if pm>temp                %Mutation Condition
                            if TempE(i,j)==0
                                TempE(i,j)=1;
                            else
                                TempE(i,j)=0;
                            end
                        end
                    end
                end
                TempE(1,:)=BestS;
                E=TempE;  
            end
            m=BestS;
            dth1=0;dth2=0;dth3=0;
            %解码二进制
                    m1=m(1:1:codeL);
                    if m1(1)==1
                        sign=1;
                    else
                        sign=-1;
                    end
                    for i=2:1:codeL
                        dth1=dth1+m1(i)*2^(3-i)*ToRad;
                    end
                    dth1=sign*dth1;
                    m2=m(codeL+1:1:2*codeL);
                    if m2(1)==1
                        sign=1;
                    else
                        sign=-1;
                    end
                    for i=2:1:codeL
                        dth2=dth2+m2(i)*2^(3-i)*ToRad;
                    end
                    dth2=sign*dth2;
                    m3=m(2*codeL+1:1:3*codeL);
                    if m3(1)==1
                        sign=1;
                    else
                        sign=-1;
                    end
                    for i=2:1:codeL
                        dth3=dth3+m3(i)*2^(3-i)*ToRad;
                    end
                    dth3=sign*dth3;
           dth=[dth1,dth2,dth3];        
        end   
       
        function dth=fuzzydth(obj,x,y,th,f)
            T01=[1,0,obj.base(1);0,1,obj.base(2);0,0,1]*[cos(th(1)),-sin(th(1)),0;
                sin(th(1)),cos(th(1)),0;
                0,0,1]*[1,0,obj.link1;0,1,0;0,0,1];
            T12=T01*[cos(th(2)),-sin(th(2)),0;
                sin(th(2)),cos(th(2)),0;
                0,0,1]*[1,0,obj.link2;0,1,0;0,0,1];
            T23=T12*[cos(th(3)),-sin(th(3)),0;
                sin(th(3)),cos(th(3)),0;
                0,0,1]*[1,0,obj.link3;0,1,0;0,0,1];
            %校正第三关节
            xy=fk(obj,th);
            error=norm(xy-[x,y]);
            mag=f(round(xy(1)),round(xy(2)));
            xy_1=fk(obj,[th(1) th(2) th(3)+3.0*obj.ToRad]);
            mag_1=f(round(xy_1(1)),round(xy_1(2)));
            dm=mag_1-mag;
            dth1=evalfis([-dm,mag,error],obj.out);
        
                
            %校正第二关节
            xy=[T12(1,3),T12(2,3)];
            mag=f(round(xy(1)),round(xy(2)))
            T12=T01*[cos(th(2)+3.0*obj.ToRad),-sin(th(2)+3.0*obj.ToRad),0;
                sin(th(2)+3.0*obj.ToRad),cos(th(2)+3.0*obj.ToRad),0;
                0,0,1]*[1,0,obj.link2;0,1,0;0,0,1];
            mag_1=f(round(T12(1,3)),round(T12(2,3)));
            dm=mag_1-mag;
            dth2=evalfis([-dm,mag,error],obj.out);
      
            %校正第一关节
            xy=[T01(1,3),T01(2,3)];
            mag=f(round(xy(1)),round(xy(2)))
            T01=[1,0,obj.base(1);0,1,obj.base(2);0,0,1]*[cos(th(1)+3.0*obj.ToRad),-sin(th(1)+3.0*obj.ToRad),0;
                sin(th(1)+3.0*obj.ToRad),cos(th(1)+3.0*obj.ToRad),0;
                0,0,1]*[1,0,obj.link1;0,1,0;0,0,1];
            mag_1=f(round(T01(1,3)),round(T01(2,3)));
            dm=mag_1-mag;
            dth3=evalfis([-dm,mag,error],obj.out);
     
            
            dth=[dth1,dth2,dth3]/5; 
            
        end
        
        function xy=get_dist(obj,th)
            %正运动学
            xy=zeros(2,3);
            T01=[1,0,obj.base(1);0,1,obj.base(2);0,0,1]*[cos(th(1)),-sin(th(1)),0;
                sin(th(1)),cos(th(1)),0;
                0,0,1]*[1,0,obj.link1;0,1,0;0,0,1];
            
            xy(:,1)=T01(1:2,3);
            T12=T01*[cos(th(2)),-sin(th(2)),0;
                sin(th(2)),cos(th(2)),0;
                0,0,1]*[1,0,obj.link2;0,1,0;0,0,1];
            xy(:,2)=T12(1:2,3);
            T23=T12*[cos(th(3)),-sin(th(3)),0;
                sin(th(3)),cos(th(3)),0;
                0,0,1]*[1,0,obj.link3;0,1,0;0,0,1];
            xy(:,3)=T23(1:2,3);
        end
            
   end
end


