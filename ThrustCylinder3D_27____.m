clc; clear all;

timerange1 = [0 20];
ic_11 = pi*80/180;
IC = [0;0;0; 0;0;8.858267007012898e-06; 0;0;0; 0;ic_11;0];
opts = odeset('RelTol',1e-2, 'AbsTol',1e-2);

% T=time, FT=Thrust
T = xlsread('jng_n_2021_2.xlsx','A2:A22');
FT= xlsread('jng_n_2021_2.xlsx','B2:B22');
SetY= xlsread('jng_n_2021_2.xlsx','C2:C22');
SetP= xlsread('jng_n_2021_2.xlsx','E2:E22');
SetR= xlsread('jng_n_2021_2.xlsx','G2:G22');

[t,x] = ode45(@(t,x) rockt_1(t,x,T,FT,SetY,SetP,SetR),timerange1,IC,opts);
x;

subplot(2,3,1)
plot(t,x(:,1),T,FT,'LineWidth',2);
grid on;
legend(["x","Thrust"])
xlabel('Time (s)')
ylabel('x (m)')
title('Time - x')

subplot(2,3,2)
plot(t,x(:,2),T,FT,'LineWidth',2);
grid on; 
legend(["y","Thrust"]);
xlabel('Time (s)')
ylabel('y (m)')
title('Time - y')

subplot(2,3,3)
plot(t,x(:,3),T,FT,'LineWidth',2);
grid on; 
legend(["z","Thrust"]);
xlabel('Time (s)')
ylabel('z (m)')
title('Time - z')

subplot(2,3,4)
plot(t,rad2deg(x(:,10)),t,rad2deg(x(:,11)),t,rad2deg(x(:,12)),'LineWidth',2);
grid on; 
legend(["yaw","pitch","roll"]);
xlabel('Time (s)')
ylabel('Euler(rad)')
title('Time - Euler')

subplot(2,3,5)
plot(t,x(:,7),t,x(:,8),t,x(:,9),'LineWidth',2);
grid on; 
legend(["p","q","r"]);
xlabel('Time (s)')
ylabel('ê°?ì†?ë?„(rad/s)')
title('Time - ê°?ì†?ë?„')

 function dx = rockt_1(t,x,T,FT,SetY,SetP,SetR)
 
    m = 1; Length = 1; r=0.04; d=0.08;
    I_xx = (m*r^2)/2;
    I_yy = (m*Length^2)/12 + (m*r^2)/4;
    I_zz = (m*Length^2)/12 + (m*r^2)/4;
    
    FT = interp1(T,FT,t);
    SetY = interp1(T,SetY,t);
    SetP = interp1(T,SetP,t);
    SetR = interp1(T,SetR,t);
    
%     AOA
    rckt_x = [1 0 0];
    
    Rel_Wind = [-x(4) -x(5) -x(6)]; % Relative Wind
    rel_wind = Rel_Wind / norm(Rel_Wind); % Unit Vector of Relative Wind
    
    RW_yz = [ -x(5); -x(6) ];
    rw_yz = RW_yz / norm(RW_yz);
    
    AOA = acos(dot(rckt_x,rel_wind)/1);
    aoa=rad2deg(AOA);
    
%     Lift Coefficient  
    if aoa < 20
        CL=0.04*(aoa);
        
    elseif aoa > 340
        CL=0.04*(aoa)-14.4;
        
    else
        CL=-0.005*(aoa)+0.9; 
        
    end

%     Drag Coefficient
    if aoa < 90
        CD=0.78/90^2*(aoa^2)+0.5;
        
    elseif aoa > 270
        CD=0.78/90^2*(aoa-360)^2+0.5;

    else
        CD=1.08/90^2*(aoa-180)^2+0.2;
    end    

%     Pitching Moment Coefficient
    CM=0.0455*(CL)^2-0.2408*CL-0.0023; %Const.=0
    
    rho = 1.2; A = 0.005; g = 9.81;
    W = m*g;
    D = 0.5*CD*rho*A*(x(4)^2+x(5)^2+x(6)^2);
    L = 0.5*CL*rho*A*(x(4)^2+x(5)^2+x(6)^2);
    M_p = 0.5*CM*rho*A*(x(4)^2+x(5)^2+x(6)^2)*d;
    
%     Unit Vector of Lift Direction 
    N_vctr = cross(rel_wind, rckt_x); % Direction of Normal Vector of plane
    n_vctr = N_vctr / norm(N_vctr);
   
    Dir_lift = cross(n_vctr,rel_wind);
    dir_lift = Dir_lift / norm(Dir_lift);
   
%     Body-fixed Coordinate and Earth-fixed Coordinate
    CONV =  [1 0 0;
            0 -1 0;
            0 0 -1];
    Psi = [cos(x(10)) -sin(x(10)) 0;
           sin(x(10)) cos(x(10)) 0;
           0 0 1];
    Theta = [cos(x(11)) 0 sin(x(11));
             0 1 0;
             -sin(x(11)) 0 cos(x(11))];
    Phi = [1 0 0;
           0 cos(x(12)) -sin(x(12));
           0 sin(x(12)) cos(x(12))];
% from Earth-fixed Coordinate to Body-fixed Coordinate
    EtoB = CONV*Psi*Theta*Phi;
% from Body-fixed Coordinate to Earth-fixed Coordinate
    BtoE = inv(Phi)*inv(Theta)*inv(Psi)*CONV; %inv(CONV)=CONV
    
    
%  Earth) x(1)=x x(2)=y x(3)=z     Body) x(4)=U x(5)=V x(6)=W
        v_E = BtoE*[x(4); x(5); x(6)]
        dx(1) = v_E(1);
        dx(2) = v_E(2);
        dx(3) = v_E(3);
        
        W_b = EtoB * [0; 0; -W]; % Weight ~ Fixed to Body
        dx(4) = (W_b(1) + FT + D*rel_wind(:,1) + L*dir_lift(:,1))/m;
        dx(5) = (W_b(2) + D*rel_wind(:,2) + L*dir_lift(:,2))/m;
        dx(6) = (W_b(3) + D*rel_wind(:,3) + L*dir_lift(:,3))/m;
        
%         moment
        M_xtnl = [0; 0; 0]
%                   M_p * rw_yz(1) ;
%                   M_p * rw_yz(2)]; %Lift Drag 
              
        K_p=[0; 0.03; 0]; %control
        M_x= -K_p(1)*(x(10)-SetR);
        M_y= -K_p(2)*(x(11)-SetP);
        M_z= -K_p(3)*(x(12)-SetY);
        
%         x(7)=p; x(8)=q; x(9)=r;
        dx(7) = (M_x + M_xtnl(1) + (I_yy-I_zz)*x(8)*x(9)) / I_xx ; 
        dx(8) = (M_y + M_xtnl(2) + (I_zz-I_xx)*x(9)*x(7)) / I_yy ;
        dx(9) = (M_z + M_xtnl(3) + (I_xx-I_yy)*x(7)*x(8)) / I_zz ;

%         x(10)=psi(yaw), x(11)=theta(pitch), x(12)=phi(roll)
        dotEuler = [1 sin(x(12))*tan(x(11)) cos(x(12))*tan(x(11)); ...
            0 cos(x(12)) -sin(x(12)); 0 sin(x(12))*sec(x(11)) cos(x(10))*sec(x(11))]...
            * [x(7); x(8); x(9)];
        dx(10)=dotEuler(1);
        dx(11)=dotEuler(2);
        dx(12)=dotEuler(3);
        
        dx = dx(:);

 end
 
 %aoa
 %v
 %dot_q = -K_p(theta-Theta_set)/Iyy
 %dot_theta = q
 %lift drag moment
 %v_z control
 %