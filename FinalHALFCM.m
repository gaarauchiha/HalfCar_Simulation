%% Parameters
%We will start by defining the parameters given in the word file to build
%our vehicle model
% Vehicle
ms   = 1794/2;    % Sprung mass                         
m_muf  = 87/2;       % Front unsprung mass                
m_mur  = 140/2;    % Rear unsprung mass                 
Is  = 3443;     % Moment of inertia of sprung mass  
a  = 1.271;      % Distance from centre of sprung mass to front axle                 
b  = 1.716;     % Distance from centre of sprung mass to rear axle               
kf  = 66824;    % Front spring constant             
kr  = 18615;    % Rear spring constant               
kt = 101115;   % Tyre spring constant                    
cf  = 1190;    % Front damping constant              
cr  = 1000;    % Rear damping constant             
% For animating our model
z_sstat = 1.6;   % Sprung mass static vertical position      [m]
z_ustat = 0.7;   % Unsprung mass static vertical position    [m]
h_s = 0.1;          % Sprung block height                       [m]
w_u = 0.3;          % Unsprung block width                      [m]
h_u = 0.1;          % Unsprung block height                     [m]
w_e = 0.2;          % Unsprung width extension                  [m]
l_win        = 5;   % Length window analysis                    [m]
l_win_maring = 0.3; % Length window margin at the begining      [m]
% Video
playback_speed = 0.1;               % Speed of playback
tF      = 1;                        % Final time                [s]
fR      = 30/playback_speed;        % Frame rate                [fps]
dt      = 1/fR;                     % Time resolution           [s]
time    = linspace(0,tF,tF*fR);     % Time                      [s]
%% Road
% Stretch 1
x_r_1_total = 5;        
dx_r_1 = 0.1;                                        
x_r_1 = 0:dx_r_1:x_r_1_total;
z_r_1 = zeros(1,length(x_r_1));
% Stretch 2
R_r = 0.2;             
th_r = 0:0.01:pi;
x_r_2 = -R_r*cos(th_r) + x_r_1_total+R_r;
z_r_2 = R_r*sin(th_r);
% Stretch 3
x_r_3_total = 15;      
dx_r_2 = 0.1;           
x_r_3 = x_r_1_total+2*R_r:dx_r_2:x_r_1_total+2*R_r+x_r_3_total;
z_r_3 = zeros(1,length(x_r_3));
% Concatenating 
X_r = [x_r_1 x_r_2(2:end) x_r_3(2:end)];
Z_r = [z_r_1 z_r_2(2:end) z_r_3(2:end)];
figure
hold on ; box on ; grid on ; axis equal
plot(x_r_1,z_r_1)
plot(x_r_2,z_r_2)
plot(x_r_3,z_r_3)
xlabel('Distance x [m]')
ylabel('Distance z [m]')
legend('Stretch 1','Stretch 2','Stretch 3')
title('Input')
figure
hold on ; box on ; grid on ; axis equal
plot(X_r,Z_r,'k','LineWidth',2)
xlabel('Distance x [m]')
ylabel('Distance z [m]')
title('Input')
%% Simulation
% z     : Body vertical motion coordinate
% th    : Body pitch motion coordinate
% xuf   : Front wheel vertical motion coordinate 
% xur   : Rear wheel vertical motion coordinate
% Mx'' + Cx' + K x = F u
M = [   ms   0   0   0   ;
        0   Is  0   0   ;
        0   0   m_muf  0   ;
        0   0   0   m_mur  ];
C = [   cf+cr           b*cr-a*cf         -cf         -cr     ;
        b*cr-a*cr     cf*a^2+cr*b^2     a*cf       -b*cr  ;
        -cf             a*cf               cf          0       ;
        -cr             -b*cr              0           cr      ];
K = [   kf+kr           b*kr-a*kf         -kf         -kr     ;
        b*kr-a*kf     kf*a^2+kr*b^2     a*kf       -b*kr  ;
        -kf             a*kf               kf+kt      0       ;
        -kr             -b*kr              0           kr+kt  ];
F = [   0       0   ;
        0       0   ;
        kt     0   ;
        0       kt ];
    
% State space model
A = [   zeros(4,4)      eye(4,4)   ;
        -M\K         -M\C    ];
B = [   zeros(4,2)  ;
        M\F      ];           
C = [   1 0 0 0 0 0 0 0 ;
        0 1 0 0 0 0 0 0 ;
        0 0 1 0 0 0 0 0 ;
        0 0 0 1 0 0 0 0 ;
        0 0 0 0 0 0 0 0 ;
        0 0 0 0 0 0 0 0 ;
        0 0 0 0 0 0 0 0 ;
        0 0 0 0 0 0 0 0 ];
D = zeros(8,2);
sys = ss(A,B,C,D);
% Input
vel = 10;                       % Longitudinal speed of the car            
lon_pos_1 = vel*time + a+b;   % Longitudinal position of the front axle  

lon_pos_2 = vel*time;           % Longitudinal position of the rear axle  

u1 = interp1(X_r,Z_r,lon_pos_1);
u2 = interp1(X_r,Z_r,lon_pos_2);
figure
hold on ; grid on ; box on
plot(time,u1,'r')
plot(time,u2,'g')
xlabel('Distance x (meters)')
ylabel('Distance z (meters)')
title('Input')
legend('u1','u2')
u_vet = [u1' u2'];
[y,time,x] = lsim(sys,u_vet,time);
z       = y(:,1); % Body vertical motion coordinate        
theta   = y(:,2); % Body pitch motion coordinate            
xuf     = y(:,3); % Front wheel vertical motion coordinate 
xur     = y(:,4); % Rear wheel vertical motion coordinate   
figure
hold on ; grid on ; box on
plot(time,z)
plot(time,xuf,'b*')
plot(time,xur,'r-')
xlabel('Time (seconds)')
ylabel('Vertical coordinate (meters)')
legend('z','xuf','xur')
%% Animation
color = cool(6); % Colormap
figure
% set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% set(gcf,'Position',[50 50 854 480])   % YouTube: 480p
set(gcf,'Position',[50 50 640 640])     % Social
% Create and open video writer object
v = VideoWriter('half_car_model.mp4','MPEG-4');
v.Quality   = 100;
% v.FrameRate = fR;
open(v);
for i=1:length(time)
    
    cla
    % Instant position
    x_inst = vel*time(i);
    
    % Track passing by:
    set(gca,'xlim',[x_inst-l_win_maring x_inst+l_win],'ylim',[-1.7 -1.7+l_win])
    hold on ; grid on ; box on %; axis equal (Dont work well. It drifts vertically)
    plot([-10 X_r],[0 Z_r],'k','LineWidth',3)
    
    set(gca,'FontName','Verdana','FontSize',16)
    title(["Half car model",strcat('Time=',num2str(time(i),'%.3f'),' s (Playback speed=',num2str(playback_speed),')')])
    % Sprung mass plot
    fill([x_inst a+b+x_inst a+b+x_inst x_inst],[z(i)+z_sstat+b*theta(i) z(i)+z_sstat-a*theta(i) z(i)+z_sstat+h_s-a*theta(i) z(i)+z_sstat+h_s+b*theta(i)],color(6,:),'LineWidth',2)
  
    % Unprung mass front plot
    fill([x_inst-w_u/2+a+b-w_e     x_inst+w_u/2+a+b   x_inst+w_u/2+a+b   x_inst-w_u/2+a+b-w_e],...
         [xuf(i)+z_ustat  xuf(i)+z_ustat  xuf(i)+h_u+z_ustat  xuf(i)+h_u+z_ustat],color(2,:),'LineWidth',2)
    
    % Unprung mass rear plot
    fill([x_inst-w_u/2   x_inst+w_u/2+w_e     x_inst+w_u/2+w_e    x_inst-w_u/2],...
         [xur(i)+z_ustat   xur(i)+z_ustat  xur(i)+h_u+z_ustat     xur(i)+h_u+z_ustat],color(1,:),'LineWidth',2)
    % Damper
    plotDamper(z_sstat,z_ustat,y,i, x_inst+a+b , x_inst,a,b,h_u)
      
	% Spring
    plotSpring(z_sstat,z_ustat,u_vet,y,i, x_inst+a+b , x_inst,a,b,h_u)
    
    % Tire
    plot(x_inst+a+b,u_vet(i,1),'ko','MarkerFacecolor','k','MarkerSize',7) % Front
    plot(x_inst,u_vet(i,2),'ko','MarkerFacecolor','k','MarkerSize',7) % Rear
  
    xlabel('x [m]')
    ylabel('z [m]')
    
    frame = getframe(gcf);
    writeVideo(v,frame);
    
end
close(v);
function plotSpring(z_s_static , z_u_static , u_vet , y , i , x_1 , x_2 , a1 , a2 , h_u)
    % Spring parameters
    rod_Pct    = 0.11;      % Length rod percentage of total gap
    spring_Pct = 1/3;       % Spring pitch percentage of spring gap 
    spring_wid = 2;         % Spring line width
    % Tire 1 spring  geometry 
    c_t_1 = x_1;            % Longitudinal position
    w_t_1 = 0.07;            % Width
    % Tire 2 spring geometry 
    c_t_2 = x_2;         	% Longitudinal position
    w_t_2 = 0.07;            % Width
    
    % Suspension 1 spring geometry 
    c_s_1 = x_1;            % Longitudinal position
    w_s_1 = 0.1;            % Width
    % Suspension 2 spring geometry 
    c_s_2 = x_2;         	% Longitudinal position
    w_s_2 = 0.1;            % Width
    
    % Base front and rear
    z_b_1 = u_vet(:,1); % Front
    z_b_2 = u_vet(:,2); % Rear
    % Unsprung mass absolute vertical position (lower center point)
    z_u_1 = y(:,3) + z_u_static; % Front
    z_u_2 = y(:,4) + z_u_static; % Rear
    % Spring front and rear (Tire) length without rods
    L_u_1 = (z_u_1 - z_b_1) - 2*rod_Pct * z_u_static; % Front
    L_u_2 = (z_u_2 - z_b_2) - 2*rod_Pct * z_u_static; % Rear
    % Sprung mass front and rear absolute vertical position (lower center point)
    z_s_1 = y(:,1) - a1*y(:,2) + z_s_static; % Front
    z_s_2 = y(:,1) + a2*y(:,2) + z_s_static; % Rear
    
    % Spring front and rear (Suspension) length without rods
    L_s_1 = (z_s_1 - z_u_1 - h_u) - 2*rod_Pct*(z_s_static-z_u_static-h_u) ;
    L_s_2 = (z_s_2 - z_u_2 - h_u) - 2*rod_Pct*(z_s_static-z_u_static-h_u) ;
    % Spring tire 1 front
    spring_u_1_X = [ 
                c_t_1                               % Start
                c_t_1                               % rod
                c_t_1 + w_t_1                       % Part 1   
                c_t_1 - w_t_1                       % Part 2
                c_t_1 + w_t_1                       % Part 3
                c_t_1 - w_t_1                       % Part 4
                c_t_1 + w_t_1                       % Part 5
                c_t_1 - w_t_1                       % Part 6
                c_t_1                               % Part 7
                c_t_1                               % rod/End
                ];
    
	spring_u_1_Y = [ 
                z_b_1(i)                                            % Start
                z_b_1(i)+  rod_Pct*z_u_static                         % rod
                z_b_1(i)+  rod_Pct*z_u_static                         % Part 1 
                z_b_1(i)+  rod_Pct*z_u_static+  spring_Pct*L_u_1(i)     % Part 2
                z_b_1(i)+  rod_Pct*z_u_static+  spring_Pct*L_u_1(i)     % Part 3
                z_b_1(i)+  rod_Pct*z_u_static+2*spring_Pct*L_u_1(i)   % Part 4
                z_b_1(i)+  rod_Pct*z_u_static+2*spring_Pct*L_u_1(i)   % Part 5
                z_b_1(i)+  rod_Pct*z_u_static+3*spring_Pct*L_u_1(i)   % Part 6
                z_b_1(i)+  rod_Pct*z_u_static+3*spring_Pct*L_u_1(i)   % Part 7
                z_b_1(i)+2*rod_Pct*z_u_static+3*spring_Pct*L_u_1(i) % rod/End
               ]; 
           
    % Spring tire 2 rear
    spring_u_2_X = [ 
                c_t_2                               % Start
                c_t_2                               % rod
                c_t_2+w_t_2                         % Part 1   
                c_t_2-w_t_2                         % Part 2
                c_t_2+w_t_2                         % Part 3
                c_t_2-w_t_2                         % Part 4
                c_t_2+w_t_2                         % Part 5
                c_t_2-w_t_2                         % Part 6
                c_t_2                               % Part 7
                c_t_2                               % rod/End
                ];
    
	spring_u_2_Y = [ 
                z_b_2(i)                                            % Start
                z_b_2(i)+rod_Pct*z_u_static                         % rod
                z_b_2(i)+rod_Pct*z_u_static                         % Part 1 
                z_b_2(i)+rod_Pct*z_u_static+spring_Pct*L_u_2(i)     % Part 2
                z_b_2(i)+rod_Pct*z_u_static+spring_Pct*L_u_2(i)     % Part 3
                z_b_2(i)+rod_Pct*z_u_static+2*spring_Pct*L_u_2(i)   % Part 4
                z_b_2(i)+rod_Pct*z_u_static+2*spring_Pct*L_u_2(i)   % Part 5
                z_b_2(i)+rod_Pct*z_u_static+3*spring_Pct*L_u_2(i)   % Part 6
                z_b_2(i)+rod_Pct*z_u_static+3*spring_Pct*L_u_2(i)   % Part 7
                z_b_2(i)+2*rod_Pct*z_u_static+3*spring_Pct*L_u_2(i) % rod/End
               ]; 
    % Spring suspension 1 front
    spring_s_1_X = [ 
                c_s_1                               % Start
                c_s_1                               % rod
                c_s_1+w_s_1                         % Part 1   
                c_s_1-w_s_1                         % Part 2
                c_s_1+w_s_1                         % Part 3
                c_s_1-w_s_1                         % Part 4
                c_s_1+w_s_1                         % Part 5
                c_s_1-w_s_1                         % Part 6
                c_s_1                               % Part 7
                c_s_1                               % rod/End
                ];
    
	spring_s_1_Y = [ 
                z_u_1(i)+h_u                                                                % Start
                z_u_1(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)                          % rod
                z_u_1(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)                          % Part 1 
                z_u_1(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+  spring_Pct*L_s_1(i)    % Part 2
                z_u_1(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+  spring_Pct*L_s_1(i)    % Part 3
                z_u_1(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+2*spring_Pct*L_s_1(i)    % Part 4
                z_u_1(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+2*spring_Pct*L_s_1(i)    % Part 5
                z_u_1(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+3*spring_Pct*L_s_1(i)    % Part 6
                z_u_1(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+3*spring_Pct*L_s_1(i)    % Part 7
                z_u_1(i)+h_u+2*rod_Pct*(z_s_static-z_u_static-h_u)+3*spring_Pct*L_s_1(i)    % rod/End
               ];
           
    % Spring suspension 2 rear 
    spring_s_2_X = [ 
                c_s_2                               % Start
                c_s_2                               % rod
                c_s_2+w_s_2                         % Part 1   
                c_s_2-w_s_2                         % Part 2
                c_s_2+w_s_2                         % Part 3
                c_s_2-w_s_2                         % Part 4
                c_s_2+w_s_2                         % Part 5
                c_s_2-w_s_2                         % Part 6
                c_s_2                               % Part 7
                c_s_2                               % rod/End
                ];
    
	spring_s_2_Y = [ 
                z_u_2(i)+h_u                                                                % Start
                z_u_2(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)                          % rod
                z_u_2(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)                          % Part 1 
                z_u_2(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+  spring_Pct*L_s_2(i)    % Part 2
                z_u_2(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+  spring_Pct*L_s_2(i)    % Part 3
                z_u_2(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+2*spring_Pct*L_s_2(i)    % Part 4
                z_u_2(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+2*spring_Pct*L_s_2(i)    % Part 5
                z_u_2(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+3*spring_Pct*L_s_2(i)    % Part 6
                z_u_2(i)+h_u+  rod_Pct*(z_s_static-z_u_static-h_u)+3*spring_Pct*L_s_2(i)    % Part 7
                z_u_2(i)+h_u+2*rod_Pct*(z_s_static-z_u_static-h_u)+3*spring_Pct*L_s_2(i)    % rod/End
               ];
    % PLOT
    plot(spring_u_1_X,spring_u_1_Y,'k','LineWidth',spring_wid)
    plot(spring_u_2_X,spring_u_2_Y,'k','LineWidth',spring_wid)
    plot(spring_s_1_X,spring_s_1_Y,'k','LineWidth',spring_wid)
    plot(spring_s_2_X,spring_s_2_Y,'k','LineWidth',spring_wid)
        
end
function plotDamper(z_s_static , z_u_static , y , i , x_1 , x_2 , a1 , a2 , h_u)
    
    % Damper parameters
    rod_Lower_Pct = 0.1;      % Length lower rod percentage of total gap  
    rod_Upper_Pct = 0.55;      % Length upper rod percentage of total gap
    cylinder_Height_Pct = 0.55;      % Length cylinder percentage of total gap
    damper_line_wid  = 2;   % Damper line width
    
    offset_hor_1 = -0.3; % Horizontal offset 1 front [m]
    offset_hor_2 =  0.3; % Horizontal offset 2 rear [m]
    
    % Suspension 1 spring geometry
    c_1 = x_1;  % Longitudinal position
    w_1 = 0.07;  % Width
    
    % Suspension 2 spring geometry
    c_2 = x_2;  % Longitudinal position
    w_2 = 0.07;  % Width
    
    % Unsprung mass absolute vertical position (lower center point)
    z_u_1 = y(:,3) + z_u_static; % Front
    z_u_2 = y(:,4) + z_u_static; % Rear
    % Sprung mass front and rear absolute vertical position (lower center point)
    z_s_1 = y(:,1) - a1*y(:,2) + z_s_static; % Front
    z_s_2 = y(:,1) + a2*y(:,2) + z_s_static; % Rear
    
    % rod attached to unsprung mass
    rod_u_1_X = [c_1+offset_hor_1 c_1+offset_hor_1]';
    rod_u_1_Y = [z_u_1+h_u  z_u_1+h_u+rod_Lower_Pct*(z_s_static-z_u_static)];
    
    rod_u_2_X = [c_2+offset_hor_2 c_2+offset_hor_2];
    rod_u_2_Y = [z_u_2+h_u  z_u_2+h_u+rod_Lower_Pct*(z_s_static-z_u_static)];
    
    % Damper 1 base cylinder - rod - base 
    cylinder_1_X = [   
                    c_1-w_1+offset_hor_1
                    c_1-w_1+offset_hor_1
                    c_1+w_1+offset_hor_1
                    c_1+w_1+offset_hor_1
                ];
                
    cylinder_1_Y = [
                    z_u_1(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static)+cylinder_Height_Pct*(z_s_static-z_u_static)
                    z_u_1(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static) 
                    z_u_1(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static) 
                    z_u_1(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static)+cylinder_Height_Pct*(z_s_static-z_u_static)
                ];
            
    % Damper 2 base cylinder - rod - base 
    cylinder_2_X = [   
                    c_2-w_2+offset_hor_2
                    c_2-w_2+offset_hor_2
                    c_2+w_2+offset_hor_2
                    c_2+w_2+offset_hor_2
                ];
                
    cylinder_2_Y = [
                    z_u_2(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static)+cylinder_Height_Pct*(z_s_static-z_u_static)
                    z_u_2(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static) 
                    z_u_2(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static) 
                    z_u_2(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static)+cylinder_Height_Pct*(z_s_static-z_u_static)
                ];
    
    % rod attached to sprung mass 1 front
    rod_s_1_X = [c_1+offset_hor_1  c_1+offset_hor_1];
    rod_s_1_Y = [z_s_1 z_s_1-rod_Upper_Pct*(z_s_static-z_u_static)];
    % rod attached to sprung mass 2 rear
    rod_s_2_X = [c_2+offset_hor_2  c_2+offset_hor_2];
    rod_s_2_Y = [z_s_2 z_s_2-rod_Upper_Pct*(z_s_static-z_u_static)];
    % Piston inside cylinder 1 front
    piston_1_X = [c_1-0.6*w_1+offset_hor_1  c_1+0.6*w_1+offset_hor_1];
    piston_1_Y = [z_s_1-rod_Upper_Pct*(z_s_static-z_u_static) z_s_1-rod_Upper_Pct*(z_s_static-z_u_static)];
    
    % Piston inside cylinder 2 rear
    piston_2_X = [c_2-0.6*w_2+offset_hor_2  c_2+0.6*w_2+offset_hor_2];
    piston_2_Y = [z_s_2-rod_Upper_Pct*(z_s_static-z_u_static) z_s_2-rod_Upper_Pct*(z_s_static-z_u_static)];
    
    % Total damper 1 iteration
    rod_u_1_Y_val  = rod_u_1_Y(i,:);
    rod_s_1_Y_val  = rod_s_1_Y(i,:);
    piston_1_Y_Val = piston_1_Y(i,:);
    
    % Total damper 2 iteration
    rod_u_2_Y_val  = rod_u_2_Y(i,:);
    rod_s_2_Y_val  = rod_s_2_Y(i,:);
    piston_2_Y_Val = piston_2_Y(i,:);
    % PLOT DAMPER 1
    % rods
    plot(rod_u_1_X,rod_u_1_Y_val,'k','LineWidth',damper_line_wid)
    plot(rod_s_1_X,rod_s_1_Y_val,'k','LineWidth',damper_line_wid)
    % Damper parts
    plot(piston_1_X,piston_1_Y_Val,'k','LineWidth',damper_line_wid)
    plot(cylinder_1_X,cylinder_1_Y,'k','LineWidth',damper_line_wid)
    
    % PLOT DAMPER 2
    % rods
    plot(rod_u_2_X,rod_u_2_Y_val,'k','LineWidth',damper_line_wid)
    plot(rod_s_2_X,rod_s_2_Y_val,'k','LineWidth',damper_line_wid)
    % Damper parts
    plot(piston_2_X,piston_2_Y_Val,'k','LineWidth',damper_line_wid)
    plot(cylinder_2_X,cylinder_2_Y,'k','LineWidth',damper_line_wid)
end
