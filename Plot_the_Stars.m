if(exist('M2USB'))
  fclose(M2USB);
else
%  fclose(instrfindall);
 delete(instrfindall);
end

maxPoints = 50;
TOP_X = zeros(1,maxPoints);                    % Pre-allocate arrays for IMU values
TOP_Y = zeros(1,maxPoints);
BOT_X = zeros(1,maxPoints);
BOT_Y = zeros(1,maxPoints);
LF_X = zeros(1,maxPoints);
LF_Y = zeros(1,maxPoints);
RT_X = zeros(1,maxPoints);
RT_Y = zeros(1,maxPoints);
TB_V = zeros(1,maxPoints);
LR_V = zeros(1,maxPoints);

% M2USB = serial('COM3','Baudrate', 9600);
M2USB = serial('COM3','Baudrate', 9600);

fopen(M2USB);                                   % Open up the port to the M2 microcontroller
flushinput(M2USB);                              % Flush the input buffer

fwrite(M2USB,1);  
i = 1;  

%% Plotting
figure();
clf;
title('Star Plot');        
% plot(x,'XDataSource','real(x)','YDataSource','imag(x)')
subplot(1,1,1);
hold on
xlabel('X Position');
ylabel('Y Position');        
axis([0 1023 0 768]);
TOP = plot(TOP_X, TOP_Y,'.r');
BOT = plot(BOT_X, BOT_Y,'.k');
LF = plot(LF_X, LF_Y,'.c');
RT = plot(RT_X, RT_Y,'.g');
TOP.XDataSource = 'TOP_X';
TOP.YDataSource = 'TOP_Y';
BOT.XDataSource = 'BOT_X';
BOT.YDataSource = 'BOT_Y';
LF.XDataSource =  'LF_X';
LF.YDataSource =  'LF_Y';
RT.XDataSource =  'RT_X';
RT.YDataSource =  'RT_Y';

legend('TOP', 'BOTTOM', 'LEFT', 'RIGHT');
grid on;
grid minor;

% Send initial confirmation packet
fwrite(M2USB,1);

%% Run program forever
try
    while 1
        
        %% Read in data and send confirmation packet
        m2_buffer = fgetl(M2USB);               % Load buffer
        fwrite(M2USB,1);                        % Confirmation packet
        
        %% Parse microcontroller data
        [T_X, remain] = strtok(m2_buffer);
        [T_Y, remain2] = strtok(remain);
        [B_X, remain3] = strtok(remain2);
        [B_Y, remain4] = strtok(remain3);
        [L_X, remain5] = strtok(remain4);
        [L_Y, remain6] = strtok(remain5);
        [R_X, remain7] = strtok(remain6);
        [R_Y, remain8] = strtok(remain7);
        [TB, remain9] = strtok(remain8);
        [LR] = strtok(remain9);
        m2_buffer;
        
        % Remove the oldest entry 
        TOP_X = [str2double(T_X) TOP_X(1:maxPoints-1)] ;
        TOP_Y = [str2double(T_Y) TOP_Y(1:maxPoints-1)] ;
        BOT_X = [str2double(B_X) BOT_X(1:maxPoints-1)] ;
        BOT_Y = [str2double(B_Y) BOT_Y(1:maxPoints-1)] ;
        LF_X =  [str2double(L_X) LF_X(1:maxPoints-1)] ;
        LF_Y =  [str2double(L_Y) LF_Y(1:maxPoints-1)] ;
        RT_X =  [str2double(R_X) RT_X(1:maxPoints-1)] ;
        RT_Y =  [str2double(R_Y) RT_Y(1:maxPoints-1)] ;
        TB_V =  [str2double(TB)  TB_V(1:maxPoints-1)] ;
        LR_V =  [str2double(LR)  LR_V(1:maxPoints-1)] ;
        
        % Update plots

        refreshdata;
        drawnow;
        disp(['TB  Vect: ' num2str(TB_V(1)) '  LR  Vect: ' num2str(LR_V(1))])
        
        pause(.0001);
        
        hold off
        
        i=i+1;
    end
catch ME
%     disp('FUCK Kp Kd Nega Nega PID');
    ME.stack
    fclose(M2USB);                              % Close serial object
end
