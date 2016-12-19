if(exist('M2USB'))
  fclose(M2USB);
else
%  fclose(instrfindall);
 delete(instrfindall);
end

maxPoints = 20;
BOT_X = zeros(1,maxPoints);                    % Pre-allocate arrays for IMU values
BOT_Y = zeros(1,maxPoints);
PUCK_X = zeros(1,maxPoints);
PUCK_Y = zeros(1,maxPoints);
BOT_HEADING = zeros(1,maxPoints);
ANGLE_TO_PUCK = zeros(1,maxPoints);
% RT_X = zeros(1,maxPoints);
% RT_Y = zeros(1,maxPoints);
% TB_V = zeros(1,maxPoints);
% LR_V = zeros(1,maxPoints);

% M2USB = serial('COM5','Baudrate', 9600);
M2USB = serial('COM3','Baudrate', 9600);

fopen(M2USB);                                   % Open up the port to the M2 microcontroller
flushinput(M2USB);                              % Flush the input buffer

fwrite(M2USB,1);  
i = 1;  

%% Plotting
figure();
clf;
title('Live Plot');        
% plot(x,'XDataSource','real(x)','YDataSource','imag(x)')
subplot(1,1,1);
hold on
xlabel('X Position');
ylabel('Y Position');        
axis([0 1023 0 768]);
PUCK = plot(BOT_X, BOT_Y,'or');
PUCK = plot(PUCK_X, PUCK_Y,'.k');
BOT_HEADING = quiver(BOT_X, BOT_Y, 20*cosd(HEADING + 90), ...
    20*sind(HEADING +90), 'b');
ANGLE_PUCK = quiver(BOT_X, BOT_Y, 20*cosd(ANGLE_TO_PUCK + 90), ...
    20*sind(ANGLE_TO_PUCK +90), 'r+');
BOT.XDataSource = 'BOT_X';
BOT.YDataSource = 'BOT_Y';
PUCK.XDataSource = 'PUCK_X';
PUCK.YDataSource = 'PUCK_Y';

legend('BOT', 'PUCK', 'BOT_HEADING', 'ANGLE_PUCK');
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
%         [L_Y, remain6] = strtok(remain5);
        [L_Y] = strtok(remain5);
%         [R_X, remain7] = strtok(remain6);
%         [R_Y, remain8] = strtok(remain7);
%         [TB, remain9] = strtok(remain8);
%         [LR] = strtok(remain9);
        m2_buffer;
        
        % Remove the oldest entry 
        BOT_X = [str2double(T_X) BOT_X(1:maxPoints-1)] ;
        BOT_Y = [str2double(T_Y) BOT_Y(1:maxPoints-1)] ;
        PUCK_X = [str2double(B_X) PUCK_X(1:maxPoints-1)] ;
        PUCK_Y = [str2double(B_Y) PUCK_Y(1:maxPoints-1)] ;
        BOT_HEADING =  [str2double(L_X) BOT_HEADING(1:maxPoints-1)] ;
        ANGLE_TO_PUCK =  [str2double(L_Y) ANGLE_TO_PUCK(1:maxPoints-1)] ;
%         RT_X =  [str2double(R_X) RT_X(1:maxPoints-1)] ;
%         RT_Y =  [str2double(R_Y) RT_Y(1:maxPoints-1)] ;
%         TB_V =  [str2double(TB)  TB_V(1:maxPoints-1)] ;
%         LR_V =  [str2double(LR)  LR_V(1:maxPoints-1)] ;
        
        % Update plots

        refreshdata;
        drawnow;
%         disp(['TB  Vect: ' num2str(TB_V(1)) '  LR  Vect: ' num2str(LR_V(1))])
        
        pause(.0001);
        
        hold off
        
        i=i+1;
    end
catch ME
%     disp('FUCK Kp Kd Nega Nega PID');
    ME.stack
    fclose(M2USB);                              % Close serial object
end
