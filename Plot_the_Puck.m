if(exist('M2USB'))
  fclose(M2USB);
else
%  fclose(instrfindall);
 delete(instrfindall);
end

maxPoints = 50;
BOT_X = zeros(1,maxPoints);
BOT_Y = zeros(1,maxPoints);
PUCK_X = zeros(1,maxPoints);
PUCK_Y = zeros(1,maxPoints);

% M2USB = serial('COM5','Baudrate', 9600);
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
BOT = plot(BOT_X, BOT_Y,'.r');
PUCK = plot(BOT_X, BOT_Y,'.k');
BOT.XDataSource = 'BOT_X';
BOT.YDataSource = 'BOT_Y';
PUCK.XDataSource = 'PUCK_X';
PUCK.YDataSource = 'PUCK_Y';

legend('BOT', 'PUCK');
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
        [B_X, remain] = strtok(m2_buffer);
        [B_Y, remain2] = strtok(remain);
        [P_X, remain3] = strtok(remain2);
        [P_Y] = strtok(remain3);
        m2_buffer;
        
        % Remove the oldest entry 
        BOT_X = [str2double(B_X) BOT_X(1:maxPoints-1)] ;
        BOT_Y = [str2double(B_Y) BOT_Y(1:maxPoints-1)] ;
        PUCK_X = [str2double(P_X) PUCK_X(1:maxPoints-1)] ;
        PUCK_Y = [str2double(P_Y) PUCK_Y(1:maxPoints-1)] ;
        
        % Update plots

        refreshdata;
        drawnow;
        
        pause(.0001);
        
        hold off
        
        i=i+1;
    end
catch ME
%     disp('FUCK Kp Kd Nega Nega PID');
    ME.stack
    fclose(M2USB);                              % Close serial object
end
