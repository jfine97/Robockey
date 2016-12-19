if(exist('M2USB'))
  fclose(M2USB);
else
%  fclose(instrfindall);
 delete(instrfindall);
end

maxPoints = 50;
Heading = zeros(1,maxPoints);                    % Pre-allocate arrays for IMU values
BOT_X = zeros(1,maxPoints);
BOT_Y = zeros(1,maxPoints);
NumStars = zeros(1,maxPoints);

M2USB = serial('COM3','Baudrate', 9600);
% M2USB = serial('COM5','Baudrate', 9600);

fopen(M2USB);                                   % Open up the port to the M2 microcontroller
flushinput(M2USB);                              % Flush the input buffer

fwrite(M2USB,1);  
i = 1;  


%% Plotting
figure();
clf;
title('XY Position Plot');        
% plot(x,'XDataSource','real(x)','YDataSource','imag(x)')
subplot(1,1,1);
hold on
xlabel('X Position');
ylabel('Y Position');        
axis([-512 512 -384 384]);
POS = plot(BOT_X, BOT_Y,'.r');

POS.XDataSource = 'BOT_X';
POS.YDataSource = 'BOT_Y';

legend('Position');
grid on;
grid minor;

% Send initial confirmation packet
fwrite(M2USB,1);

%% Run program forever
try
    while 1
        
        %% Read in data and send confirmation packet
        m2_buffer = fgetl(M2USB);               % Load buffer
        disp('recieved');
        fwrite(M2USB,1);                        % Confirmation packet
         disp('written');
        %% Parse microcontroller data
        [T_X, remain] = strtok(m2_buffer);
        [B_X, remain2] = strtok(remain);
        [B_Y, remain3] = strtok(remain2);
        [L_X] = strtok(remain3);
        m2_buffer;
        
        % Remove the oldest entry 
        Heading = [str2double(T_X) Heading(1:maxPoints-1)] ;
        BOT_X = [str2double(B_X) BOT_X(1:maxPoints-1)] ;
        BOT_Y = [str2double(B_Y) BOT_Y(1:maxPoints-1)] ;
        NumStars = [str2double(L_X) NumStars(1:maxPoints-1)] ;
        
        % Update plots
        refreshdata;
        drawnow;
%         disp(['Heading: ' num2str(Heading(1)) '   NumStars: ' num2str(NumStars(1))])
%         disp(['X: ' num2str(BOT_X(1)) '   Y: ' num2str(BOT_Y(1))])
        pause(.0001);
        
        hold off
        
        i=i+1;
    end
catch ME
    ME.stack
    fclose(M2USB);                              % Close serial object
end
