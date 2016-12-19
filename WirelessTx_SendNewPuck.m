kp = 1;
kd = 0.5;
PuckX = 300;    
PuckY = 0;



%     axis([-512 512 -384 384]);
%     [PuckX,PuckY] = ginput(1);
    
    delete(instrfindall);

%     if(exist('M2USB'))
%       fclose(M2USB);
%     else
% %      fclose(instrfindall);
%      delete(instrfindall);
%     end
    % *** Use the device manager to check where the microcontroller is plugged
    % into.

    M2USB = serial('COM5', 'BaudRate', 9600);

    fopen(M2USB);                                   % Open up the port to the M2 microcontroller
    flushinput(M2USB); 

    kp = 10*kp;        %scale up for 8-bit conversion
    kd = 10*kd;
    PuckX = PuckX;    
    PuckY = PuckY;
%     Speed = 100*Speed;  
    buffer = [1 PuckX PuckY kp kd 0 0 0 0 0]; % 1 is there as a flag
    for i = 1:10
        fwrite(M2USB, buffer(i));
        disp(buffer(i));
    end
    disp('sent bitches');

    fclose(M2USB);
    delete(M2USB);

