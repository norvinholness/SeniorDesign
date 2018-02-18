clear all; 
clc; 
close all;     
delete(instrfind({'Port'},{'COM6'})); 

% Setting com number for communication 
a = serial('COM6');
set(a,'BaudRate',9600);
%arduino = serial('COM5','BaudRate',9600);     

% Opening serial port 
fopen(a); 
hold on   

% Variables; Sample rate & Data number 
SR = 0.16; 
i = 10;   

% Infinite loop 
while (1)       
    % Reading the serial port          
    str = fscanf(a);   
    
% Finding the letter a for seperation  
index3 = find(str == 'a');   

% First data obtained 
str1 = str(1:index3-1); 

% Second data is obtained 
str2 = str(index3+1:end);   

% Converting string to double precision value (Gyro) 
Gx(i) = str2double(str1);      

% Converting string to double precision value (Acc) 
Ax(i) = str2double(str2);  

% Figure 1 configuration 
subplot(2,1,1),title('Accelerometer XYZ Axis Average Scaled Data');
ylim([0.4 1.6]);
xlabel('Time(s)'),ylabel('Acc (G)'); 
set(gca,'YTick',[0.4 0.6 0.8 1.0 1.2 1.4 1.6]) 
grid on   

% Plotting the graph of ACC 
% X axis is constructed (data number x sample rate) 
% Y axis is constructed by using the read data 
% Only last 5 data is stored for memory limitation 
plot(SR.*(i-5:i),Ax(i-5:i),'--r','LineWidth',2);
hold on  

% Figure 2 configuration 
subplot(2,1,2),title('Gyroscope XYZ Axis Average Scaled Data');
ylim([-100 100]);
xlabel('Time (s)');
ylabel('Gyro (deg/s)'); 
set(gca,'YTick',[-200 0 200 400 600 800 1000]) 
grid on   

% Plotting the graph of GYRO 
% X axis is constructed (data number x sample rate) 
% Y axis is constructed by using the read data 
% Only last 5 data is stored for memory limitation 
plot(SR.*(i-5:i),Gx(i-5:i),'--b','LineWidth',2) 
hold on  
% Remainder for shutting the window periodically 
if (rem (i,200) == 0)     
    close; 
end

% Data number increases & Begin Drawing 
i=i+1; 
drawnow; 
end





