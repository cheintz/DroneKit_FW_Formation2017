%Receive any UDP packets on port 5001


fclose(instrfindall);
u = udp('192.168.1.1','LocalPort',5001);
fopen(u);
disp('Port opened')

while (1)
   	msg = fread(u,10);
    disp(char(msg'))
end


fclose(u);
delete(u);
clear u


