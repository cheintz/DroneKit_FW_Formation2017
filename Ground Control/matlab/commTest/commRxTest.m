%Receive any UDP packets on port 5001


fclose(instrfindall);
u = udp('192.168.1.254','LocalPort',5001, 'InputBufferSize', 8192);
fopen(u);
disp('Port opened')

while (1)
   	msg = fread(u,2048);
    disp(char(msg'))
end


fclose(u);
delete(u);
clear u


