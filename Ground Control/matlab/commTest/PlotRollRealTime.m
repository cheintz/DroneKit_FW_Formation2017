%Receive any UDP packets on port 5001
clc;clear;close all;

fclose(instrfindall);
u = udp('192.168.1.254','LocalPort',5001, 'InputBufferSize', 8192);
fopen(u);

disp('Port opened')

rolls = [];
while (1)
   	msg = fread(u,2048);
    obj = loadjson(char(msg'));
    id = obj.py_0x2F_state.py_0x2F_tuple{1,3}.py_0x2F_state.py_0x2F_tuple{2};
    rolls(end+1,id)= obj.py_0x2F_state.py_0x2F_tuple{1,3}.py_0x2F_state.py_0x2F_tuple{1,4}.roll;
    plot(rolls);
end


fclose(u);
delete(u);
clear u


