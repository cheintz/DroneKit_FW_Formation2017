%Broadcast UDP packets to verify communication

fclose(instrfindall);
u = udp('192.168.1.255',5001);
fopen(u);

msg='Hello World';

try
    while (1)
       fprintf(u,msg)
       pause(0.02) %50 hz
    end
    
catch e
   fclose(u);
    delete(u);
    clear u
    

end


