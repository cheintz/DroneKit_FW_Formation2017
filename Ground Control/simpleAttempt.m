echotcpip('on',4012)
t = tcpip('localhost',4012);
fopen(t)
fwrite(t,65:74)
A = fread(t, 10);

fclose(t)
echotcpip('off')