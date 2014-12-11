com = serial('/dev/ttyACM0', 'BaudRate', 115200);
fopen(com);
fscanf(com)

%%
fclose(com)