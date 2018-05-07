# Serves as heartbeat and continuous device monitoring

Header header # Primarly for timestamp
string id # Device serial number
string ip # Ethernet IP adress
string type # Device type, eg. camera, manipulator, etc
int8 leak # 0 for no leak, 1 for leak, -1 for data unavailable
int8 cpu_temp # -1 for data unavailable
