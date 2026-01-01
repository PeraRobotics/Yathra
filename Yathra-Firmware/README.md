python3 -m esp_rfc2217_server -p 4000 /dev/ttyUSB0

sudo chmod 666 /dev/ttyUSB0

sudo ufw allow 4000/tcp

"idf.port": "rfc2217://10.193.85.203:4000?ign_set_control",  


// Thruster 1: Front-Right
// Thruster 2: Front-Left
// Thruster 3: Rear-Right
// Thruster 4: Rear-Left
// Thruster 5: Middle-UP-Right
// Thruster 6: Middle-UP-Left