Compile First:
```bash
g++ -O2 -Wall -Wextra -pedantic udp_server.cpp -o udp_server
```

```bash
g++ -O2 -Wall -Wextra -pedantic udp_client.cpp -o udp_client
```

Then run:
```bash
cd src
./udp_server
./udp_client <server_ip> <message> <port_number>
```

example (run client in a new terminal):
```bash
./udp_client 127.0.0.1 "hello world" 6000
```
