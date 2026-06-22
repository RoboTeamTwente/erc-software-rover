
#include <netinet/in.h> //sockaddr_in
#include <sys/socket.h> // socket, bind, recvfrom, sendto
#include <unistd.h> // close
#include <arpa/inet.h> // htons, htonl, nthos, inet_ntop, INET_ADDRSTRLEN
#include <cstdio> // perror

#include <cstdint>
#include <cstring> //memset
#include <iostream>

int main() {
    // 1) Create a UDP socket
    // socket() expects three parameters: domain, type, and protocol. For UDP, we use AF_INET for IPv4, SOCK_DGRAM for datagram socket, and 0 for the default protocol.
    // default protocol in this instance is chosen based on AF_INET and SOCK_DGRAM.
    // this points to the UDP protocol in the IP suite. (IPPROTO_UDP);
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    // the "::" before socket is used to specify that we want to use the global namespace version of the socket function, which is the one provided by the system's socket library. This is often done to avoid any potential naming conflicts with other functions or variables named socket in the code.
    if (sock < 0) {
        perror("Socket creation failed");
        return 1;
    }

    // 2) Bind the socket to a local IP and port so the OS delivers the packets
    // sock was created, but it is not yet associated with any specific local address or port. By binding the socket to a local IP address and port, we tell the operating system that we want to receive UDP packets sent to that address and port. This is necessary for the server to function correctly, as it allows the OS to route incoming packets to our application.
    // sock is the socket handle, a file descriptor and just identies which socket we are configuring.
    // sockaddr_in is a structure that holds an internet address. It is used to specify the address and port for the socket. We initialize it with zeros using {} and then set the family to AF_INET (IPv4), the port to 5000 (converted to network byte order using htons), and the address to INADDR_ANY (
    // sockaddr_Familyin is an address description, specifying what local IP+port you want the socket to use.
    sockaddr_in local{};
    // sockaddr_in local{} simply initializes the struct with 0s (C++ logic). Every byte becomes 0.
    // It's the modern C++ version of memset(&local, 0, sizeof(local)) in C. It ensures that all fields of the sockaddr_in structure are initialized to zero before we set the specific values for the family, port, and address.

    local.sin_family = AF_INET;
    local.sin_port = htons(5000);              // listen here
    local.sin_addr.s_addr = htonl(INADDR_ANY); // all interfaces
    // sin_addr is the IPv4 address field in the struct (type in_addr)
    // s_addr is the actual 32-bit integer that holds the IP address in network byte order. By setting it to htonl(INADDR_ANY), we are telling the server to listen on all available network interfaces for incoming UDP packets on port 5000.
    // This corresponds to IPv4 address 0.0.0.0
    // So if the machine has multiple network interfaces (e.g., Ethernet, Wi-Fi, 127.0.0.1 loopback), the server will receive packets sent to any of those interfaces on port 5000.
    // htonls converts the host byte order to network byte order (big-endian).
    // because s_addr is a 32-bit field in network byte order. Port is 16-bit -> htons, while IPv4 address is 32-bit -> htonl.

    if (::bind(sock, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        perror("Bind failed");
        ::close(sock);
        return 1;
    }

    // Defining two destination addresses for forwarding received packets
    // dstA and dstB represent the addresses of two different machines (or ports) to which we want to forward the received UDP packets. In this example, both destinations are set to localhost (127.0.0.1)
    sockaddr_in dstA{};
    dstA.sin_family = AF_INET; // IPv4
    dstA.sin_port = htons(6000); // port on machine A that listens
    if (inet_pton(AF_INET, "127.0.0.1", &dstA.sin_addr) != 1) {
        perror("inet_pton failed for dstA");
        ::close(sock);
        return 1;
    }
    sockaddr_in dstB{};
    dstB.sin_family = AF_INET; // IPv4
    dstB.sin_port = htons(6001); // port on machine B that listens
    if (inet_pton(AF_INET, "127.0.0.1", &dstB.sin_addr) != 1) {
        perror("inet_pton failed for dstB");
        ::close(sock);
        return 1;
    }
   
    // 3) Receive loop: each recvfrom() get exactly one UDP datagram
    std::uint8_t buffer[2048]; // pick a size you expect; keep in mind that the maximum size of a UDP datagram is 65507 bytes (65,535 - 8 byte UDP header - 20 byte IP header)
    // buffer is a raw byte array living on the stack, used to store the payload of the received datagram. The size of 2048 bytes is an arbitrary choice that should be sufficient for many applications, but you can adjust it based on your specific needs and expected datagram sizes.
    // if a UDP packet arrives larger than the buffer size, the extra bytes are discarded (truncated). n will be the amount yoou actually copied.

    for (;;) {
        sockaddr_in sender{};
        // sender is a sockaddr_in structure that will be filled with the sender's address information when a datagram is received. It will contain the sender's IP address and port number, which can be useful for processing the received data and sending responses back to the sender.
        // sender{} zeroes the sender structure, ensuring that all fields are initialized to zero before recvfrom() fills it with the sender's information.
        socklen_t sender_len = sizeof(sender);
        // sender_len tells the kernel how much space we have allocated for the sender's address information. It is initialized to the size of the sender structure, and after recvfrom() returns, it will be updated to reflect the actual size of the sender's address information that was filled in by the kernel.

        // Blocks until a datagram arrives. The received data is stored in buffer, and the sender's address is stored in sender.
        ssize_t n = ::recvfrom(
            sock,       // (1) which socket to receive on
            buffer,     // (2) where to store payload bytes of the received datagram
            sizeof(buffer),     // (3) max bytes to copy into buffer
            0,      // (4) flags (0 for no special options)
            reinterpret_cast<sockaddr*>(&sender),       // (5) where to store sender address (IP + port)
            &sender_len     // (6) in/out: size of address structure (input: how much space we have, output: how much was actually filled in by the kernel
        );
        // (4) flags parameter can be used to specify special options for receiving data, such as non-blocking mode or out-of-band data. In this case, we set it to 0, which means we want the default behavior (blocking mode, no special options).
        // Other examples of flags include MSG_DONTWAIT for non-blocking mode, MSG_PEEK to peek at the incoming data without removing it from the queue, and MSG_WAITALL to wait for the full amount of data to be received before returning.

        // n is the number of payload bytes copied into buffer.
        if (n < 0) {
            perror("recvfrom failed");
            continue; // try to receive the next datagram, or you could choose to break the loop and exit the server
        }

        
        // 4) At this point: buffer [0..n-1] contains the raw payload bytes of the received datagram, and sender contains the sender's IP address and port. You can process the data as needed. For example, you could print the sender's information and the received message.
        // sender contains the source IP/port
        char ip[INET_ADDRSTRLEN]{};
        ::inet_ntop(AF_INET, &sender.sin_addr, ip, sizeof(ip)); // convert sender's IP address to human-readable form
        std::cout << "Received " << n << " bytes from " << ip << ":" << ntohs(sender.sin_port) << std::endl;

        // 5) Optionally, you can send a response back to the sender using sendto(). For example, you could echo the received message back to the sender.
        // Forward everything to both ports for now
        ssize_t sentB = sendto(
            sock,
            buffer,
            n,
            0,
            reinterpret_cast<sockaddr*>(&dstB),
            sizeof(dstB)
        );
        // came efrom elsewhere -> forward to A
        ssize_t sentA = sendto(
            sock,
            buffer,
            n,
            0,
            reinterpret_cast<sockaddr*>(&dstA),
            sizeof(dstA)
        );
        if (sentA < 0 || sentB < 0) {
            perror("sendto failed");
        } else {
           std::cout << "Forwarded A=" << sentA << " bytes, B=" << sentB << " bytes\n";
        }

        // 6) Optionally: parse payload (later: protobuf, custom header etc.)
    }

    // 7) Cleanup (unreachable in this infinite loop, but good practice to include)

    ::close(sock);
    return 0;
}