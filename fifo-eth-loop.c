/**
 * @file axis-fifo-eth-loop.c
 * @author Jason Gutel
 *
 * Sets up an echo ping-pong server over a TCP connection. Packets are sent
 * over sockets, sent to the AXI Stream FIFO core (assumed in loopback), and
 * then sent back out over the socket.
 *
 * Shows example of using poll() with the kernel module
 *
 * @bug No known bugs.
 **/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

#include <fcntl.h>              // Flags for open()
#include <sys/stat.h>           // Open() system call
#include <sys/types.h>          // Types for open()
#include <unistd.h>             // Close() system call
#include <string.h>             // Memory setting and copying
#include <getopt.h>             // Option parsing
#include <errno.h>              // Error codes
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <poll.h>

#include <sys/socket.h>
#include <netinet/in.h>

/*----------------------------------------------------------------------------
 * Internal Definitions
 *----------------------------------------------------------------------------*/
#define DEFAULT_MAX_BUF_SIZE_BYTES 1102
#define DEFAULT_PORT_NO    7777

#define DEBUG
#if defined(DEBUG)
        #define DEBUG_PRINT(fmt, args...) printf("DEBUG %s:%d(): " fmt, \
                __func__, __LINE__, ##args)
#else
        #define DEBUG_PRINT(fmt, args...) /* do nothing */
#endif

struct thread_data {
    int rc;
};

pthread_t eth_rx_thread;
pthread_t fifo_rx_thread;

static volatile bool running = true;
static int _opt_tcp_port = DEFAULT_PORT_NO;
static int _opt_max_bytes = DEFAULT_MAX_BUF_SIZE_BYTES;
static char _opt_dev[255];
static int writeFifoFd;
static int readFifoFd;
static int serverFd;
static int tcpSocketFd;

static int socket_read_error = 0;
static int socket_write_error = 0;

static void signal_handler(int signal);
static void *fifo_rx_thread_fn(void *data);
static int process_options(int argc, char * argv[]);
static void print_opts();
static void display_help(char * progName);
static void *ethn_rx_thread_fn(void *data);
static void quit(void);

/*----------------------------------------------------------------------------
 * Main
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    int rc;
    struct sockaddr_in tcpAddr;
    int opt;
    int addrlen;

    process_options(argc, argv);
    sleep(1);
    printf("Begin...\n");

    // Listen to ctrl+c and assert
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGQUIT, signal_handler);

    /*************/
    /* open FIFO */
    /*************/
    readFifoFd = open(_opt_dev, O_RDONLY);
    writeFifoFd = open(_opt_dev, O_WRONLY);
    if (readFifoFd < 0) {
        printf("Open read failed with error: %s\n", strerror(errno));
        return -1;
    }
    if (writeFifoFd < 0) {
        printf("Open write failed with error: %s\n", strerror(errno));
        return -1;
    }

    /**********************************************/
    /* Start TCP Server and wait for a connection */
    /**********************************************/
    opt=1;
    addrlen=sizeof(tcpAddr);
    if ((serverFd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        DEBUG_PRINT("socket failed");
        exit(EXIT_FAILURE);
    }

    if (setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                  &opt, sizeof(opt))) {
        DEBUG_PRINT("setsockopt");
        exit(EXIT_FAILURE);
    }
    tcpAddr.sin_family = AF_INET;
    tcpAddr.sin_addr.s_addr = INADDR_ANY;
    tcpAddr.sin_port = htons(_opt_tcp_port);

    // Forcefully attaching socket to the port 8080
    if (bind(serverFd, (struct sockaddr *)&tcpAddr,
            sizeof(tcpAddr))<0) {
        DEBUG_PRINT("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(serverFd, 3) < 0) {
        DEBUG_PRINT("listen");
        exit(EXIT_FAILURE);
    }

    if ((tcpSocketFd = accept(serverFd, (struct sockaddr *)&tcpAddr,
            (socklen_t*)&addrlen))<0) {
        DEBUG_PRINT("accept");
        exit(EXIT_FAILURE);
    } else {
        DEBUG_PRINT("accepted client\n\r");
    }

    /*****************/
    /* start threads */
    /*****************/

    /* start thread listening for ethernet packets */
    rc = pthread_create(&eth_rx_thread, NULL, ethn_rx_thread_fn,
            (void *)NULL);

    /* start thread listening for fifo receive packets */
    rc = pthread_create(&fifo_rx_thread, NULL, fifo_rx_thread_fn,
            (void *)NULL);

    /* perform noops */
    while (running) {
       if (socket_read_error || socket_write_error) {
           DEBUG_PRINT("Error %s socket...\n",socket_read_error ? "reading" : "writing"); 
           goto ret;
       }
       sleep(1);
    }

ret:
    printf("SHUTTING DOWN\n");
    pthread_join(eth_rx_thread, NULL);
    pthread_join(fifo_rx_thread, NULL);
    close(writeFifoFd);
    close(readFifoFd);
    return rc;
}

static void quit(void)
{
    running = false;
}

static void *ethn_rx_thread_fn(void *data)
{
    int rc;
    ssize_t bytesSock;
    ssize_t bytesFifo;
    int packets_rx, packets_tx;
    uint8_t buf[_opt_max_bytes+10];
    struct pollfd fds[2];
    int packetRead = 0;

    /* shup up compiler */
    (void)data;

    fds[0].fd = tcpSocketFd;
    fds[1].fd = writeFifoFd;
    fds[0].events = POLLIN;
    fds[1].events = POLLOUT;

    packets_rx = 0;
    packets_tx = 0;

    while (running) {
        rc = poll(fds, 2, -1);

        if (rc > 0) {
            if(packetRead == 0 && (fds[0].revents & POLLIN)) {
                bytesSock = read(tcpSocketFd, buf, _opt_max_bytes);
                if (bytesSock > 0) {
                    packetRead = 1;
                    packets_rx++;
                    DEBUG_PRINT("bytes from socket %d\n",bytesSock);
                } else if (bytesSock == 0) {
                    DEBUG_PRINT("Connection lost\n");
                    socket_read_error = 1;
                    quit();
                } else {
                    perror("read");
                    socket_read_error = 1;
                    quit();
                }
            }

            if (packetRead == 1 && (fds[1].revents & POLLOUT)) {
                bytesFifo = write(writeFifoFd, buf, bytesSock);
                if (bytesFifo > 0) {
                    DEBUG_PRINT("bytes to fifo %d\n",bytesFifo);
                    packets_tx++;
                    packetRead = 0;
                } else {
                    perror("write");
                    quit();
                }
            }
        }
    }

    DEBUG_PRINT("ethernet packets rx : %d, fifo packets tx : %d\n",packets_rx, packets_tx);

    return (void *)0;
}

static void *fifo_rx_thread_fn(void *data)
{
    int rc;
    ssize_t bytesSock;
    ssize_t bytesFifo;
    int packets_rx, packets_tx;
    uint8_t buf[_opt_max_bytes+10];
    struct pollfd fds[2];
    int packetRead = 0;

    /* shup up compiler */
    (void)data;

    fds[0].fd = tcpSocketFd;
    fds[1].fd = writeFifoFd;
    fds[0].events = POLLOUT;
    fds[1].events = POLLIN;

    packets_rx = 0;
    packets_tx = 0;

    while (running) {
        rc = poll(fds, 2, -1);

        if (rc > 0) {
            if(packetRead == 0 && (fds[1].revents & POLLIN)) {
                bytesFifo = read(readFifoFd, buf, _opt_max_bytes);
                if (bytesFifo > 0) {
                    packetRead = 1;
                    DEBUG_PRINT("bytes from fifo %d\n",bytesFifo);
                    packets_rx++;
                } else {
                    perror("read");
                    quit();
                }
            }

            if (packetRead == 1 && (fds[0].revents & POLLOUT)) {
                bytesSock = write(tcpSocketFd, buf, bytesFifo);
                if (bytesSock > 0) {
                    DEBUG_PRINT("bytes to socket %d\n",bytesSock);
                    packets_tx++;
                    packetRead = 0;
                } else {
                    perror("write");
                    socket_write_error = 1;
                    quit();
                }
            }
        }
    }

    DEBUG_PRINT("fifo packets rx : %d, ethernet packets tx : %d\n",packets_rx, packets_tx);

    return (void *)0;
}

static void signal_handler(int signal)
{
    switch (signal) {
        case SIGINT:
        case SIGTERM:
        case SIGQUIT:
            running = false;
            break;

        default:
            break;
    }
}

static void display_help(char * progName)
{
    printf("Usage : %s [OPTIONS]\n"
           "\n"
           "  -h, --help   Print this menu\n"
           "  -d, --dev    Device to use ... /dev/axis_fifo_0x43c10000\n"
           "  -b, --bytes  Number of bytes to expect in a packet\n"
           "  -p, --port   Port number to bind to\n"
           ,
           progName
          );
}

static void print_opts()
{
    printf("Options :\n"
           "Port    : %d\n"
           "Bytes   : %d\n"
           "Dev     : %s\n"
           ,
           _opt_tcp_port,
           _opt_max_bytes,
           _opt_dev
          );
}

static int process_options(int argc, char * argv[])
{
        int devProvided = 0;

        for (;;) {
            int option_index = 0;
            static const char *short_options = "hd:b:p:";
            static const struct option long_options[] = {
                    {"help", no_argument, 0, 'h'},
                    {"dev", required_argument, 0, 'd'},
                    {"bytes", required_argument, 0, 'b'},
                    {"port", required_argument, 0, 'p'},
                    {0,0,0,0},
                    };

            int c = getopt_long(argc, argv, short_options,
            long_options, &option_index);

            if (c == EOF) {
            break;
            }

            switch (c) {
            case 'd':
                devProvided = 1;
                strcpy(_opt_dev, optarg);
                break;

            case 'b':
                _opt_max_bytes = atoi(optarg);
                break;

            case 'p':
                _opt_tcp_port = atoi(optarg);
                break;

            default:
            case 'h':
                display_help(argv[0]);
                exit(0);
                break;
                }
        }

        if (!devProvided) {
            printf("Must provide --dev flag...\n");
            display_help(argv[0]);
            exit(0);
        }
        print_opts();

        return 0;
}
