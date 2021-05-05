/**
 * @file axis-fifo-ethernet-bridge.c
 * @author Jason Gutel jason.gutel@gmail.com
 *
 * Sets up a bridge between a TCP|UDP socket and the axis fifo
 * driver. Packets received from the socket will be sent to
 * the associated TX FIFO. Packets received form the RX FIFO
 * will be sent back out over the socket. For UDP the socket
 * needs to be initialized so that the partner IPv4 address
 * is known.
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
#include <sys/ioctl.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include "../axis-fifo.h"

/*----------------------------------------------------------------------------
 * Internal Definitions
 *----------------------------------------------------------------------------*/
#define TCP_PROTOCOL 1
#define UDP_PROTOCOL 0
#define DEF_PROTOCOL UDP_PROTOCOL
#define MAX_BUF_SIZE_BYTES 1500
#define DEF_PORT_NO    7777
#define DEF_DEV_TX "/dev/axis_fifo_0x43c10000"
#define DEF_DEV_RX "/dev/axis_fifo_0x43c10000"

#define DEBUG_PRINT(fmt, args...) printf("DEBUG %s:%d(): " fmt, \
        __func__, __LINE__, ##args)

struct thread_data {
    int rc;
};

pthread_t eth_rx_thread;
pthread_t fifo_rx_thread;

static volatile bool running = true;
static int _opt_sock_port = DEF_PORT_NO;
static int _opt_use_protocol = DEF_PROTOCOL;
static char _opt_dev_tx[255];
static char _opt_dev_rx[255];
static int writeFifoFd;
static int readFifoFd;
static int serverFd;
static int tcpSocketFd;
static int _opt_verbose;

/* for udp packets */
static struct sockaddr_in cliaddr;
static socklen_t cliaddrlen;
static int udpInit = 0;

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
    process_options(argc, argv);

    int rc;
    struct sockaddr_in servAddr;
    int opt;
    int addrlen;

    // Listen to ctrl+c and assert
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGQUIT, signal_handler);

    /*************/
    /* open FIFO */
    /*************/
    readFifoFd = open(_opt_dev_rx, O_RDONLY | O_NONBLOCK);
    writeFifoFd = open(_opt_dev_tx, O_WRONLY);
    if (readFifoFd < 0) {
        printf("Open read failed with error: %s\n", strerror(errno));
        return -1;
    }
    if (writeFifoFd < 0) {
        printf("Open write failed with error: %s\n", strerror(errno));
        return -1;
    }

    /*****************************/
    /* initialize the fifo core  */
    /*****************************/
    rc = ioctl(readFifoFd, AXIS_FIFO_RESET_IP);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    rc = ioctl(writeFifoFd, AXIS_FIFO_RESET_IP);
    if (rc) {
        perror("ioctl");
        return -1;
    }

    /**********************************************/
    /* Start TCP Server and wait for a connection */
    /**********************************************/
    opt=1;
    addrlen=sizeof(servAddr);
    if (_opt_use_protocol == TCP_PROTOCOL) {
        if ((serverFd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            printf("socket failed");
            exit(EXIT_FAILURE);
        }
    } else {
        if ((serverFd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }
        cliaddrlen = sizeof(cliaddr);
    }

    if (setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                  &opt, sizeof(opt))) {
        printf("setsockopt");
        exit(EXIT_FAILURE);
    }

    bzero(&servAddr,sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servAddr.sin_port = htons(_opt_sock_port);

    if (bind(serverFd, (struct sockaddr *)&servAddr, sizeof(servAddr))<0) {
        printf("bind failed");
        exit(EXIT_FAILURE);
    }

    if (_opt_use_protocol == TCP_PROTOCOL) {
        if (listen(serverFd, 3) < 0) {
            printf("listen");
            exit(EXIT_FAILURE);
        }

        if ((tcpSocketFd = accept(serverFd, (struct sockaddr *)&servAddr,
                (socklen_t*)&addrlen))<0) {
            printf("accept");
            exit(EXIT_FAILURE);
        } else {
            printf("accepted client\n\r");
        }
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
           printf("Error %s socket...\n",socket_read_error ? "reading" : "writing");
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
    uint8_t buf[MAX_BUF_SIZE_BYTES];
    uint32_t vacancy;
    int vacancyPrint = 0;

    /* shup up compiler */
    (void)data;

    memset(&cliaddr, 0, sizeof(cliaddr));

    packets_rx = 0;
    packets_tx = 0;

    while (running) {
        if (_opt_use_protocol == TCP_PROTOCOL) {
            bytesSock = read(tcpSocketFd, buf, MAX_BUF_SIZE_BYTES);
        } else {
            bytesSock = recvfrom(serverFd, buf, MAX_BUF_SIZE_BYTES,
                    MSG_WAITALL, (struct sockaddr *)&cliaddr,
                    &cliaddrlen);
            udpInit = 1;
        }
        if (bytesSock > 0) {
            packets_rx++;
            if (_opt_verbose)
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

        do {
            rc = ioctl(writeFifoFd, AXIS_FIFO_GET_TX_VACANCY, &vacancy);
            if (rc) {
                perror("ioctl");
                return (void *)0;
            }
            if (vacancy < (uint32_t)bytesSock) {
                usleep(100);
                vacancyPrint++;
            }
        } while (vacancy < (uint32_t)bytesSock);

        bytesFifo = write(writeFifoFd, buf, bytesSock);
        if (bytesFifo > 0) {
            if (_opt_verbose)
                DEBUG_PRINT("bytes to fifo %d\n",bytesFifo);
            packets_tx++;
        } else {
            perror("write");
            quit();
        }

        if (vacancyPrint) {
            DEBUG_PRINT("Had to wait for tx vacancy %d usec\n",vacancyPrint*100);
            vacancyPrint = 0;
        }
    }
    printf("ethernet packets rx : %d, fifo packets tx     : %d\n",packets_rx, packets_tx);

    return (void *)0;
}

static void *fifo_rx_thread_fn(void *data)
{
    ssize_t bytesSock;
    ssize_t bytesFifo;
    int packets_rx, packets_tx;
    uint8_t buf[MAX_BUF_SIZE_BYTES];

    /* shup up compiler */
    (void)data;

    memset(&cliaddr, 0, sizeof(cliaddr));

    packets_rx = 0;
    packets_tx = 0;

    while (running) {
        bytesFifo = read(readFifoFd, buf, MAX_BUF_SIZE_BYTES);
        if (bytesFifo > 0) {
            if (_opt_verbose)
                DEBUG_PRINT("bytes from fifo %d\n",bytesFifo);
            packets_rx++;
            if (_opt_use_protocol == TCP_PROTOCOL) {
                bytesSock = write(tcpSocketFd, buf, bytesFifo);
                if (bytesSock > 0) {
                    if (_opt_verbose)
                        DEBUG_PRINT("bytes to socket %d\n",bytesSock);
                    packets_tx++;
                } else {
                    perror("write");
                    socket_write_error = 1;
                    quit();
                }
            } else if (_opt_use_protocol == UDP_PROTOCOL && udpInit) {
                bytesSock = sendto(serverFd, buf, bytesFifo,
                        MSG_CONFIRM, (const struct sockaddr *)&cliaddr,
                        cliaddrlen);
                if (bytesSock > 0) {
                    if (_opt_verbose)
                        DEBUG_PRINT("bytes to socket %d\n",bytesSock);
                    packets_tx++;
                } else {
                    perror("write");
                    socket_write_error = 1;
                    quit();
                }
            }
        }
    }
    printf("fifo packets rx     : %d, ethernet packets tx : %d\n",packets_rx, packets_tx);

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
           "  -h, --help     Print this menu\n"
           "  -t, --devTx    Device to use ... /dev/axis_fifo_0x43c10000\n"
           "  -r, --devRx    Device to use ... /dev/axis_fifo_0x43c10000\n"
           "  -p, --port     Port number to bind to\n"
           "  -x, --tcp      UDP used by default, pass this flag to use TCP\n"
           "  -v, --verbose  Turn on Debug prints, will affect performance\n\n"
           " Creates a UDP (or TCP) server that waits for data passed to the port\n"
           " and then forwards the data to the attached transmit axis fifo device.\n"
           " Data incoming from the receive axis fifo is pushed back to the client's\n"
           " IP Address and same port.\n"
           " When using UDP no data will be sent to client until a message is received.\n"
           ,
           progName
          );
}

static void print_opts()
{
    printf("Options : \n"
            "Port           : %d\n"
            "DevTX          : %s\n"
            "DevRx          : %s\n"
            "Protocol       : %s\n"
            "Verbose        : %s\n"
           ,
           _opt_sock_port,
           _opt_dev_tx,
           _opt_dev_rx,
           _opt_use_protocol ? "TCP" : "UDP",
           _opt_verbose ? "On" : "Off"
          );
}

static int process_options(int argc, char * argv[])
{
        strcpy(_opt_dev_tx,DEF_DEV_TX);
        strcpy(_opt_dev_rx,DEF_DEV_RX);
        _opt_verbose = 0;

        for (;;) {
            int option_index = 0;
            static const char *short_options = "htvr:p:x:";
            static const struct option long_options[] = {
                    {"help", no_argument, 0, 'h'},
                    {"verbose", no_argument, 0, 'v'},
                    {"tcp", no_argument, 0, 'x'},
                    {"devRx", required_argument, 0, 'r'},
                    {"devTx", required_argument, 0, 't'},
                    {"port", required_argument, 0, 'p'},
                    {0,0,0,0},
                    };

            int c = getopt_long(argc, argv, short_options,
            long_options, &option_index);

            if (c == EOF) {
            break;
            }

            switch (c) {
                case 't':
                    strcpy(_opt_dev_tx, optarg);
                    break;

                case 'r':
                    strcpy(_opt_dev_rx, optarg);
                    break;

                case 'x':
                    _opt_use_protocol = TCP_PROTOCOL;
                    break;

                case 'v':
                    _opt_verbose = 1;
                    break;

                case 'p':
                    _opt_sock_port = atoi(optarg);
                    break;

                default:
                case 'h':
                    display_help(argv[0]);
                    exit(0);
                    break;
                    }
            }

        print_opts();
        return 0;
}
