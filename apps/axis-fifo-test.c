#include <unistd.h>
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <poll.h>
#include <sys/ioctl.h>
#include "../axis-fifo.h"

/*
for testing the axis fifo driver by
simultaneously writing/reading to/from the fifo

fifo should be configured either
(1) in a "loopback" with its AXI-Stream TX interface
feeding back into it's AXI-Stream RX interface
(2) in a "pasthrough" such that one AXI-Stream fifo TX interface
is feeding into another AXI-Stream fifo's RX interface

pass in the number of words to write and the driver device file location(s)
*/

// time to wait before giving up if read/write
// returns without having sent/received any data
#define TIMEOUT 4

// number of bytes to send in a packet (max is fifo depth * 4)
#define MAX_PACKET_SIZE 8196
#define POLL_PACKET_SIZE 1024

int main(int argc, char *argv[])
{
    if (argc < 4 || argc > 5) {
        printf("usage: %s [# words] [tkeep] [read device file] [optional: write device file]\n", argv[0]);
        printf("        e.g. \"%s 1024 1 /dev/axis_fifo0\" for loopback configuration\n", argv[0]);
        printf("        e.g. \"%s 1024 1 /dev/axis_fifo0 /dev/axis_fifo1\" for passthrough configuration\n", argv[0]);
        return -1;
    }

    // transmitted data string
    char *data_string;
    unsigned data_string_len;
    int rc;

    // device file locations
    char *read_device_file;
    char *write_device_file;

    ssize_t bytes_written;
    ssize_t bytes_read;
    // file descriptors
    int f_rd;
    int f_wr;
    int tkeep;

    // first argument is how many words to write
    unsigned num_words = atoi(argv[1]);
    data_string_len = num_words*4;

    printf("START :\n\n");

    tkeep = atoi(argv[2]);
    if (!!tkeep)
        printf("TKEEP enabled ... testing byte boundary read/writes\n");
    else
        printf("TKEEP not enabled ... testing word boundary read/writes\n");

    // second / third arguments are read/write device file names
    if (argc == 4) {
        read_device_file = argv[3];
        write_device_file = argv[3];
    } else {
        read_device_file = argv[3];
        write_device_file = argv[4];
    }

    f_rd = open(read_device_file, O_RDONLY);
    f_wr = open(write_device_file, O_WRONLY);

    // test error conditions
    if (f_rd < 0) {
        printf("Open read failed with error: %s\n", strerror(errno));
        return -1;
    }
    if (f_wr < 0) {
        printf("Open write failed with error: %s\n", strerror(errno));
        return -1;
    }

    unsigned eight_bytes[2] = {0xDEADBEEF, 0xDEADBEEF};

    printf("\nTESTING error conditions...\n");
    fflush(stdout);

    // read buffer too small test
    bytes_written = write(f_wr, eight_bytes, 8);
    if (bytes_written != 8) {
        if (bytes_written == -1) {
            printf("error condition tests FAILED: write failed with code %s\n",
                strerror(errno));
        } else {
            printf("error condition tests FAILED: write failed - only wrote %i bytes\n",
                bytes_written);
        }
        return -1;
    }
    bytes_read = read(f_rd, NULL, 4);
    if (bytes_read != -1 || errno != EINVAL) {
        printf("error condition tests FAILED: didn't catch read buffer too small error (errno=%i)\n",
            errno);
        return -1;
    }

    // userland read pointer error test
    bytes_written = write(f_wr, eight_bytes, 8);
    if (bytes_written != 8) {
        if (bytes_written == -1) {
            printf("error condition tests FAILED: write failed with code %s\n",
                strerror(errno));
        } else {
            printf("error condition tests FAILED: write failed - only wrote %i bytes\n",
                bytes_written);
        }
        return -1;
    }
    bytes_read = read(f_rd, NULL, 8);
    if (bytes_read != -1 || errno != EFAULT) {
        printf("error condition tests FAILED: didn't catch userland read pointer error (%s)\n",
            strerror(errno));
        return -1;
    }

    // userland write pointer error test
    bytes_written = write(f_wr, NULL, 8);
    if (bytes_written != -1 || errno != EFAULT) {
        printf("error condition tests FAILED: didn't catch userland write pointer error (%s)\n",
            strerror(errno));
        return -1;
    }

    // write 0-length packet
    bytes_written = write(f_wr, eight_bytes, 0);
    if (bytes_written != -1 || errno != EINVAL) {
        printf("error condition tests FAILED: didn't catch 0-length packet write error (%s)\n",
            strerror(errno));
        return -1;
    }

    if (!tkeep) {
        // write misaligned packet
        // this will only fail if TKEEP is NOT enabled
        bytes_written = write(f_wr, eight_bytes, 7);
        if (bytes_written != -1 || errno != EINVAL) {
            printf("error condition tests FAILED: didn't catch misaligned packet write error (%s)\n",
                strerror(errno));
            return -1;
        }
    }

    printf("Reseting with ioctl...\n");
    rc = ioctl(f_rd, AXIS_FIFO_RESET_IP);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    rc = ioctl(f_wr, AXIS_FIFO_RESET_IP);
    if (rc) {
        perror("ioctl");
        return -1;
    }

    // write packet larger than fifo size
    unsigned big_buffer_num = 1000000;
    unsigned big_buffer[big_buffer_num];
    bytes_written = write(f_wr, big_buffer, big_buffer_num*4);
    if (bytes_written != -1 || errno != EINVAL) {
        printf("error condition tests FAILED: didn't catch oversized packet write error (%s)\n",
            strerror(errno));
        return -1;
    }

    printf("error condition tests PASSED\n");

    uint8_t bufa[MAX_PACKET_SIZE];
    uint8_t bufb[MAX_PACKET_SIZE];
    if (tkeep) {
        for(int i = 4; i < 16; i++) {
            memset(bufa,0,MAX_PACKET_SIZE);
            memset(bufb,0,MAX_PACKET_SIZE);
            for(int j = 0; j < i; j++)
                bufa[j] = j % 255;

            bytes_written = write(f_wr, bufa, i);
            bytes_read = read(f_rd, bufb, bytes_written);
            if(bytes_written != bytes_read){
                printf("non-word boundary read/write FAILED : bytes_written != bytes_read\n");
                return -1;
            }

            for(int j = 0; j < i; j++) {
                if (bufa[j] != bufb[j]) {
                    printf("\tbufa[%d]=0x%x != bufb[%d]=0x%x\n",
                        j,bufa[j],j,bufb[j]);
                    printf("non-word boundary read/write FAILED : with bytes size = %d ...\n",i);
                    return -1;
                }
            }
        }
        printf("non-word boundary read/write test PASSED\n");
    }

    // test poll subsystem
    struct pollfd fds[2];
    int pollTimeoutSec = 1;
    int pollrc;
    uint32_t minPktBak;
    uint32_t maxPktBak;
    fds[0].fd = f_rd;
    fds[1].fd = f_wr;
    fds[0].events = POLLIN;
    fds[1].events = POLLOUT;
    uint32_t minPkt = 255;
    uint32_t maxPkt = 257;

    /* initializing rx-min-pkt-size and tx-max-pkt-size */
    /* back up curent values */
    rc = ioctl(f_rd, AXIS_FIFO_GET_RX_MIN_PKT, &minPktBak);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    rc = ioctl(f_wr, AXIS_FIFO_GET_TX_MAX_PKT, &maxPktBak);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    /* update values for this test */
    rc = ioctl(f_rd, AXIS_FIFO_SET_RX_MIN_PKT, &minPkt);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    rc = ioctl(f_wr, AXIS_FIFO_SET_TX_MAX_PKT, &maxPkt);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    /* reset core */
    rc = ioctl(f_rd, AXIS_FIFO_RESET_IP);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    rc = ioctl(f_wr, AXIS_FIFO_RESET_IP);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    printf("running poll test ... will take ~%d seconds\n",4*pollTimeoutSec);

    /* check false positive read */
    while (1) {
        pollrc = poll(&fds[0],1,pollTimeoutSec*1000);
        if (pollrc < 0) {
            printf("poll test FAILED : \n");
            fflush(stdout);
            perror("poll");
            return -1;
        } if (pollrc == 0) {
            printf("\tread false pos pass\n");
            break;
        } else {
            if (fds[0].revents & POLLIN) {
                printf("poll test FAILED : claimed there was data to read when there was not\n");
                return -1;
            }
        }
    }

    /* fill fifo */
    bytes_written = 0;
    while (1) {
        pollrc = poll(&fds[1],1,pollTimeoutSec*1000);
        if (pollrc > 0) {
            if (fds[1].revents & POLLOUT) {
                rc = write(f_wr, big_buffer, POLL_PACKET_SIZE);
                if (rc > 0) {
                    bytes_written += rc;
                } else {
                    printf("poll test FAILED : ");
                    fflush(stdout);
                    perror("write");
                return -1;
                }
            }
        } else if (pollrc == 0) {
            printf("\twrite false positive pass\n");
            break;
        } else {
            perror("poll");
            return -1;
        }
    }

    /* empty out fifo */
    bytes_read = 0;
    while (1) {
        pollrc = poll(&fds[0],1,pollTimeoutSec*1000);
        if (pollrc < 0) {
            printf("poll test FAILED : ");
            fflush(stdout);
            perror("poll");
            return -1;
        } if (pollrc == 0) {
            break;
        } else {
            if ((fds[0].revents & POLLIN)) {
                rc = read(f_rd, bufb, POLL_PACKET_SIZE);
                if (rc >= 0){
                    bytes_read += rc;
                } else {
                    printf("poll test FAILED : ");
                    fflush(stdout);
                    perror("read");
                    return -1;
                }
            }
        }
    }
    if (bytes_written != bytes_read) {
        printf("\t(bytes_written) %d != (bytes_read) %d\n",
                bytes_written, bytes_read);
        printf("\tThis occurs when a packet isnt within the range of the dts values\n"
                "\trx-min-pkt-size and tx-max-pkt-size ... this test code uses 1024 bytes\n"
                "\tso rx-min-pkt-size <= 1024/4-1 = 255\n"
                "\tand tx-max-pkt-size >= 1024/4+1 = 257\n\n");
        printf("poll test FAILED\n");
        return -1;
    }
    printf("\t(bytes_written) %d == (bytes_read) %d\n",
                bytes_written, bytes_read);
    printf("poll test PASSED\n");


    /* restore original min/max pkt sizes */
    rc = ioctl(f_rd, AXIS_FIFO_SET_RX_MIN_PKT, &minPktBak);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    rc = ioctl(f_wr, AXIS_FIFO_SET_TX_MAX_PKT, &maxPktBak);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    /* reset cores */
    rc = ioctl(f_rd, AXIS_FIFO_RESET_IP);
    if (rc) {
        perror("ioctl");
        return -1;
    }
    rc = ioctl(f_wr, AXIS_FIFO_RESET_IP);
    if (rc) {
        perror("ioctl");
        return -1;
    }

    close(f_rd);
    close(f_wr);

    data_string = (char *)malloc(data_string_len);

    // seed random number generator
    srand(time(NULL));

    printf("generating byte stream...\n");

    // generate random string of numbers
    // e.g. 3 words 111155558888
    for (unsigned i = 0; i < num_words; i++) {
        unsigned r = rand() % 10 + '0';
        data_string[i*4 + 0] = (char)r;
        data_string[i*4 + 1] = (char)r;
        data_string[i*4 + 2] = (char)r;
        data_string[i*4 + 3] = (char)r;
    }

    printf("\ttransferring %i words\n", num_words);

    struct timespec start_time, stop_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    // fork into a reader/writer process
    pid_t pid = fork();
    if (pid == -1) {
        printf("fork failed: %s\n", strerror(errno));
        free(data_string);
        return EXIT_FAILURE;
    }
    else if (pid == 0) {
        // reader (child) process

        int f = open(read_device_file, O_RDONLY);

        if (f < 0) {
            printf("read open failed with code '%s'\n",
                strerror(errno));
            free(data_string);
            _exit(EXIT_FAILURE);
        }

        time_t read_timeout = time(NULL) + TIMEOUT;

        // read buffer
        char *read_data = (char *)malloc(MAX_PACKET_SIZE);

        unsigned read_offset = 0;
        unsigned bytes_remaining = data_string_len;
        while (bytes_remaining) {

            // read from device until we get all
            // the bytes sent or a timeout occurs
            bytes_read = read(f, read_data, bytes_remaining);
            if (bytes_read > 0) {
                if (memcmp(read_data, data_string + read_offset,
                        bytes_read) != 0) {
                    printf("\nread failed - data corruption\n");
                    close(f);
                    free(data_string);
                    free(read_data);
                    _exit(EXIT_FAILURE);
                }
                bytes_remaining -= bytes_read;
                read_offset += bytes_read;
                read_timeout = time(NULL) + TIMEOUT;
            } else if (bytes_read < 0) {
                // read error
                printf("\nread failed with code '%s'\n",
                    strerror(errno));
                close(f);
                free(data_string);
                free(read_data);
                _exit(EXIT_FAILURE);
            }

            if (time(NULL) > read_timeout) {
                // timeout
                printf("\nread timed out\n");
                close(f);
                free(data_string);
                free(read_data);
                _exit(EXIT_FAILURE);
            }
        }

        // success
        close(f);
    }
    else {
        // writer process

        int f = open(write_device_file, O_WRONLY);

        if (f < 0) {
            printf("write open failed with code '%s'\n",
                strerror(errno));
            free(data_string);
            return EXIT_FAILURE;
        }

        time_t write_timeout = time(NULL) + TIMEOUT;

        // write data_string to file
        unsigned bytes_remaining = data_string_len;
        char *bytes_to_write = data_string;

        unsigned write_packet_size;
        while (bytes_remaining) {

            write_packet_size = MAX_PACKET_SIZE;
            bytes_written = write(f, bytes_to_write,
                    bytes_remaining > write_packet_size ?
                    write_packet_size :
                    bytes_remaining);
            if (bytes_written > 0) {
                bytes_remaining -= bytes_written;
                bytes_to_write += bytes_written;
                write_timeout = time(NULL) + TIMEOUT;
                printf("\rtransfer %0.1f%% complete   ",
                    100 * (1 - (float)bytes_remaining /
                    (float)data_string_len));
            } else if (bytes_written < 0) {
                printf("\nwrite failed with code '%s' " \
                    "write_packet_size=%i\n",
                    strerror(errno), write_packet_size);
                close(f);
                break;
            } else if (time(NULL) > write_timeout) {
                // timeout
                printf("\nwrite timed out\n");
                close(f);
                break;
            }
        }

        printf("\n");

        // wait for child process to complete and collect return status
        int status;
        (void)waitpid(pid, &status, 0);

        clock_gettime(CLOCK_MONOTONIC, &stop_time);
        float runtime = (float)(stop_time.tv_sec - start_time.tv_sec) +
                (float)(stop_time.tv_nsec - start_time.tv_nsec)
                / 1000000000.0;

        if (WIFEXITED(status)) {
            // exited normally
            if (WEXITSTATUS(status) == EXIT_SUCCESS) {
                printf("transfer succeeded after %.3f seconds\n",
                    runtime);
                printf("transfer speed %.3fMB/s\n",
                    (float)data_string_len / runtime /
                    1024.0 / 1024.0);
            } else {
                printf("transfer failed after %.3f seconds\n",
                    runtime);
            }
        } else {
            printf("transfer failed after %.3f seconds\n",
                runtime);
        }

        close(f);
        free(data_string);
    }

    return EXIT_SUCCESS;
}
