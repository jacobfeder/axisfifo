#include <unistd.h>
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

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

int main(int argc, char *argv[])
{
	if (argc < 3 || argc > 4) {
		printf("usage: %s [# words] [read device file] [optional: write device file]\n", argv[0]);
		printf("		e.g. \"%s 1024 /dev/axis_fifo0\" for loopback configuration\n", argv[0]);
		printf("		e.g. \"%s 1024 /dev/axis_fifo0 /dev/axis_fifo1\" for passthrough configuration\n", argv[0]);
		return -1;
	}

	// transmitted data string
	char *data_string;
	unsigned data_string_len;

	// device file locations
	char *read_device_file;
	char *write_device_file;

	ssize_t bytes_written;
	ssize_t bytes_read;
	// file descriptors
	int f_rd;
	int f_wr;

	// first argument is how many words to write
	unsigned num_words = atoi(argv[1]);
	data_string_len = num_words*4;

	// second / third arguments are read/write device file names
	if (argc == 3) {
		read_device_file = argv[2];
		write_device_file = argv[2];
	} else {
		read_device_file = argv[2];
		write_device_file = argv[3];
	}

	// test error conditions

	f_rd = open(read_device_file, O_RDONLY);
	f_wr = open(write_device_file, O_WRONLY);

	if (f_rd < 0) {
		printf("Open read failed with error: %s\n", strerror(errno));
		return -1;
	}
	if (f_wr < 0) {
		printf("Open write failed with error: %s\n", strerror(errno));
		return -1;
	}

	unsigned eight_bytes[2] = {0xDEADBEEF, 0xDEADBEEF};

	printf("TESTING error conditions...\n");

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

	// write misaligned packet
	bytes_written = write(f_wr, eight_bytes, 7);
	if (bytes_written != -1 || errno != EINVAL) {
		printf("error condition tests FAILED: didn't catch misaligned packet write error (%s)\n",
			strerror(errno));
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

	printf("transferring %i words\n", num_words);

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
				printf("\rtransfer %0.1f%% complete      ",
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