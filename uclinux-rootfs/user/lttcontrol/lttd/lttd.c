/* lttd
 *
 * Linux Trace Toolkit Daemon
 *
 * This is a simple daemon that reads a few relay+debugfs channels and save
 * them in a trace.
 *
 * CPU hot-plugging is supported using inotify.
 *
 * Copyright 2005 -
 * 	Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#define _REENTRANT
#define _GNU_SOURCE
#include <features.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/mman.h>
#include <signal.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <asm/ioctls.h>

#include <linux/version.h>

/* Relayfs IOCTL */
#include <asm/ioctl.h>
#include <asm/types.h>

/* Get the next sub buffer that can be read. */
#define RELAY_GET_SB		_IOR(0xF5, 0x00,__u32)
/* Release the oldest reserved (by "get") sub buffer. */
#define RELAY_PUT_SB		_IOW(0xF5, 0x01,__u32)
/* returns the number of sub buffers in the per cpu channel. */
#define RELAY_GET_N_SB		_IOR(0xF5, 0x02,__u32)
/* returns the size of the current sub buffer. */
#define RELAY_GET_SB_SIZE	_IOR(0xF5, 0x03, __u32)
/* returns the size of data to consume in the current sub-buffer. */
#define RELAY_GET_MAX_SB_SIZE	_IOR(0xF5, 0x04, __u32)

# define SPLICE_F_MOVE                1       /* Move pages instead of copying.  */
# define SPLICE_F_MORE                4       /* Expect more data.  */

#include <sys/syscall.h>
#define splice(...)      syscall(__NR_splice, __VA_ARGS__)



#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,14)
#include <sys/inotify.h>
#if 0	/* should now be provided by libc. */
/* From the inotify-tools 2.6 package */
static inline int inotify_init (void)
{
	return syscall (__NR_inotify_init);
}

static inline int inotify_add_watch (int fd, const char *name, __u32 mask)
{
	return syscall (__NR_inotify_add_watch, fd, name, mask);
}

static inline int inotify_rm_watch (int fd, __u32 wd)
{
	return syscall (__NR_inotify_rm_watch, fd, wd);
}
#endif //0
#define HAS_INOTIFY
#else
static inline int inotify_init (void)
{
	return -1;
}

static inline int inotify_add_watch (int fd, const char *name, __u32 mask)
{
	return 0;
}

static inline int inotify_rm_watch (int fd, __u32 wd)
{
	return 0;
}
#undef HAS_INOTIFY
#endif

struct fd_pair {
	int channel;
	int trace;
	unsigned int n_sb;
	unsigned int max_sb_size;
	void *mmap;
	pthread_mutex_t	mutex;
};

struct channel_trace_fd {
	struct fd_pair *pair;
	int num_pairs;
};

struct inotify_watch {
	int wd;
	char path_channel[PATH_MAX];
	char path_trace[PATH_MAX];
};

struct inotify_watch_array {
	struct inotify_watch *elem;
	int num;
};

static __thread int thread_pipe[2];

struct channel_trace_fd fd_pairs = { NULL, 0 };
int inotify_fd = -1;
struct inotify_watch_array inotify_watch_array = { NULL, 0 };

/* protects fd_pairs and inotify_watch_array */
pthread_rwlock_t fd_pairs_lock = PTHREAD_RWLOCK_INITIALIZER;


static char		*trace_name = NULL;
static char		*channel_name = NULL;
static int		daemon_mode = 0;
static int		append_mode = 0;
static unsigned long	num_threads = 1;
volatile static int	quit_program = 0;	/* For signal handler */
static int		dump_flight_only = 0;
static int		dump_normal_only = 0;
static int		verbose_mode = 0;

#define printf_verbose(fmt, args...) \
  do {                               \
    if (verbose_mode)                \
      printf(fmt, ##args);           \
  } while (0)

/* Args :
 *
 * -t directory		Directory name of the trace to write to. Will be created.
 * -c directory		Root directory of the debugfs trace channels.
 * -d          		Run in background (daemon).
 * -a			Trace append mode.
 * -s			Send SIGUSR1 to parent when ready for IO.
 */
void show_arguments(void)
{
	printf("Please use the following arguments :\n");
	printf("\n");
	printf("-t directory  Directory name of the trace to write to.\n"
				 "              It will be created.\n");
	printf("-c directory  Root directory of the debugfs trace channels.\n");
	printf("-d            Run in background (daemon).\n");
	printf("-a            Append to an possibly existing trace.\n");
	printf("-N            Number of threads to start.\n");
	printf("-f            Dump only flight recorder channels.\n");
	printf("-n            Dump only normal channels.\n");
	printf("-v            Verbose mode.\n");
	printf("\n");
}


/* parse_arguments
 *
 * Parses the command line arguments.
 *
 * Returns 1 if the arguments were correct, but doesn't ask for program
 * continuation. Returns -1 if the arguments are incorrect, or 0 if OK.
 */
int parse_arguments(int argc, char **argv)
{
	int ret = 0;
	int argn = 1;
	
	if(argc == 2) {
		if(strcmp(argv[1], "-h") == 0) {
			return 1;
		}
	}

	while(argn < argc) {

		switch(argv[argn][0]) {
			case '-':
				switch(argv[argn][1]) {
					case 't':
						if(argn+1 < argc) {
							trace_name = argv[argn+1];
							argn++;
						}
						break;
					case 'c':
						if(argn+1 < argc) {
							channel_name = argv[argn+1];
							argn++;
						}
						break;
					case 'd':
						daemon_mode = 1;
						break;
					case 'a':
						append_mode = 1;
						break;
					case 'N':
						if(argn+1 < argc) {
							num_threads = strtoul(argv[argn+1], NULL, 0);
							argn++;
						}
						break;
					case 'f':
						dump_flight_only = 1;
						break;
					case 'n':
						dump_normal_only = 1;
						break;
					case 'v':
						verbose_mode = 1;
						break;
					default:
						printf("Invalid argument '%s'.\n", argv[argn]);
						printf("\n");
						ret = -1;
				}
				break;
			default:
				printf("Invalid argument '%s'.\n", argv[argn]);
				printf("\n");
				ret = -1;
		}
		argn++;
	}
	
	if(trace_name == NULL) {
		printf("Please specify a trace name.\n");
		printf("\n");
		ret = -1;
	}
	
	if(channel_name == NULL) {
		printf("Please specify a channel name.\n");
		printf("\n");
		ret = -1;
	}
	
	return ret;
}

void show_info(void)
{
	printf("Linux Trace Toolkit Trace Daemon " VERSION "\n");
	printf("\n");
	printf("Reading from debugfs directory : %s\n", channel_name);
	printf("Writing to trace directory : %s\n", trace_name);
	printf("\n");
}


/* signal handling */

static void handler(int signo)
{
	printf("Signal %d received : exiting cleanly\n", signo);
	quit_program = 1;
}


int open_buffer_file(char *filename, char *path_channel, char *path_trace,
	struct channel_trace_fd *fd_pairs)
{
	int open_ret = 0;
	int ret = 0;
	struct stat stat_buf;

	if(strncmp(filename, "flight-", sizeof("flight-")-1) != 0) {
		if(dump_flight_only) {
			printf_verbose("Skipping normal channel %s\n",
				path_channel);
			return 0;
		}
	} else {
		if(dump_normal_only) {
			printf_verbose("Skipping flight channel %s\n",
				path_channel);
			return 0;
		}
	}
	printf_verbose("Opening file.\n");
	
	fd_pairs->pair = realloc(fd_pairs->pair,
			++fd_pairs->num_pairs * sizeof(struct fd_pair));

	/* Open the channel in read mode */
	fd_pairs->pair[fd_pairs->num_pairs-1].channel = 
		open(path_channel, O_RDONLY | O_NONBLOCK);
	if(fd_pairs->pair[fd_pairs->num_pairs-1].channel == -1) {
		perror(path_channel);
		fd_pairs->num_pairs--;
		return 0;	/* continue */
	}
	/* Open the trace in write mode, only append if append_mode */
	ret = stat(path_trace, &stat_buf);
	if(ret == 0) {
		if(append_mode) {
			printf_verbose("Appending to file %s as requested\n",
				path_trace);

			fd_pairs->pair[fd_pairs->num_pairs-1].trace = 
				open(path_trace, O_WRONLY,
						S_IRWXU|S_IRWXG|S_IRWXO);
			if(fd_pairs->pair[fd_pairs->num_pairs-1].trace == -1) {
				perror(path_trace);
				open_ret = -1;
				close(fd_pairs->pair[fd_pairs->num_pairs-1].channel);
				fd_pairs->num_pairs--;
				goto end;
			}
			ret = lseek(fd_pairs->pair[fd_pairs->num_pairs-1].trace,
				    0, SEEK_END);
			if (ret < 0) {
				perror(path_trace);
				open_ret = -1;
				close(fd_pairs->pair[fd_pairs->num_pairs-1].channel);
				close(fd_pairs->pair[fd_pairs->num_pairs-1].trace);
				fd_pairs->num_pairs--;
				goto end;
			}
		} else {
			printf("File %s exists, cannot open. Try append mode.\n", path_trace);
			open_ret = -1;
			close(fd_pairs->pair[fd_pairs->num_pairs-1].channel);
			fd_pairs->num_pairs--;
			goto end;
		}
	} else {
		if(errno == ENOENT) {
			fd_pairs->pair[fd_pairs->num_pairs-1].trace = 
				open(path_trace, O_WRONLY|O_CREAT|O_EXCL,
						S_IRWXU|S_IRWXG|S_IRWXO);
			if(fd_pairs->pair[fd_pairs->num_pairs-1].trace == -1) {
				perror(path_trace);
				open_ret = -1;
				close(fd_pairs->pair[fd_pairs->num_pairs-1].channel);
				fd_pairs->num_pairs--;
				goto end;
			}
		}
	}
end:
	return open_ret;
}

int open_channel_trace_pairs(char *subchannel_name, char *subtrace_name,
		struct channel_trace_fd *fd_pairs, int *inotify_fd,
		struct inotify_watch_array *iwatch_array)
{
	DIR *channel_dir = opendir(subchannel_name);
	struct dirent *entry;
	struct stat stat_buf;
	int ret;
	char path_channel[PATH_MAX];
	int path_channel_len;
	char *path_channel_ptr;
	char path_trace[PATH_MAX];
	int path_trace_len;
	char *path_trace_ptr;
	int open_ret = 0;

	if(channel_dir == NULL) {
		perror(subchannel_name);
		open_ret = ENOENT;
		goto end;
	}

	printf_verbose("Creating trace subdirectory %s\n", subtrace_name);
	ret = mkdir(subtrace_name, S_IRWXU|S_IRWXG|S_IRWXO);
	if(ret == -1) {
		if(errno != EEXIST) {
			perror(subtrace_name);
			open_ret = -1;
			goto end;
		}
	}

	strncpy(path_channel, subchannel_name, PATH_MAX-1);
	path_channel_len = strlen(path_channel);
	path_channel[path_channel_len] = '/';
	path_channel_len++;
	path_channel_ptr = path_channel + path_channel_len;

	strncpy(path_trace, subtrace_name, PATH_MAX-1);
	path_trace_len = strlen(path_trace);
	path_trace[path_trace_len] = '/';
	path_trace_len++;
	path_trace_ptr = path_trace + path_trace_len;
	
#ifdef HAS_INOTIFY
	iwatch_array->elem = realloc(iwatch_array->elem,
		++iwatch_array->num * sizeof(struct inotify_watch));
	
	printf_verbose("Adding inotify for channel %s\n", path_channel);
	iwatch_array->elem[iwatch_array->num-1].wd = inotify_add_watch(*inotify_fd, path_channel, IN_CREATE);
	strcpy(iwatch_array->elem[iwatch_array->num-1].path_channel, path_channel);
	strcpy(iwatch_array->elem[iwatch_array->num-1].path_trace, path_trace);
	printf_verbose("Added inotify for channel %s, wd %u\n",
		iwatch_array->elem[iwatch_array->num-1].path_channel,
		iwatch_array->elem[iwatch_array->num-1].wd);
#endif

	while((entry = readdir(channel_dir)) != NULL) {

		if(entry->d_name[0] == '.') continue;
		
		strncpy(path_channel_ptr, entry->d_name, PATH_MAX - path_channel_len);
		strncpy(path_trace_ptr, entry->d_name, PATH_MAX - path_trace_len);
		
		ret = stat(path_channel, &stat_buf);
		if(ret == -1) {
			perror(path_channel);
			continue;
		}
		
		printf_verbose("Channel file : %s\n", path_channel);
		
		if(S_ISDIR(stat_buf.st_mode)) {

			printf_verbose("Entering channel subdirectory...\n");
			ret = open_channel_trace_pairs(path_channel, path_trace, fd_pairs,
				inotify_fd, iwatch_array);
			if(ret < 0) continue;
		} else if(S_ISREG(stat_buf.st_mode)) {
			open_ret = open_buffer_file(entry->d_name, path_channel, path_trace,
				fd_pairs);
			if(open_ret)
				goto end;
		}
	}
	
end:
	closedir(channel_dir);

	return open_ret;
}


int read_subbuffer(struct fd_pair *pair)
{
	unsigned int consumed_old, len;
	int err;
	long ret;
	off_t offset;


	err = ioctl(pair->channel, RELAY_GET_SB, &consumed_old);
	printf_verbose("cookie : %u\n", consumed_old);
	if(err != 0) {
		ret = errno;
		perror("Reserving sub buffer failed (everything is normal, it is due to concurrency)");
		goto get_error;
	}
#if 0
	err = TEMP_FAILURE_RETRY(write(pair->trace,
				pair->mmap 
					+ (consumed_old & ((pair->n_subbufs * pair->subbuf_size)-1)),
				pair->subbuf_size));

	if(err < 0) {
		ret = errno;
		perror("Error in writing to file");
		goto write_error;
	}
#endif //0
	err = ioctl(pair->channel, RELAY_GET_SB_SIZE, &len);
	if(err != 0) {
		ret = errno;
		perror("Getting sub-buffer len failed.");
		goto get_error;
	}

	offset = 0;
	while (len > 0) {
		printf_verbose("splice chan to pipe offset %lu\n",
			(unsigned long)offset);
		ret = splice(pair->channel, &offset, thread_pipe[1], NULL,
			len, SPLICE_F_MOVE | SPLICE_F_MORE);
		printf_verbose("splice chan to pipe ret %ld\n", ret);
		if (ret < 0) {
			perror("Error in relay splice");
			goto write_error;
		}
		ret = splice(thread_pipe[0], NULL, pair->trace, NULL,
			ret, SPLICE_F_MOVE | SPLICE_F_MORE);
		printf_verbose("splice pipe to file %ld\n", ret);
		if (ret < 0) {
			perror("Error in file splice");
			goto write_error;
		}
		len -= ret;
	}

#if 0
	err = fsync(pair->trace);
	if(err < 0) {
		ret = errno;
		perror("Error in writing to file");
		goto write_error;
	}
#endif //0
write_error:
	ret = 0;
	err = ioctl(pair->channel, RELAY_PUT_SB, &consumed_old);
	if(err != 0) {
		ret = errno;
		if(errno == EFAULT) {
			perror("Error in unreserving sub buffer\n");
		} else if(errno == EIO) {
			/* Should never happen with newer LTTng versions */
			perror("Reader has been pushed by the writer, last sub-buffer corrupted.");
		}
		goto get_error;
	}

get_error:
	return ret;
}


int map_channels(struct channel_trace_fd *fd_pairs,
	int idx_begin, int idx_end)
{
	int i,j;
	int ret=0;

	if(fd_pairs->num_pairs <= 0) {
		printf("No channel to read\n");
		goto end;
	}
	
	/* Get the subbuf sizes and number */

	for(i=idx_begin;i<idx_end;i++) {
		struct fd_pair *pair = &fd_pairs->pair[i];

		ret = ioctl(pair->channel, RELAY_GET_N_SB, &pair->n_sb);
		if(ret != 0) {
			perror("Error in getting the number of sub-buffers");
			goto end;
		}
		ret = ioctl(pair->channel, RELAY_GET_MAX_SB_SIZE, 
			    &pair->max_sb_size);
		if(ret != 0) {
			perror("Error in getting the max sub-buffer size");
			goto end;
		}
		ret = pthread_mutex_init(&pair->mutex, NULL);	/* Fast mutex */
		if(ret != 0) {
			perror("Error in mutex init");
			goto end;
		}
	}

#if 0
	/* Mmap each FD */
	for(i=idx_begin;i<idx_end;i++) {
		struct fd_pair *pair = &fd_pairs->pair[i];

		pair->mmap = mmap(0, pair->subbuf_size * pair->n_subbufs, PROT_READ,
				MAP_SHARED, pair->channel, 0);
		if(pair->mmap == MAP_FAILED) {
			perror("Mmap error");
			goto munmap;
		}
	}

	goto end; /* success */

	/* Error handling */
	/* munmap only the successfully mmapped indexes */
munmap:
		/* Munmap each FD */
	for(j=idx_begin;j<i;j++) {
		struct fd_pair *pair = &fd_pairs->pair[j];
		int err_ret;

		err_ret = munmap(pair->mmap, pair->subbuf_size * pair->n_subbufs);
		if(err_ret != 0) {
			perror("Error in munmap");
		}
		ret |= err_ret;
	}

#endif //0
end:
	return ret;
}

int unmap_channels(struct channel_trace_fd *fd_pairs)
{
	int j;
	int ret=0;

	/* Munmap each FD */
	for(j=0;j<fd_pairs->num_pairs;j++) {
		struct fd_pair *pair = &fd_pairs->pair[j];
		int err_ret;

#if 0
		err_ret = munmap(pair->mmap, pair->subbuf_size * pair->n_subbufs);
		if(err_ret != 0) {
			perror("Error in munmap");
		}
		ret |= err_ret;
#endif //0
		err_ret = pthread_mutex_destroy(&pair->mutex);
		if(err_ret != 0) {
			perror("Error in mutex destroy");
		}
		ret |= err_ret;
	}

	return ret;
}

#ifdef HAS_INOTIFY
/* Inotify event arrived.
 *
 * Only support add file for now.
 */

int read_inotify(int inotify_fd,
	struct channel_trace_fd *fd_pairs,
	struct inotify_watch_array *iwatch_array)
{
	char buf[sizeof(struct inotify_event) + PATH_MAX];
	char path_channel[PATH_MAX];
	char path_trace[PATH_MAX];
	ssize_t len;
	struct inotify_event *ievent;
	size_t offset;
	unsigned int i;
	int ret;
	int old_num;
	
	offset = 0;
	len = read(inotify_fd, buf, sizeof(struct inotify_event) + PATH_MAX);
	if(len < 0) {

		if(errno == EAGAIN)
			return 0;  /* another thread got the data before us */

		printf("Error in read from inotify FD %s.\n", strerror(len));
		return -1;
	}
	while(offset < len) {
		ievent = (struct inotify_event *)&(buf[offset]);
		for(i=0; i<iwatch_array->num; i++) {
			if(iwatch_array->elem[i].wd == ievent->wd &&
				ievent->mask == IN_CREATE) {
				printf_verbose(
					"inotify wd %u event mask : %u for %s%s\n",
					ievent->wd, ievent->mask,
					iwatch_array->elem[i].path_channel,
					ievent->name);
				old_num = fd_pairs->num_pairs;
				strcpy(path_channel, iwatch_array->elem[i].path_channel);
				strcat(path_channel, ievent->name);
				strcpy(path_trace, iwatch_array->elem[i].path_trace);
				strcat(path_trace, ievent->name);
				if(ret = open_buffer_file(ievent->name, path_channel,
					path_trace, fd_pairs)) {
					printf("Error opening buffer file\n");
					return -1;
				}
				if(ret = map_channels(fd_pairs, old_num, fd_pairs->num_pairs)) {
					printf("Error mapping channel\n");
					return -1;
				}

			}
		}
		offset += sizeof(*ievent) + ievent->len;
	}
}
#endif //HAS_INOTIFY

/* read_channels
 *
 * Thread worker.
 *
 * Read the debugfs channels and write them in the paired tracefiles.
 *
 * @fd_pairs : paired channels and trace files.
 *
 * returns 0 on success, -1 on error.
 *
 * Note that the high priority polled channels are consumed first. We then poll
 * again to see if these channels are still in priority. Only when no
 * high priority channel is left, we start reading low priority channels.
 *
 * Note that a channel is considered high priority when the buffer is almost
 * full.
 */

int read_channels(unsigned long thread_num, struct channel_trace_fd *fd_pairs,
	int inotify_fd, struct inotify_watch_array *iwatch_array)
{
	struct pollfd *pollfd = NULL;
	int num_pollfd;
	int i,j;
	int num_rdy, num_hup;
	int high_prio;
	int ret = 0;
	int inotify_fds;
	unsigned int old_num;

#ifdef HAS_INOTIFY
	inotify_fds = 1;
#else
	inotify_fds = 0;
#endif

	pthread_rwlock_rdlock(&fd_pairs_lock);

	/* Start polling the FD. Keep one fd for inotify */
	pollfd = malloc((inotify_fds + fd_pairs->num_pairs) * sizeof(struct pollfd));

#ifdef HAS_INOTIFY
	pollfd[0].fd = inotify_fd;
	pollfd[0].events = POLLIN|POLLPRI;
#endif

	for(i=0;i<fd_pairs->num_pairs;i++) {
		pollfd[inotify_fds+i].fd = fd_pairs->pair[i].channel;
		pollfd[inotify_fds+i].events = POLLIN|POLLPRI;
	}
	num_pollfd = inotify_fds + fd_pairs->num_pairs;


	pthread_rwlock_unlock(&fd_pairs_lock);

	while(1) {
		high_prio = 0;
		num_hup = 0; 
#ifdef DEBUG
		printf("Press a key for next poll...\n");
		char buf[1];
		read(STDIN_FILENO, &buf, 1);
		printf("Next poll (polling %d fd) :\n", num_pollfd);
#endif //DEBUG

		/* Have we received a signal ? */
		if(quit_program) break;
		
		num_rdy = poll(pollfd, num_pollfd, -1);

		if(num_rdy == -1) {
			perror("Poll error");
			goto free_fd;
		}

		printf_verbose("Data received\n");
#ifdef HAS_INOTIFY
		switch(pollfd[0].revents) {
			case POLLERR:
				printf_verbose(
					"Error returned in polling inotify fd %d.\n",
					pollfd[0].fd);
				break;
			case POLLHUP:
				printf_verbose(
					"Polling inotify fd %d tells it has hung up.\n",
					pollfd[0].fd);
				break;
			case POLLNVAL:
				printf_verbose(
					"Polling inotify fd %d tells fd is not open.\n",
					pollfd[0].fd);
				break;
			case POLLPRI:
			case POLLIN:
				printf_verbose(
					"Polling inotify fd %d : data ready.\n",
					pollfd[0].fd);

				pthread_rwlock_wrlock(&fd_pairs_lock);
				read_inotify(inotify_fd, fd_pairs, iwatch_array);
				pthread_rwlock_unlock(&fd_pairs_lock);

			break;
		}
#endif

		for(i=inotify_fds;i<num_pollfd;i++) {
			switch(pollfd[i].revents) {
				case POLLERR:
					printf_verbose(
						"Error returned in polling fd %d.\n",
						pollfd[i].fd);
					num_hup++;
					break;
				case POLLHUP:
					printf_verbose(
						"Polling fd %d tells it has hung up.\n",
						pollfd[i].fd);
					num_hup++;
					break;
				case POLLNVAL:
					printf_verbose(
						"Polling fd %d tells fd is not open.\n",
						pollfd[i].fd);
					num_hup++;
					break;
				case POLLPRI:
					pthread_rwlock_rdlock(&fd_pairs_lock);
					if(pthread_mutex_trylock(&fd_pairs->pair[i-inotify_fds].mutex) == 0) {
						printf_verbose(
							"Urgent read on fd %d\n",
							pollfd[i].fd);
						/* Take care of high priority channels first. */
						high_prio = 1;
						/* it's ok to have an unavailable sub-buffer */
						ret = read_subbuffer(&fd_pairs->pair[i-inotify_fds]);
						if(ret == EAGAIN) ret = 0;

						ret = pthread_mutex_unlock(&fd_pairs->pair[i-inotify_fds].mutex);
						if(ret)
							printf("Error in mutex unlock : %s\n", strerror(ret));
					}
					pthread_rwlock_unlock(&fd_pairs_lock);
					break;
			}
		}
		/* If every buffer FD has hung up, we end the read loop here */
		if(num_hup == num_pollfd - inotify_fds) break;

		if(!high_prio) {
			for(i=inotify_fds;i<num_pollfd;i++) {
				switch(pollfd[i].revents) {
					case POLLIN:
						pthread_rwlock_rdlock(&fd_pairs_lock);
						if(pthread_mutex_trylock(&fd_pairs->pair[i-inotify_fds].mutex) == 0) {
							/* Take care of low priority channels. */
							printf_verbose(
								"Normal read on fd %d\n",
								pollfd[i].fd);
							/* it's ok to have an unavailable subbuffer */
							ret = read_subbuffer(&fd_pairs->pair[i-inotify_fds]);
							if(ret == EAGAIN) ret = 0;

							ret = pthread_mutex_unlock(&fd_pairs->pair[i-inotify_fds].mutex);
							if(ret)
								printf("Error in mutex unlock : %s\n", strerror(ret));
						}
						pthread_rwlock_unlock(&fd_pairs_lock);
						break;
				}
			}
		}

		/* Update pollfd array if an entry was added to fd_pairs */
		pthread_rwlock_rdlock(&fd_pairs_lock);
		if((inotify_fds + fd_pairs->num_pairs) != num_pollfd) {
			pollfd = realloc(pollfd,
					(inotify_fds + fd_pairs->num_pairs) * sizeof(struct pollfd));
			for(i=num_pollfd-inotify_fds;i<fd_pairs->num_pairs;i++) {
				pollfd[inotify_fds+i].fd = fd_pairs->pair[i].channel;
				pollfd[inotify_fds+i].events = POLLIN|POLLPRI;
			}
			num_pollfd = fd_pairs->num_pairs + inotify_fds;
		}
		pthread_rwlock_unlock(&fd_pairs_lock);

		/* NB: If the fd_pairs structure is updated by another thread from this
		 *     point forward, the current thread will wait in the poll without
		 *     monitoring the new channel. However, this thread will add the
		 *     new channel on next poll (and this should not take too much time
		 *     on a loaded system).
		 *
		 *     This event is quite unlikely and can only occur if a CPU is
		 *     hot-plugged while multple lttd threads are running.
		 */
	}

free_fd:
	free(pollfd);

end:
	return ret;
}


void close_channel_trace_pairs(struct channel_trace_fd *fd_pairs, int inotify_fd,
	struct inotify_watch_array *iwatch_array)
{
	int i;
	int ret;

	for(i=0;i<fd_pairs->num_pairs;i++) {
		ret = close(fd_pairs->pair[i].channel);
		if(ret == -1) perror("Close error on channel");
		ret = close(fd_pairs->pair[i].trace);
		if(ret == -1) perror("Close error on trace");
	}
	free(fd_pairs->pair);
	free(iwatch_array->elem);
}

/* Thread worker */
void * thread_main(void *arg)
{
	long ret;
	unsigned long thread_num = (unsigned long)arg;

	ret = pipe(thread_pipe);
	if (ret < 0) {
		perror("Error creating pipe");
		return (void*)ret;
	}
	ret = read_channels(thread_num, &fd_pairs, inotify_fd, &inotify_watch_array);
	close(thread_pipe[0]);	/* close read end */
	close(thread_pipe[1]);	/* close write end */
	return (void*)ret;
}


int channels_init()
{
	int ret = 0;

	inotify_fd = inotify_init();
	fcntl(inotify_fd, F_SETFL, O_NONBLOCK);

	if(ret = open_channel_trace_pairs(channel_name, trace_name, &fd_pairs,
			&inotify_fd, &inotify_watch_array))
		goto close_channel;
	if (fd_pairs.num_pairs == 0) {
		printf("No channel available for reading, exiting\n");
		ret = -ENOENT;
		goto close_channel;
	}
	if(ret = map_channels(&fd_pairs, 0, fd_pairs.num_pairs))
		goto close_channel;
	return 0;

close_channel:
	close_channel_trace_pairs(&fd_pairs, inotify_fd, &inotify_watch_array);
	if(inotify_fd >= 0)
		close(inotify_fd);
	return ret;
}


int main(int argc, char ** argv)
{
	int ret = 0;
	struct sigaction act;
	pthread_t *tids;
	unsigned long i;
	void *tret;
	
	ret = parse_arguments(argc, argv);

	if(ret != 0) show_arguments();
	if(ret < 0) return EINVAL;
	if(ret > 0) return 0;

	show_info();

	/* Connect the signal handlers */
	act.sa_handler = handler;
	act.sa_flags = 0;
	sigemptyset(&(act.sa_mask));
	sigaddset(&(act.sa_mask), SIGTERM);
	sigaddset(&(act.sa_mask), SIGQUIT);
	sigaddset(&(act.sa_mask), SIGINT);
	sigaction(SIGTERM, &act, NULL);
	sigaction(SIGQUIT, &act, NULL);
	sigaction(SIGINT, &act, NULL);

	if(ret = channels_init())
		return ret;

	if(daemon_mode) {
		ret = daemon(0, 0);

		if(ret == -1) {
			perror("An error occured while daemonizing.");
			exit(-1);
		}
	}

	tids = malloc(sizeof(pthread_t) * num_threads);
	for(i=0; i<num_threads; i++) {

		ret = pthread_create(&tids[i], NULL, thread_main, (void*)i);
		if(ret) {
			perror("Error creating thread");
			break;
		}
	}

	for(i=0; i<num_threads; i++) {
		ret = pthread_join(tids[i], &tret);
		if(ret) {
			perror("Error joining thread");
			break;
		}
		if((long)tret != 0) {
			printf("Error %s occured in thread %u\n",
				strerror((long)tret), i);
		}
	}

	free(tids);
	ret = unmap_channels(&fd_pairs);
	close_channel_trace_pairs(&fd_pairs, inotify_fd, &inotify_watch_array);
	if(inotify_fd >= 0)
		close(inotify_fd);
			
	return ret;
}
