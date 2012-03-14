/* lttctl
 *
 * Linux Trace Toolkit Control
 *
 * Small program that controls LTT through libltt.
 *
 * Copyright 2005 -
 * 	Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 *
 * Copyright 2008 FUJITSU
 * 	Zhao Lei <zhaolei@cn.fujitsu.com>
 * 	Gui Jianfeng <guijianfeng@cn.fujitsu.com>
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <liblttctl/lttctl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <string.h>
#include <limits.h>
#define _GNU_SOURCE
#include <getopt.h>

#define OPT_MAX			(1024)
#define OPT_NAMELEN		(256)
#define OPT_VALSTRINGLEN	(256)

enum opt_type {
	CHANNEL,
};

struct channel_option {
	char chan_name[OPT_NAMELEN];
	int enable;
	int overwrite;
	int bufnum;
	int bufsize;
	int switch_timer;
};

struct lttctl_option {
	union {
		struct channel_option chan_opt;
	} opt_mode;
	enum opt_type type;
	struct lttctl_option *next;
};

struct lttctl_option *opt_head, *last_opt;

static int opt_create;
static int opt_destroy;
static int opt_start;
static int opt_pause;
static int opt_help;
static const char *opt_transport;
static const char *opt_write;
static int opt_append;
static unsigned int opt_dump_threads;
static char channel_root_default[PATH_MAX];
static const char *opt_channel_root;
static const char *opt_tracename;

/* Args :
 *
 */
static void show_arguments(void)
{
	printf("Linux Trace Toolkit Trace Control " VERSION"\n");
	printf("\n");
	printf("Usage: lttctl [OPTION]... [TRACENAME]\n");
	printf("\n");
	printf("Examples:\n");
	printf("  lttctl -c trace1                 "
		"# Create a trace named trace1.\n");
	printf("  lttctl -s trace1                 "
		"# start a trace named trace1.\n");
	printf("  lttctl -p trace1                 "
		"# pause a trace named trace1.\n");
	printf("  lttctl -d trace1                 "
		"# Destroy a trace named trace1.\n");
	printf("  lttctl -C -w /tmp/trace1 trace1  "
		"# Create a trace named trace1, start it and\n"
		"                                   "
		"# write non-overwrite channels' data to\n"
		"                                   "
		"# /tmp/trace1, debugfs must be mounted for\n"
		"                                   "
		"# auto-find\n");
	printf("  lttctl -D -w /tmp/trace1 trace1  "
		"# Pause and destroy a trace named trace1 and\n"
		"                                   "
		"# write overwrite channels' data to\n"
		"                                   "
		"# /tmp/trace1, debugfs must be mounted for\n"
		"                                   "
		"# auto-find\n");
	printf("\n");
	printf(" Basic options:\n");
	printf("  -c, --create\n");
	printf("        Create a trace.\n");
	printf("  -d, --destroy\n");
	printf("        Destroy a trace.\n");
	printf("  -s, --start\n");
	printf("        Start a trace.\n");
	printf("  -p, --pause\n");
	printf("        Pause a trace.\n");
	printf("  -h, --help\n");
	printf("        Show this help.\n");
	printf("\n");
	printf(" Advanced options:\n");
	printf("  --transport TRANSPORT\n");
	printf("        Set trace's transport. (ex. relay-locked or relay)\n");
	printf("  -o, --option OPTION\n");
	printf("        Set options, following operations are supported:\n");
	printf("        channel.<channelname>.enable=\n");
	printf("        channel.<channelname>.overwrite=\n");
	printf("        channel.<channelname>.bufnum=\n");
	printf("        channel.<channelname>.bufsize= (in bytes, rounded to "
	       "next power of 2)\n");
	printf("        <channelname> can be set to all for all channels\n");
	printf("        channel.<channelname>.switch_timer= (timer interval in "
	       "ms)\n");
	printf("\n");
	printf(" Integration options:\n");
	printf("  -C, --create_start\n");
	printf("        Create and start a trace.\n");
	printf("  -D, --pause_destroy\n");
	printf("        Pause and destroy a trace.\n");
	printf("  -w, --write PATH\n");
	printf("        Path for write trace datas.\n");
	printf("        For -c, -C, -d, -D options\n");
	printf("  -a, --append\n");
	printf("        Append to trace, For -w option\n");
	printf("  -n, --dump_threads NUMBER\n");
	printf("        Number of lttd threads, For -w option\n");
	printf("  --channel_root PATH\n");
	printf("        Set channels root path, For -w option."
		" (ex. /mnt/debugfs/ltt)\n");
	printf("\n");
}

/*
 * Separate option name to 3 fields
 * Ex:
 *  Input: name = channel.cpu.bufsize
 *  Output: name1 = channel
 *          name2 = cpu
 *          name3 = bufsize
 *  Ret: 0 on success
 *       1 on fail
 *
 * Note:
 *  Make sure that name1~3 longer than OPT_NAMELEN.
 *  name1~3 can be NULL to discard value
 *
 */
static int separate_opt(const char *name, char *name1, char *name2, char *name3)
{
	char *p;

	if (!name)
		return 1;

	/* segment1 */
	p = strchr(name, '.');
	if (!p)
		return 1;
	if (p - name >= OPT_NAMELEN)
		return 1;
	if (name1) {
		memcpy(name1, name, p - name);
		name1[p - name] = 0;
	}
	name = p + 1;

	/* segment2 */
	p = strchr(name, '.');
	if (!p)
		return 1;
	if (p - name >= OPT_NAMELEN)
		return 1;
	if (name2) {
		memcpy(name2, name, p - name);
		name2[p - name] = 0;
	}
	name = p + 1;

	/* segment3 */
	if (strlen(name) >= OPT_NAMELEN)
		return 1;
	if (name3)
		strcpy(name3, name);

	return 0;
}

static void init_channel_opt(struct channel_option *opt, char *opt_name)
{
	if (opt && opt_name) {
		opt->enable = -1;
		opt->overwrite = -1;
		opt->bufnum = -1;
		opt->bufsize = -1;
		opt->switch_timer = -1;
		strcpy(opt->chan_name, opt_name);
	}
}

static struct lttctl_option *find_insert_channel_opt(char *opt_name)
{
	struct lttctl_option *iter, *new_opt;

	if (!opt_head) {
		opt_head = (struct lttctl_option *)malloc(sizeof(struct lttctl_option));
		init_channel_opt(&opt_head->opt_mode.chan_opt, opt_name);
		opt_head->type = CHANNEL;
		opt_head->next = NULL;
		last_opt = opt_head;
		return opt_head;
	}

	for (iter = opt_head; iter; iter = iter->next) {
		if (iter->type != CHANNEL)
			continue;
		if (!strcmp(iter->opt_mode.chan_opt.chan_name, opt_name))
			return iter;
	}

	new_opt = (struct lttctl_option *)malloc(sizeof(struct lttctl_option));
	init_channel_opt(&new_opt->opt_mode.chan_opt, opt_name);
	new_opt->type = CHANNEL;
	new_opt->next = NULL;
	last_opt->next = new_opt;
	last_opt = new_opt;
	return new_opt;
}

int set_channel_opt(struct channel_option *opt, char *opt_name, char *opt_valstr)
{
	int opt_val, ret;

	if (!strcmp("enable", opt_name)) {
		if (opt_valstr[1] != 0) {
			return -EINVAL;
		}
		if (opt_valstr[0] == 'Y' || opt_valstr[0] == 'y'
		    || opt_valstr[0] == '1')
			opt_val = 1;
		else if (opt_valstr[0] == 'N' || opt_valstr[0] == 'n'
			 || opt_valstr[0] == '0')
			opt_val = 0;
		else {
			return -EINVAL;
		}

		opt->enable = opt_val;
		return 0;
	} else if (!strcmp("overwrite", opt_name)) {
		if (opt_valstr[1] != 0) {
			return -EINVAL;
		}
		if (opt_valstr[0] == 'Y' || opt_valstr[0] == 'y'
		    || opt_valstr[0] == '1')
			opt_val = 1;
		else if (opt_valstr[0] == 'N' || opt_valstr[0] == 'n'
			 || opt_valstr[0] == '0')
			opt_val = 0;
		else {
			return -EINVAL;
		}

		opt->overwrite = opt_val;
		return 0;

	} else if (!strcmp("bufnum", opt_name)) {
		ret = sscanf(opt_valstr, "%d", &opt_val);
		if (ret != 1 || opt_val < 0) {
			return -EINVAL;
		}
		
		opt->bufnum = opt_val;
		return 0;
	} else if (!strcmp("bufsize", opt_name)) {
		ret = sscanf(opt_valstr, "%d", &opt_val);
		if (ret != 1 || opt_val < 0) {
			return -EINVAL;
		}
		
		opt->bufsize = opt_val;
		return 0;
	} else if (!strcmp("switch_timer", opt_name)) {
		ret = sscanf(opt_valstr, "%d", &opt_val);
		if (ret != 1 || opt_val < 0) {
			return -EINVAL;
		}
		
		opt->switch_timer = opt_val;
		return 0;
	} else {
		return -EINVAL;
	}

}

static int parst_opt(const char *optarg)
{
	int ret;
	char opt_name[OPT_NAMELEN * 3];
	char opt_valstr[OPT_VALSTRINGLEN];
	char *p;

	char name1[OPT_NAMELEN];
	char name2[OPT_NAMELEN];
	char name3[OPT_NAMELEN];

	int opt_intval;
	int opt_val;
	unsigned int opt_uintval;
	struct lttctl_option *opt;

	if (!optarg) {
		fprintf(stderr, "Option empty\n");
		return -EINVAL;
	}

	/* Get option name and val_str */
	p = strchr(optarg, '=');
	if (!p) {
		fprintf(stderr, "Option format error: %s\n", optarg);
		return -EINVAL;
	}

	if (p - optarg >= sizeof(opt_name)/sizeof(opt_name[0])) {
		fprintf(stderr, "Option name too long: %s\n", optarg);
		return -EINVAL;
	}

	if (strlen(p+1) >= OPT_VALSTRINGLEN) {
		fprintf(stderr, "Option value too long: %s\n", optarg);
		return -EINVAL;
	}

	memcpy(opt_name, optarg, p - optarg);
	opt_name[p - optarg] = 0;
	strcpy(opt_valstr, p+1);

	/* separate option name into 3 fields */
	ret = separate_opt(opt_name, name1, name2, name3);
	if (ret != 0) {
		fprintf(stderr, "Option name error1: %s\n", optarg);
		return -EINVAL;
	}
	
	if (!strcmp("channel", name1)) {
		opt = find_insert_channel_opt(name2);
		if ((ret = set_channel_opt(&opt->opt_mode.chan_opt, 
					   name3, opt_valstr) != 0)) {
			fprintf(stderr, "Option name error2: %s\n", optarg);
			return ret;
		}
	} else {
		fprintf(stderr, "Option name error3: %s\n", optarg);
		return -EINVAL;
	}
	
	return 0;
}

/* parse_arguments
 *
 * Parses the command line arguments.
 *
 * Returns -1 if the arguments were correct, but doesn't ask for program
 * continuation. Returns EINVAL if the arguments are incorrect, or 0 if OK.
 */
static int parse_arguments(int argc, char **argv)
{
	int ret = 0;
	
	static struct option longopts[] = {
		{"create",		no_argument,		NULL,	'c'},
		{"destroy",		no_argument,		NULL,	'd'},
		{"start",		no_argument,		NULL,	's'},
		{"pause",		no_argument,		NULL,	'p'},
		{"help",		no_argument,		NULL,	'h'},
		{"transport",		required_argument,	NULL,	2},
		{"option",		required_argument,	NULL,	'o'},
		{"create_start",	no_argument,		NULL,	'C'},
		{"pause_destroy",	no_argument,		NULL,	'D'},
		{"write",		required_argument,	NULL,	'w'},
		{"append",		no_argument,		NULL,	'a'},
		{"dump_threads",	required_argument,	NULL,	'n'},
		{"channel_root",	required_argument,	NULL,	3},
		{ NULL,			0,			NULL,	0 },
	};

	/*
	 * Enable all channels in default
	 * To make novice users happy
	 */
	parst_opt("channel.all.enable=1");

	opterr = 1; /* Print error message on getopt_long */
	while (1) {
		int c;
		c = getopt_long(argc, argv, "cdspho:CDw:an:", longopts, NULL);
		if (-1 == c) {
			/* parse end */
			break;
		}
		switch (c) {
		case 'c':
			opt_create = 1;
			break;
		case 'd':
			opt_destroy = 1;
			break;
		case 's':
			opt_start = 1;
			break;
		case 'p':
			opt_pause = 1;
			break;
		case 'h':
			opt_help = 1;
			break;
		case 2:
			if (!opt_transport) {
				opt_transport = optarg;
			} else {
				fprintf(stderr,
					"Please specify only 1 transport\n");
				return -EINVAL;
			}
			break;
		case 'o':
			ret = parst_opt(optarg);
			if (ret)
				return ret;
			break;
		case 'C':
			opt_create = 1;
			opt_start = 1;
			break;
		case 'D':
			opt_pause = 1;
			opt_destroy = 1;
			break;
		case 'w':
			if (!opt_write) {
				opt_write = optarg;
			} else {
				fprintf(stderr,
					"Please specify only 1 write dir\n");
				return -EINVAL;
			}
			break;
		case 'a':
			opt_append = 1;
			break;
		case 'n':
			if (opt_dump_threads) {
				fprintf(stderr,
					"Please specify only 1 dump threads\n");
				return -EINVAL;
			}

			ret = sscanf(optarg, "%u", &opt_dump_threads);
			if (ret != 1) {
				fprintf(stderr,
					"Dump threads not positive number\n");
				return -EINVAL;
			}
			break;
		case 3:
			if (!opt_channel_root) {
				opt_channel_root = optarg;
			} else {
				fprintf(stderr,
					"Please specify only 1 channel root\n");
				return -EINVAL;
			}
			break;
		case '?':
			return -EINVAL;
		default:
			break;
		};
	};

	/* Don't check args when user needs help */
	if (opt_help)
		return 0;

	/* Get tracename */
	if (optind < argc - 1) {
		fprintf(stderr, "Please specify only 1 trace name\n");
		return -EINVAL;
	}
	if (optind > argc - 1) {
		fprintf(stderr, "Please specify trace name\n");
		return -EINVAL;
	}
	opt_tracename = argv[optind];

	/*
	 * Check arguments
	 */
	if (!opt_create && !opt_start && !opt_destroy && !opt_pause) {
		fprintf(stderr,
			"Please specify a option of "
			"create, destroy, start, or pause\n");
		return -EINVAL;
	}

	if ((opt_create || opt_start) && (opt_destroy || opt_pause)) {
		fprintf(stderr,
			"Create and start conflict with destroy and pause\n");
		return -EINVAL;
	}

	if (opt_create) {
		if (!opt_transport)
			opt_transport = "relay";
	}

	if (opt_transport) {
		if (!opt_create) {
			fprintf(stderr,
				"Transport option must be combine with create"
				" option\n");
			return -EINVAL;
		}
	}

	if (opt_write) {
		if (!opt_create && !opt_destroy) {
			fprintf(stderr,
				"Write option must be combine with create or"
				" destroy option\n");
			return -EINVAL;
		}

		if (!opt_channel_root)
			if (getdebugfsmntdir(channel_root_default) == 0) {
				strcat(channel_root_default, "/ltt");
				opt_channel_root = channel_root_default;
			} else {
				fprintf(stderr,
					"Channel_root is necessary for -w"
					" option, but neither --channel_root"
					" option\n"
					"specified, nor debugfs's mount dir"
					" found, mount debugfs also failed\n");
				return -EINVAL;
			}

		if (opt_dump_threads == 0)
			opt_dump_threads = 1;
	}

	if (opt_append) {
		if (!opt_write) {
			fprintf(stderr,
				"Append option must be combine with write"
				" option\n");
			return -EINVAL;
		}
	}

	if (opt_dump_threads) {
		if (!opt_write) {
			fprintf(stderr,
				"Dump_threads option must be combine with write"
				" option\n");
			return -EINVAL;
		}
	}

	if (opt_channel_root) {
		if (!opt_write) {
			fprintf(stderr,
				"Channel_root option must be combine with write"
				" option\n");
			return -EINVAL;
		}
	}

	return 0;
}
 
static void show_info(void)
{
	printf("Linux Trace Toolkit Trace Control " VERSION"\n");
	printf("\n");
	if (opt_tracename != NULL) {
		printf("Controlling trace : %s\n", opt_tracename);
		printf("\n");
	}
}

static int lttctl_channel_setup(struct channel_option *opt)
{
	int ret;

	if (opt->enable != -1) {
		if ((ret = lttctl_set_channel_enable(opt_tracename,
						     opt->chan_name,
						     opt->enable)) != 0)
			return ret;
	} 
	if (opt->overwrite != -1) {
		if ((ret = lttctl_set_channel_overwrite(opt_tracename,
							opt->chan_name,
							opt->overwrite)) != 0)
			return ret;
	}
	if (opt->bufnum != -1) {
		if ((ret = lttctl_set_channel_subbuf_num(opt_tracename,
							 opt->chan_name,
							 opt->bufnum)) != 0)
			return ret;
	}
	if (opt->bufsize != -1) {
		if ((ret = lttctl_set_channel_subbuf_size(opt_tracename,
							  opt->chan_name,
							  opt->bufsize)) != 0)
			return ret;
	}
	if (opt->switch_timer != -1) {
		if ((ret = lttctl_set_channel_switch_timer(opt_tracename,
				opt->chan_name, opt->switch_timer)) != 0)
			return ret;
	}

	return 0;
}

static int lttctl_create_trace(void)
{
	int ret;
	int i;
	struct lttctl_option *opt;

	ret = lttctl_setup_trace(opt_tracename);
	if (ret)
		goto setup_trace_fail;

	for (opt = opt_head; opt; opt = opt->next) {
		if (opt->type != CHANNEL)
			continue;
		ret = lttctl_channel_setup(&opt->opt_mode.chan_opt);
		if (ret)
			goto set_option_fail;;
	}

	ret = lttctl_set_trans(opt_tracename, opt_transport);
	if (ret)
		goto set_option_fail;

	ret = lttctl_alloc_trace(opt_tracename);
	if (ret)
		goto alloc_trace_fail;

	return 0;

alloc_trace_fail:
set_option_fail:
	lttctl_destroy_trace(opt_tracename);
setup_trace_fail:
	return ret;
}

/*
 * Start a lttd daemon to write trace datas
 * Dump overwrite channels on overwrite!=0
 * Dump normal(non-overwrite) channels on overwrite=0
 *
 * ret: 0 on success
 *      !0 on fail
 */
static int lttctl_daemon(int overwrite)
{
	pid_t pid;
	int status;

	pid = fork();
	if (pid < 0) {
		perror("Error in forking for lttd daemon");
		return errno;
	}

	if (pid == 0) {
		/* child */
		char *argv[16];
		int argc = 0;
		char channel_path[PATH_MAX];
		char thread_num[16];

		/* prog path */
		argv[argc] = getenv("LTT_DAEMON");
		if (argv[argc] == NULL)
			argv[argc] = PACKAGE_BIN_DIR "/lttd";
		argc++;

		/* -t option */
		argv[argc] = "-t";
		argc++;
		/*
		 * we allow modify of opt_write's content in new process
		 * for get rid of warning of assign char * to const char *
		 */
		argv[argc] = (char *)opt_write;
		argc++;

		/* -c option */
		strcpy(channel_path, opt_channel_root);
		strcat(channel_path, "/");
		strcat(channel_path, opt_tracename);
		argv[argc] = "-c";
		argc++;
		argv[argc] = channel_path;
		argc++;

		/* -N option */
		sprintf(thread_num, "%u", opt_dump_threads);
		argv[argc] = "-N";
		argc++;
		argv[argc] = thread_num;
		argc++;

		/* -a option */
		if (opt_append) {
			argv[argc] = "-a";
			argc++;
		}

		/* -d option */
		argv[argc] = "-d";
		argc++;

		/* overwrite option */
		if (overwrite) {
			argv[argc] = "-f";
			argc++;
		} else {
			argv[argc] = "-n";
			argc++;
		}

		argv[argc] = NULL;

		execvp(argv[0], argv);

		perror("Error in executing the lttd daemon");
		exit(errno);
	}

	/* parent */
	if (waitpid(pid, &status, 0) == -1) {
		perror("Error in waitpid\n");
		return errno;
	}

	if (!WIFEXITED(status)) {
		fprintf(stderr, "lttd process interrupted\n");
		return status;
	}

	if (WEXITSTATUS(status))
		fprintf(stderr, "lttd process running failed\n");

	return WEXITSTATUS(status);
}

int main(int argc, char **argv)
{
	int ret;

	ret = parse_arguments(argc, argv);
	/* If user needs show help, we disregard other options */
	if (opt_help) {
		show_arguments();
		return 0;
	}

	/* exit program if arguments wrong */
	if (ret)
		return 1;

	show_info();

	ret = lttctl_init();
	if (ret != 0)
		return ret;

	if (opt_create) {
		printf("lttctl: Creating trace\n");
		ret = lttctl_create_trace();
		if (ret)
			goto op_fail;

		if (opt_write) {
			printf("lttctl: Forking lttd\n");
			ret = lttctl_daemon(0);
			if (ret)
				goto op_fail;
		}
	}

	if (opt_start) {
		printf("lttctl: Starting trace\n");
		ret = lttctl_start(opt_tracename);
		if (ret)
			goto op_fail;
	}

	if (opt_pause) {
		printf("lttctl: Pausing trace\n");
		ret = lttctl_pause(opt_tracename);
		if (ret)
			goto op_fail;
	}

	if (opt_destroy) {
		if (opt_write) {
			printf("lttctl: Forking lttd\n");
			ret = lttctl_daemon(1);
			if (ret)
				goto op_fail;
		}

		printf("lttctl: Destroying trace\n");
		ret = lttctl_destroy_trace(opt_tracename);
		if (ret)
			goto op_fail;
	}

op_fail:
	lttctl_destroy();

	return ret;
}
