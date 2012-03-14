/*
 * Simple command-line tool to set affinity
 * 	Robert Love, 20020311
 *  Modified by Troy Trammel
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <unistd.h>
#include <linux/unistd.h>
#include <sys/syscall.h>

/*
 * provide the proper syscall information if our libc
 * is not yet updated.
 */
int __sched_setaffinity(unsigned long pid, unsigned long len, void *ptr)
{
	return syscall(__NR_sched_setaffinity, pid, len, ptr);
}

int __sched_getaffinity(unsigned long pid, unsigned long len, void *ptr)
{
	return syscall(__NR_sched_getaffinity, pid, len, ptr);
}

int main(int argc, char * argv[])
{
	unsigned long new_mask;
	unsigned long cur_mask;
	unsigned int len = sizeof(new_mask);
	pid_t pid;

	if (argc != 3) {
		printf(" usage: %s <pid> <cpu_mask>\n", argv[0]);
		return -1;
	}

	pid = atol(argv[1]);
	sscanf(argv[2], "%08lx", &new_mask);

	if (__sched_getaffinity(pid, len, &cur_mask) < 0) {
		printf("error: could not get pid %d's affinity.\n", pid);
		return -1;
	}

	printf(" pid %d's old affinity: %08lx\n", pid, cur_mask);

	if (__sched_setaffinity(pid, len, &new_mask)) {
		printf("error: could not set pid %d's affinity.\n", pid);
		return -1;
	}

	if (__sched_getaffinity(pid, len, &cur_mask) < 0) {
		printf("error: could not get pid %d's affinity.\n", pid);
		return -1;
	}

	printf(" pid %d's new affinity: %08lx\n", pid, cur_mask);

	return 0;
}
