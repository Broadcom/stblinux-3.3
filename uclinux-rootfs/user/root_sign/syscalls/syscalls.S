#include <asm/asm.h>
#include <asm/regdef.h>
#include <asm/unistd.h>

	.set noreorder

LEAF(__syscall_common)
	syscall
        beqz    a3, 1f
	# sw is actually two instructions; the first one goes
	# in the branch delay slot
	# XXX: Break this up manually; as it is now it generates warnings.
        sw      v0, errno
        li      v0, -1
1:      jr      ra
END(__syscall_common)

LEAF(access)
	j	__syscall_common
	li	v0, __NR_access
END(access)

LEAF(close)
	j	__syscall_common
	li	v0, __NR_close
END(close)

LEAF(chdir)
	j	__syscall_common
	li	v0, __NR_chdir
END(chdir)

LEAF(chroot)
	j	__syscall_common
	li	v0, __NR_chroot
END(chroot)

LEAF(dup2)
	j	__syscall_common
	li	v0, __NR_dup2
END(dup2)

LEAF(execve)
	j	__syscall_common
	li	v0, __NR_execve
END(execve)

LEAF(_exit)
	j	__syscall_common
	li	v0, __NR_exit
END(_exit)

LEAF(getdents)
	j	__syscall_common
	li	v0, __NR_getdents64
END(getdents)

LEAF(getpid)
	j	__syscall_common
	li	v0, __NR_getpid
END(getpid)

LEAF(kill)
	j	__syscall_common
	li	v0, __NR_kill
END(kill)

LEAF(lstat)
	j	__syscall_common
	li	v0, __NR_lstat64
END(lstat)

LEAF(mkdir)
	j	__syscall_common
	li	v0, __NR_mkdir
END(mkdir)

LEAF(mknod)
	j	__syscall_common
	li	v0, __NR_mknod
END(mknod)

LEAF(__mmap2)
	j	__syscall_common
	li	v0, __NR_mmap2
END(__mmap2)

LEAF(mount)
	j	__syscall_common
	li	v0, __NR_mount
END(mount)

LEAF(munmap)
	j	__syscall_common
	li	v0, __NR_munmap
END(munmap)

LEAF(__open)
	j	__syscall_common
	li	v0, __NR_open
END(__open)

LEAF(read)
	j	__syscall_common
	li	v0, __NR_read
END(read)

LEAF(rmdir)
	j	__syscall_common
	li	v0, __NR_rmdir
END(rmdir)

LEAF(__statfs64)
	j	__syscall_common
	li	v0, __NR_statfs64
END(__statfs64)

LEAF(stat)
	j	__syscall_common
	li	v0, __NR_stat64
END(stat)

LEAF(umount2)
	j	__syscall_common
	li	v0, __NR_umount2
END(umount2)

LEAF(unlink)
	j	__syscall_common
	li	v0, __NR_unlink
END(unlink)

LEAF(write)
	j	__syscall_common
	li	v0, __NR_write
END(write)
