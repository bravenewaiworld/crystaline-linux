/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Port notes: newer kernel interfaces used by this driver, written in terms
 * of what Linux 4.9 provides, and the upstream fixes applied.
 *
 * The driver is fs/exfat from Linux 6.18.57. Each entry here is a one-to-one
 * rename or a small helper, so the rest of the driver stays as upstream wrote
 * it and upstream fixes apply cleanly; the parts where 4.9 works differently
 * (mount options, page cache and mmap hooks, attribute calls) are changed in
 * place.
 *
 * Upstream fixes from after 6.18 applied on top (mainline commits), each with
 * "commit <id> upstream." and how it was adapted in its message:
 *   866cba367541 exfat: validate the cluster bitmap bits of directory
 *   d1d75eaf01ab exfat: fix error handling for FAT table operations
 *   73f01258002f exfat: fix passing zero to ERR_PTR() in exfat_mkdir()
 *   4129a3a2751c exfat: fix s_maxbytes
 *   896fcc93574f exfat: write moved entry before removing source
 *   ce12a39bed48 exfat: free new directory cluster if zeroing fails
 *   0901a71e8cff exfat: clean up new entry on add entry failure
 *   0eb24ad05a6a exfat: fix overflow in cluster-to-dentry conversion
 * Not listed because 6.18.57 already has it: ff37797badd8 (directory
 * checksum after a rename to a shorter name).
 *
 * 64-bit only: on 64-bit, 4.9 defines struct timespec64 as struct timespec
 * (the type of inode timestamps), so the timestamp helpers below copy them as
 * they are.
 */

#ifndef _EXFAT_COMPAT_H
#define _EXFAT_COMPAT_H

#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/mm.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

/* include/uapi/linux/exfat.h in newer kernels. */
#define EXFAT_IOC_SHUTDOWN		_IOR('X', 125, __u32)
#define EXFAT_GOING_DOWN_DEFAULT	0x0	/* default with full sync */
#define EXFAT_GOING_DOWN_FULLSYNC	0x1	/* going down with full sync */
#define EXFAT_GOING_DOWN_NOSYNC		0x2	/* going down */

/* include/uapi/linux/fs.h in newer kernels. */
#define FSLABEL_MAX		256
#define FS_IOC_GETFSLABEL	_IOR(0x94, 49, char[FSLABEL_MAX])
#define FS_IOC_SETFSLABEL	_IOW(0x94, 50, char[FSLABEL_MAX])

/* include/uapi/linux/magic.h and include/linux/blkdev.h in newer kernels. */
#define EXFAT_SUPER_MAGIC	0x2011BAB0UL
#define SECTOR_SIZE		512

#define SB_RDONLY		MS_RDONLY
#define SB_SYNCHRONOUS		MS_SYNCHRONOUS
#define SB_NODIRATIME		MS_NODIRATIME

static inline bool sb_rdonly(const struct super_block *sb)
{
	return sb->s_flags & MS_RDONLY;
}

#define alloc_inode_sb(sb, cache, gfp)	kmem_cache_alloc(cache, gfp)
#define set_default_d_op(sb, ops)	((sb)->s_d_op = (ops))
#define get_random_u32			get_random_int
#define filemap_splice_read		generic_file_splice_read

static inline __kernel_fsid_t u64_to_fsid(u64 v)
{
	return (__kernel_fsid_t){ .val = { (u32)v, (u32)(v >> 32) } };
}

/* Inode timestamps and change counter */

static inline struct timespec64 inode_get_atime(const struct inode *inode)
{
	return inode->i_atime;
}

static inline struct timespec64 inode_get_mtime(const struct inode *inode)
{
	return inode->i_mtime;
}

static inline struct timespec64 inode_set_atime_to_ts(struct inode *inode,
		struct timespec64 ts)
{
	inode->i_atime = ts;
	return ts;
}

static inline struct timespec64 inode_set_mtime_to_ts(struct inode *inode,
		struct timespec64 ts)
{
	inode->i_mtime = ts;
	return ts;
}

static inline struct timespec64 inode_set_ctime_to_ts(struct inode *inode,
		struct timespec64 ts)
{
	inode->i_ctime = ts;
	return ts;
}

static inline struct timespec64 inode_set_ctime_current(struct inode *inode)
{
	return inode_set_ctime_to_ts(inode, current_time(inode));
}

/* Sets all three timestamps to now and returns it. */
static inline struct timespec64 simple_inode_init_ts(struct inode *inode)
{
	struct timespec64 ts = inode_set_ctime_current(inode);

	inode_set_atime_to_ts(inode, ts);
	inode_set_mtime_to_ts(inode, ts);
	return ts;
}

static inline void simple_rename_timestamp(struct inode *old_dir,
		struct dentry *old_dentry, struct inode *new_dir,
		struct dentry *new_dentry)
{
	struct inode *newino = d_inode(new_dentry);

	inode_set_mtime_to_ts(old_dir, inode_set_ctime_current(old_dir));
	if (new_dir != old_dir)
		inode_set_mtime_to_ts(new_dir,
				      inode_set_ctime_current(new_dir));
	inode_set_ctime_current(d_inode(old_dentry));
	if (newino)
		inode_set_ctime_current(newino);
}

static inline void inode_set_iversion(struct inode *inode, u64 val)
{
	inode->i_version = val;
}

static inline u64 inode_query_iversion(struct inode *inode)
{
	return inode->i_version;
}

static inline u64 inode_peek_iversion_raw(const struct inode *inode)
{
	return inode->i_version;
}

static inline bool inode_eq_iversion(const struct inode *inode, u64 old)
{
	return inode->i_version == old;
}

/* Memory */

/* kmalloc, falling back to vmalloc for large sizes; free with kvfree. */
static inline void *kvmalloc_array(size_t n, size_t size, gfp_t flags)
{
	void *p;

	if (size && n > SIZE_MAX / size)
		return NULL;
	p = kmalloc(n * size, flags | __GFP_NOWARN | __GFP_NORETRY);
	if (!p)
		p = __vmalloc(n * size, flags | __GFP_HIGHMEM, PAGE_KERNEL);
	return p;
}

static inline void *kvcalloc(size_t n, size_t size, gfp_t flags)
{
	return kvmalloc_array(n, size, flags | __GFP_ZERO);
}

/* Block devices and buffers */

static inline unsigned int bdev_max_discard_sectors(struct block_device *bdev)
{
	return bdev_get_queue(bdev)->limits.max_discard_sectors;
}

static inline unsigned int bdev_discard_granularity(struct block_device *bdev)
{
	return bdev_get_queue(bdev)->limits.discard_granularity;
}

/* Freezes the filesystem mounted on bdev, if any, after syncing it. */
static inline int bdev_freeze(struct block_device *bdev)
{
	return PTR_ERR_OR_ZERO(freeze_bdev(bdev));
}

static inline int bdev_thaw(struct block_device *bdev)
{
	return thaw_bdev(bdev, bdev->bd_super);
}

static inline int sync_blockdev_range(struct block_device *bdev,
		loff_t lstart, loff_t lend)
{
	return filemap_write_and_wait_range(bdev->bd_inode->i_mapping,
					    lstart, lend);
}

/* 4.9 also takes a GFP mask and a pointer for the failing sector. */
#define blkdev_issue_flush(bdev)	blkdev_issue_flush(bdev, GFP_KERNEL, NULL)

/*
 * Reads bh if it isn't up to date: 1 if it already was, 0 once read, -EIO on
 * a read error. op_flags is ignored (the driver passes none).
 */
static inline int bh_read(struct buffer_head *bh, int op_flags)
{
	if (bh_uptodate_or_lock(bh))
		return 1;
	return bh_submit_read(bh);
}

/* Page faults */

typedef int vm_fault_t;
#define vmf_fs_error		block_page_mkwrite_return

#endif /* _EXFAT_COMPAT_H */
