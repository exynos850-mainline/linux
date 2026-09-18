/******************************************************************************
 *
 * Copyright (c) 2012 - 2016 Samsung Electronics Co., Ltd. All rights reserved
 *
 *****************************************************************************/

#ifndef __SLSI_PROCFS_H__
#define __SLSI_PROCFS_H__

#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/errno.h>

struct slsi_dev;
struct slsi_vif;

/* proc entry ownership is root on mainline; Android AID tables are not used */
#define SLSI_PROCFS_SET_UID_GID(entry)

#define SLSI_pde_data(inode) pde_data(inode)

/* procfs operations */
int slsi_create_proc_dir(struct slsi_dev *sdev);
void slsi_remove_proc_dir(struct slsi_dev *sdev);

int slsi_procfs_open_file_generic(struct inode *inode, struct file *file);

#define SLSI_PROCFS_SEQ_FILE_OPS(name)                                                      \
	static int slsi_procfs_ ## name ## _show(struct seq_file *m, void *v);              \
	static int slsi_procfs_ ## name ## _open(struct inode *inode, struct file *file)    \
	{                                                                                   \
		return single_open(file, slsi_procfs_  ## name ## _show, SLSI_pde_data(inode)); \
	}                                                                                   \
	static const struct proc_ops slsi_procfs_ ## name ## _fops = {               \
		.proc_open = slsi_procfs_ ## name ## _open,                                      \
		.proc_read = seq_read,                                                           \
		.proc_llseek = seq_lseek,                                                        \
		.proc_release = single_release,                                                  \
	}

#define SLSI_PROCFS_SEQ_ADD_FILE(_sdev, name, parent, mode) \
	do {                                                \
		struct proc_dir_entry *entry;               \
		entry = proc_create_data(# name, mode, parent, &slsi_procfs_ ## name ## _fops, _sdev); \
		if (!entry) {                               \
			goto err;                           \
		}                                           \
		SLSI_PROCFS_SET_UID_GID(entry);                            \
	} while (0)

#define SLSI_PROCFS_READ_FILE_OPS(name)                                       \
	static ssize_t slsi_procfs_ ## name ## _read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos); \
	static const struct proc_ops slsi_procfs_ ## name ## _fops = { \
		.proc_read = slsi_procfs_ ## name ## _read,                        \
		.proc_open = slsi_procfs_open_file_generic,                        \
		.proc_llseek = generic_file_llseek                                 \
	}

#define SLSI_PROCFS_WRITE_FILE_OPS(name)                                       \
	static ssize_t slsi_procfs_ ## name ## _write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos); \
	static const struct proc_ops slsi_procfs_ ## name ## _fops = { \
		.proc_write = slsi_procfs_ ## name ## _write,                        \
		.proc_open = slsi_procfs_open_file_generic,                        \
		.proc_llseek = generic_file_llseek                                 \
	}

#define SLSI_PROCFS_RW_FILE_OPS(name)                                               \
	static ssize_t slsi_procfs_ ## name ## _write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos); \
	static ssize_t                      slsi_procfs_ ## name ## _read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos); \
	static const struct proc_ops slsi_procfs_ ## name ## _fops = { \
		.proc_read = slsi_procfs_ ## name ## _read,                        \
		.proc_write = slsi_procfs_ ## name ## _write,                      \
		.proc_open = slsi_procfs_open_file_generic,                        \
		.proc_llseek = generic_file_llseek                                 \
	}

#define SLSI_PROCFS_ADD_FILE(_sdev, name, parent, mode)                    \
	do {                                                               \
		struct proc_dir_entry *entry = proc_create_data(# name, mode, parent, &slsi_procfs_ ## name ## _fops, _sdev); \
		SLSI_PROCFS_SET_UID_GID(entry);                            \
	} while (0)
#define SLSI_PROCFS_REMOVE_FILE(name, parent) remove_proc_entry(# name, parent)

void slsi_procfs_inc_node(void);
void slsi_procfs_dec_node(void);

#endif
