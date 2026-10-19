/*
 * Built-in device-tree edits: a table of fdtput-like steps applied to the
 * bootloader's flat tree before it is unflattened (CONFIG_OF_EDITS).
 *
 * The table is generated at build time by recording the fdtput/fdtget calls
 * of a script run against "the vendor's tree", the tree the bootloader is
 * shipped with (Crystaline OS: tools/system/dt-edits.sh runs dt-tweaks.sh on
 * the DTB in TrimUI's boot package). This header and fdt_edits.c also build on
 * a PC against dtc's libfdt, so the same code can be tested there.
 */
#ifndef _FDT_EDITS_H
#define _FDT_EDITS_H

#ifdef __KERNEL__
#include <linux/libfdt.h>
#include <linux/types.h>
#else
#include <libfdt.h>
#include <stdbool.h>
#endif

/*
 * Free space added to the bootloader's tree for the edits to grow into: far
 * more than they need. Running out fails with FDT_ERR_NOSPACE, and then
 * nothing is applied.
 */
#define FDT_EDITS_ROOM (64 * 1024)

/* Where the outcome is recorded (struct fdt_edits_result's status). */
#define FDT_EDITS_STATUS_PATH "/chosen"
#define FDT_EDITS_STATUS_PROP "crystaline,dt-edits"

enum fdt_edit_kind {
	/*
	 * The property must hold value: the script read it (e.g. a node's
	 * phandle, which it then wrote into another property), so the table is
	 * valid only while it is unchanged.
	 */
	FDT_EDIT_EXPECT,
	FDT_EDIT_SET,		/* set the property to value; see was and keep_bootloader_value */
	FDT_EDIT_DELETE,	/* delete the property; absent is fine */
	FDT_EDIT_NODE,		/* create the node and any missing parents; existing is fine */
};

/*
 * One step of the edits. Paths are absolute. For every kind but FDT_EDIT_NODE,
 * the node must already exist (or be made by an earlier NODE step).
 */
struct fdt_edit {
	enum fdt_edit_kind kind;
	const char *path;
	const char *name;	/* property; NULL for FDT_EDIT_NODE */
	const void *value;	/* EXPECT, SET */
	int len;		/* bytes in value */
	/*
	 * SET: the vendor's value (was_len < 0: the vendor's tree has no such
	 * property). A different current value means something before the kernel
	 * (the bootloader, or an older edited tree) changed it.
	 */
	const void *was;
	int was_len;
	/* SET: leave such a changed value as it is (dt-tweaks.sh: KEEP_UBOOT_VALUE=1). */
	bool keep_bootloader_value;
};

/*
 * Only SET steps are counted, each under exactly one count; after a failure
 * the counts are those up to it.
 */
struct fdt_edits_result {
	int applied;	/* held the vendor's value (or was absent, as there) and was set */
	int in_place;	/* already our value */
	int kept;	/* set before the kernel started and kept (keep_bootloader_value) */
	int replaced;	/* set before the kernel started and set to ours */
	/* libfdt error that stopped the edits (-FDT_ERR_BADVALUE: an EXPECT mismatch), or 0 */
	int error;
	/*
	 * The outcome as recorded in FDT_EDITS_STATUS_PROP: "ok <table id>: <counts>"
	 * or "failed: edit <n> (<kind> <path> <property>): <error>; table <id>", plus
	 * a note when the bootloader's tree already has a node the edits add (it
	 * came edited, e.g. from an older patched boot package).
	 */
	char status[384];
};

/* Called for each kept or replaced value, with what = "kept" or "replaced". */
typedef void (*fdt_edits_note_fn)(const struct fdt_edit *e, const char *what);

/* The table, generated at build time. */
extern const struct fdt_edit fdt_edits[];
extern const int fdt_edits_count;
extern const char fdt_edits_id[];	/* names the script and vendor tree it came from */

/*
 * Copies the bootloader's tree src into buf (fdt_totalsize(src) +
 * FDT_EDITS_ROOM bytes) and applies the built-in table to the copy, all or
 * nothing: if an edit fails (res->error != 0), buf is src unedited instead.
 * Either way res->status is written to FDT_EDITS_STATUS_PATH's
 * FDT_EDITS_STATUS_PROP. Returns 0 when buf holds a tree to boot, or a libfdt
 * error when it does not.
 */
int fdt_edits_copy_and_apply(const void *src, void *buf,
			     struct fdt_edits_result *res,
			     fdt_edits_note_fn note);

#endif
