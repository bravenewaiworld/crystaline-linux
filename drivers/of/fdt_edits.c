/*
 * Applies the built-in device-tree edits (see fdt_edits.h).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/string.h>
#else
#include <stdio.h>
#include <string.h>
#endif
#include "fdt_edits.h"

/* Offset of the node at path; with create, missing nodes on the way are added. */
static int node_at(void *fdt, const char *path, bool create)
{
	int off = 0;

	while (*path) {
		int len, next;

		if (*path == '/') {
			path++;
			continue;
		}
		len = strcspn(path, "/");
		next = fdt_subnode_offset_namelen(fdt, off, path, len);
		if (next == -FDT_ERR_NOTFOUND && create)
			next = fdt_add_subnode_namelen(fdt, off, path, len);
		if (next < 0)
			return next;
		off = next;
		path += len;
	}
	return off;
}

/* Whether a property's value (NULL: absent) equals want (want_len < 0: absent). */
static bool equal(const void *val, int len, const void *want, int want_len)
{
	if (!val)
		return want_len < 0;
	return len == want_len && !memcmp(val, want, len);
}

static const char *kind_name(enum fdt_edit_kind kind)
{
	static const char *const names[] = {
		[FDT_EDIT_EXPECT] = "expect", [FDT_EDIT_SET] = "set",
		[FDT_EDIT_DELETE] = "delete", [FDT_EDIT_NODE] = "node",
	};

	return names[kind];
}

static int set(void *fdt, int off, const struct fdt_edit *e, const void *cur,
	       int len, struct fdt_edits_result *res, fdt_edits_note_fn note)
{
	if (equal(cur, len, e->value, e->len)) {
		res->in_place++;
		return 0;
	}
	if (equal(cur, len, e->was, e->was_len)) {
		res->applied++;
	} else if (e->keep_bootloader_value) {
		/* Neither ours nor the vendor's: the bootloader, or an older edited tree, set it. */
		res->kept++;
		note(e, "kept");
		return 0;
	} else {
		res->replaced++;
		note(e, "replaced");
	}
	return fdt_setprop(fdt, off, e->name, e->value, e->len);
}

static int apply_one(void *fdt, const struct fdt_edit *e,
		     struct fdt_edits_result *res, fdt_edits_note_fn note)
{
	int off = node_at(fdt, e->path, e->kind == FDT_EDIT_NODE);
	int len;
	const void *cur;

	if (off < 0)
		return off;
	if (e->kind == FDT_EDIT_NODE)
		return 0;
	cur = fdt_getprop(fdt, off, e->name, &len);
	switch (e->kind) {
	case FDT_EDIT_SET:
		return set(fdt, off, e, cur, len, res, note);
	case FDT_EDIT_DELETE:
		return cur ? fdt_delprop(fdt, off, e->name) : 0;
	case FDT_EDIT_EXPECT:
		return equal(cur, len, e->value, e->len) ? 0 : -FDT_ERR_BADVALUE;
	default:
		return -FDT_ERR_INTERNAL;
	}
}

/*
 * Applies the table in order to fdt, which must have room to grow. Stops at the
 * first EXPECT mismatch or libfdt error, leaving fdt partly edited, and returns
 * the index of that edit (-1: none).
 */
static int apply_all(void *fdt, struct fdt_edits_result *res, fdt_edits_note_fn note)
{
	int i;

	for (i = 0; i < fdt_edits_count; i++) {
		res->error = apply_one(fdt, &fdt_edits[i], res, note);
		if (res->error < 0)
			return i;
	}
	res->error = 0;
	return -1;
}

/*
 * Whether src already has a node that the edits add. The vendor's tree has
 * none of them (the script's fdtput -c would have failed on it), so such a
 * src was edited before it reached the kernel.
 */
static bool came_edited(const void *src)
{
	int i;

	for (i = 0; i < fdt_edits_count; i++)
		if (fdt_edits[i].kind == FDT_EDIT_NODE &&
		    fdt_path_offset(src, fdt_edits[i].path) >= 0)
			return true;
	return false;
}

int fdt_edits_copy_and_apply(const void *src, void *buf,
			     struct fdt_edits_result *res,
			     fdt_edits_note_fn note)
{
	int size = fdt_totalsize(src) + FDT_EDITS_ROOM;
	const char *extra = came_edited(src) ?
		"; the bootloader's tree already had these edits (a boot package patched by an older build)" : "";
	const struct fdt_edit *e;
	int failed, err, off;

	*res = (struct fdt_edits_result){ 0 };
	err = fdt_open_into(src, buf, size);
	if (err)
		return err;
	failed = apply_all(buf, res, note);
	if (failed < 0) {
		snprintf(res->status, sizeof(res->status),
			 "ok %s: %d applied, %d in place, %d kept, %d replaced%s",
			 fdt_edits_id, res->applied, res->in_place, res->kept,
			 res->replaced, extra);
	} else {
		e = &fdt_edits[failed];
		snprintf(res->status, sizeof(res->status),
			 "failed: edit %d (%s %s%s%s): %s; table %s%s", failed,
			 kind_name(e->kind), e->path, e->name ? " " : "",
			 e->name ? e->name : "", fdt_strerror(res->error),
			 fdt_edits_id, extra);
		err = fdt_open_into(src, buf, size);
		if (err)
			return err;
	}
	off = node_at(buf, FDT_EDITS_STATUS_PATH, true);
	return off < 0 ? off : fdt_setprop_string(buf, off, FDT_EDITS_STATUS_PROP, res->status);
}
