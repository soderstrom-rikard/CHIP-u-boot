/*
 * Copyright 2014 Broadcom Corporation.
 * Copyright 2015 Free Electrons.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>
#include <common.h>

#include <aboot.h>
#include <sparse_format.h>

#include <linux/mtd/mtd.h>
#include <jffs2/jffs2.h>
#include <nand.h>

/* TODO: share that fastboot_fail / fastboot_okay code */

/* The 64 defined bytes plus the '\0' */
#define RESPONSE_LEN	(64 + 1)

static char *response_str;

void fastboot_fail(const char *s)
{
	strncpy(response_str, "FAIL\0", 5);
	strncat(response_str, s, RESPONSE_LEN - 4 - 1);
}

void fastboot_okay(const char *s)
{
	strncpy(response_str, "OKAY\0", 5);
	strncat(response_str, s, RESPONSE_LEN - 4 - 1);
}

struct fb_nand_sparse {
	nand_info_t		*nand;
	struct part_info	*part;
};

__weak int board_fastboot_erase_partition_setup(char *name)
{
	return 0;
}

__weak int board_fastboot_write_partition_setup(char *name)
{
	return 0;
}

static int fb_nand_lookup(const char *partname, char *response,
			  nand_info_t **nand,
			  struct part_info **part)
{
	struct mtd_device *dev;
	int ret;
	u8 pnum;

	ret = mtdparts_init();
	if (ret) {
		error("Cannot initialize MTD partitions\n");
		fastboot_fail("cannot init mtdparts");
		return ret;
	}

	ret = find_dev_and_part(partname, &dev, &pnum, part);
	if (ret) {
		error("cannot find partition: '%s'", partname);
		fastboot_fail("cannot find partition");
		return ret;
	}

	if (dev->id->type != MTD_DEV_TYPE_NAND) {
		error("partition '%s' is not stored on a NAND device",
		      partname);
		fastboot_fail("not a NAND device");
		return -EINVAL;
	}

	*nand = &nand_info[dev->id->num];

	return 0;
}

static int _fb_nand_erase(nand_info_t *nand, struct part_info *part)
{
	nand_erase_options_t opts;
	int ret;

	memset(&opts, 0, sizeof(opts));
	opts.offset = part->offset;
	opts.length = part->size;
	opts.quiet = 1;

	printf("Erasing blocks 0x%llx to 0x%llx\n",
	       part->offset, part->offset + part->size);

	ret = nand_erase_opts(nand, &opts);
	if (ret)
		return ret;

	printf("........ erased 0x%llx bytes from '%s'\n",
	       part->size, part->name);

	return 0;
}

static int _fb_nand_write(nand_info_t *nand, struct part_info *part,
			  void *buffer, unsigned int offset,
			  unsigned int length)
{
	int flags = WITH_WR_VERIFY;
	int ret;

#ifdef CONFIG_FASTBOOT_FLASH_NAND_TRIMFFS
	flags |= WITH_DROP_FFS;
#endif

	ret = nand_write_skip_bad(nand, offset, &length, NULL,
				  part->size - (offset - part->offset),
				  buffer, flags);
	if (ret)
		return ret;

	return ret;
}

static unsigned int fb_nand_sparse_write(struct sparse_storage *storage,
					 void *priv,
					 unsigned int offset,
					 unsigned int size,
					 char *data)
{
	struct fb_nand_sparse *sparse = priv;

	if (sparse->nand->slc_mode) {
		unsigned int erase, page;

		erase = (offset * storage->block_sz) / sparse->nand->erasesize;
		erase = erase * 2;
		page = (offset * storage->block_sz) % sparse->nand->erasesize;
		offset = erase * sparse->nand->erasesize + page;
	} else {
		offset = offset * storage->block_sz;
	}

	_fb_nand_write(sparse->nand, sparse->part, data, offset,
		       size * storage->block_sz);

	return size;
}

void fb_nand_flash_write(const char *partname, void *download_buffer,
			 unsigned int download_bytes, char *response)
{
	struct part_info *part;
	nand_info_t *nand = NULL;
	int ret;

	/* initialize the response buffer */
	response_str = response;

	ret = fb_nand_lookup(partname, response, &nand, &part);
	if (ret) {
		error("invalid NAND device");
		fastboot_fail("invalid NAND device");
		return;
	}

	ret = board_fastboot_erase_partition_setup(part->name);
	if (ret)
		return;

	ret = _fb_nand_erase(nand, part);
	if (ret) {
		error("failed erasing from device %s", nand->name);
		fastboot_fail("failed erasing from device");
		return;
	}

	ret = board_fastboot_write_partition_setup(part->name);
	if (ret)
		return;

	if (is_sparse_image(download_buffer)) {
		struct fb_nand_sparse sparse_priv;
		sparse_storage_t sparse;

		sparse_priv.nand = nand;
		sparse_priv.part = part;

		sparse.block_sz = nand->writesize;
		sparse.start = part->offset / sparse.block_sz;
		sparse.size = part->size  / sparse.block_sz;
		sparse.name = part->name;
		sparse.write = fb_nand_sparse_write;

		printf("Flashing sparse image at offset 0x%llx\n",
		       part->offset);

		store_sparse_image(&sparse, &sparse_priv, download_buffer);
	} else {
		printf("Flashing raw image at offset 0x%llx\n",
		       part->offset);

		_fb_nand_write(nand, part, download_buffer, part->offset,
			       download_bytes);

		printf("........ wrote %u bytes to '%s'\n",
		       download_bytes, part->name);
	}

	fastboot_okay("");
}

void fb_nand_erase(const char *partname, char *response)
{
	struct part_info *part;
	nand_info_t *nand = NULL;
	int ret;

	/* initialize the response buffer */
	response_str = response;

	ret = fb_nand_lookup(partname, response, &nand, &part);
	if (ret) {
		error("invalid NAND device");
		fastboot_fail("invalid NAND device");
		return;
	}

	ret = board_fastboot_erase_partition_setup(part->name);
	if (ret)
		return;

	ret = _fb_nand_erase(nand, part);
	if (ret) {
		error("failed erasing from device %s", nand->name);
		fastboot_fail("failed erasing from device");
		return;
	}

	fastboot_okay("");
}
