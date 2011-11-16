/*
 * Mini-block driver.
 *
 * This code is based on the example of LWN.NET: http://lwn.net/Articles/31513/
 *
 * Copyright 2003 Eklektix, Inc.  Redistributable under the terms
 * of the GNU GPL.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h> 
#include <linux/fs.h>     
#include <linux/errno.h>  
#include <linux/types.h>   
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/buffer_head.h> /* invalidate_bdev */
#include <linux/bio.h>

MODULE_LICENSE("Dual BSD/GPL");

#define KERNEL_SECTOR_SIZE 512 
#define MAJOR_NUM 240 /* free mayor number, see devices.txt */
#define MINIBD_MINORS 16 /* maximum partitions availables */
#define N_SECTORS 1024 /* number of sectors of our block device */
#define MINIBLOCK_SECTOR_SIZE 512 /* our sector size */

/*
 * miniblock request queue.
 */
static struct request_queue *miniblock_queue;

/*
 * Device private structure. The internal 
 * representation of our device.
 */
static struct miniblock_device {
	unsigned long size;
	spinlock_t lock;
	u8 *data;
	struct gendisk *gd;
};

static struct miniblock_device *Miniblock = NULL;

/*
 * Real handler of I/O requests.
 */
static void miniblock_transfer(struct miniblock_device *dev, unsigned long sector,
		unsigned long nsect, char *buffer, int write)
{
    unsigned long offset = sector*MINIBLOCK_SECTOR_SIZE;
    unsigned long nbytes = nsect*MINIBLOCK_SECTOR_SIZE;
    
    if ((offset + nbytes) > dev->size) {
	printk (KERN_NOTICE "minibd: Beyond-end write (%ld %ld)\n", offset, nbytes);
	return;
    }
    if (write)
	memcpy(dev->data + offset, buffer, nbytes);
    else
	memcpy(buffer, dev->data + offset, nbytes);
}

/*
 * I/O request handler.
 */
static void miniblock_request( struct request_queue *q)
{
#if 0
	struct request *rq = blk_fetch_request( q );
	printk( KERN_DEBUG " %s:%d\n", __FILE__, __LINE__);

	for( ; rq ; )
	{
		if( rq->cmd_type != REQ_TYPE_FS )
		{
			printk( KERN_DEBUG "minibd: cmd type = %d\n",rq->cmd_type);
			__blk_end_request_all( rq, 0 );
			break;
		//	continue;
		}	

		bool	do_write = (rq_data_dir(rq) == WRITE );
		int 	size	= blk_rq_bytes( rq );

		printk( KERN_DEBUG " minibf: w/r=%s 0x%x at 0x%llx\n", (do_write ? "write" : "read"), size, blk_rq_pos(rq)*512ULL );

		__blk_end_request_all( rq, 1 );
		break;

		rq = blk_fetch_request(q) ;
	}
//	__blk_end_request_all( rq, 0 );
#endif


        struct request *req = blk_fetch_request(q);
        while (req) {
                unsigned block = blk_rq_pos(req);
                unsigned count = blk_rq_cur_sectors(req);
//                XD_INFO *disk = req->rq_disk->private_data;
                int res = -EIO;
                int retry;

                if (req->cmd_type != REQ_TYPE_FS)
                        goto done;
                if (block + count > get_capacity(req->rq_disk))
                        goto done;


		bool	do_write = (rq_data_dir(req) == WRITE );
		int 	size	= blk_rq_bytes( req );

/*
                for (retry = 0; (retry < XD_RETRIES) && !res; retry++)
                        res = xd_readwrite(rq_data_dir(req), disk, req->buffer,
                                           block, count);
*/
		res = 0;
        done:  
                /* wrap up, 0 = success, -errno = fail */
                if (!__blk_end_request_cur(req, res))
                        req = blk_fetch_request(q);
        }

}


int miniblock_ioctl (struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{
	long size;
	struct hd_geometry geo;

	switch(cmd) {
	/*
	 * The only command we need to interpret is HDIO_GETGEO, since
	 * we can't partition the drive otherwise.  We have no real
	 * geometry, of course, so make something up.
	 */
	    case HDIO_GETGEO:
		size = Miniblock->size*(MINIBLOCK_SECTOR_SIZE/KERNEL_SECTOR_SIZE);
		geo.cylinders = (size & ~0x3f) >> 6;
		geo.heads = 4;
		geo.sectors = 16;
		geo.start = 4;
		if (copy_to_user((void *) arg, &geo, sizeof(geo)))
			return -EFAULT;
		return 0;
	}

    return -ENOTTY; 
}

/*
 * Device operations.
 */
static struct block_device_operations miniblock_ops = {
    .owner           = THIS_MODULE,
    .ioctl	     = miniblock_ioctl
};

/*
 * LKM stuff.
 */
static int __init miniblock_init(void)
{
	static int ret;

	ret = register_blkdev(MAJOR_NUM, "minibd");
	if (ret < 0) {
		printk(KERN_WARNING "minibd: unable to get major number\n");
		return -EBUSY;
	}

	printk(KERN_DEBUG "minibd: register success\n");

	Miniblock = kmalloc(sizeof(struct miniblock_device), GFP_KERNEL);

		bool	do_write = (rq_data_dir(rq) == WRITE );
		int 	size	= blk_rq_bytes( rq );
	if (Miniblock == NULL) {
		printk(KERN_WARNING "minidb: unable to get memory with kmalloc\n");
		goto out_unregister;
	}

	/*
	 * Device initialization
	 */
	memset(Miniblock, 0, sizeof(struct miniblock_device));
	Miniblock->size = N_SECTORS*MINIBLOCK_SECTOR_SIZE;
	Miniblock->data = vmalloc(Miniblock->size);
	if (Miniblock->data == NULL) {
		printk(KERN_WARNING "minidb: unable to get memory with vmalloc\n");
		kfree(Miniblock);
		goto out_unregister;
	}
	spin_lock_init(&Miniblock->lock);

	/*
	 * request queue creation (one request per block device).
	 */
	miniblock_queue = blk_nit_queue(miniblock_request, &Miniblock->lock);
	if (miniblock_queue == NULL) {
		printk(KERN_WARNING "minibd: error in blk_init_queue\n");
		goto out_free;
	}

	//blk_queue_hardsect_size(miniblock_queue, MINIBLOCK_SECTOR_SIZE);
	

	/*
	 * fill gendisk structure.
	 */
	Miniblock->gd = alloc_disk(MINIBD_MINORS);
	if (!Miniblock->gd) {
		printk(KERN_WARNING "minibd: error in alloc_disk\n");
		goto out_free;
	}

	Miniblock->gd->major = MAJOR_NUM;
	Miniblock->gd->first_minor = 0;
	Miniblock->gd->fops = &miniblock_ops;
	Miniblock->gd->private_data = Miniblock;
	snprintf(Miniblock->gd->disk_name, 10, "%s", "minibd0");
	set_capacity(Miniblock->gd, N_SECTORS*(MINIBLOCK_SECTOR_SIZE/KERNEL_SECTOR_SIZE));
	Miniblock->gd->queue = miniblock_queue;

	add_disk(Miniblock->gd);

	return 0;

out_free:
	kfree(Miniblock);
	vfree(Miniblock->data);
out_unregister:
	unregister_blkdev(MAJOR_NUM, "minibd");

	return -ENOMEM;
}

static void __exit miniblock_exit(void)
{
	del_gendisk(Miniblock->gd);
	put_disk(Miniblock->gd);

	unregister_blkdev(MAJOR_NUM, "minibd");
	blk_cleanup_queue(miniblock_queue);

	vfree(Miniblock->data);
	kfree(Miniblock);

	printk(KERN_DEBUG "minibd: download with succes!\n");
}
	
module_init(miniblock_init);
module_exit(miniblock_exit);
