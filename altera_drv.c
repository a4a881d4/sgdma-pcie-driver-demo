#include <linux/fs.h>
#include <linux/module.h>                         // MOD_DEVICE_TABLE,
#include <linux/init.h>
#include <linux/pci.h>                            // pci_device_id,
#include <linux/interrupt.h>
#include <asm/uaccess.h>                          // copy_to_user,
#include <linux/version.h>                        // KERNEL_VERSION,
#include <iso646.h>
#include <linux/platform_device.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/vmalloc.h>


/* **************************************************************
 * Bar 0_1 - on-chip mem (r/w), data width 64, size 262144 bytes
 *           ddr2 sdram controller, data width 64
 *           so, we can address these mem from 0x0 ~ 0x1ffffff (128mb for 64 bit address)
 *
 * Bar 2 - Modular SGDMA dispatcher, mode: mem-mapped 2 mem-mapped
 *
 * **************************************************************/


//#include <linux/config.h>
//#include <linux/init.h>

// simple version number
#define _ALTERATW_SOFTWARE_VERSION_NUMBER 1.0

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)    /* not < 2.5 */
#  error "This kernel is too old: not supported by this file"
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("kfchou at altera.com");
MODULE_DESCRIPTION("PCI +rom/ram device driver example");

// vendor and device id of the PCI device
#define VENDOR_ID 0x1172
#define DEVICE_ID 0xe001

static int DEVICE_MAJOR = 231;
#define DEVICE_NAME "altera_test"
#define MAJOR_NUM 240 /* free mayor number, see devices.txt */
#define MAX_PARTITION_NR 16

// not really necessary; for future use
//MODULE_DEVICE_TABLE(pci, altera_pci_drv);

static struct
pci_device_id altera_pci_ids[] __devinitdata =
{
  // { VENDOR_ID, DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
  { PCI_DEVICE(VENDOR_ID, DEVICE_ID), },
  { 0, }
};

/**************************Function Declarations*******************************************/
// declarations for fops, pci_driver
static int altera_device_probe(struct pci_dev *, const struct pci_device_id *);
static void handler_altera_device_deinit( struct pci_dev *);

// block driver
static int block_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
/**************************Function Declarations (end)*************************************/

/**************************Structure Define*******************************************/
static struct pci_driver altera_pci_drv =
{
  .name= "altera",
  .id_table= altera_pci_ids,
  .probe= altera_device_probe,
  .remove= handler_altera_device_deinit,
};
static struct gendisk *altera_gendisk = NULL;

static struct {
	unsigned long size;
	spinlock_t lock;
	u8 *data;
	struct gendisk *gd;
} PrivateData; 

static struct request_queue *blk_queue; 

/**************************Structure Define (end)*************************************/
// example for reading a PCI config byte
static unsigned char
get_revision(struct pci_dev *dev)
{
  u8 revision;

  pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
  return (revision);
}


static irqreturn_t pci_isr( int irq, void *dev_id, struct pt_regs *regs )
{
  // nothing todo for the irq yet
  return (IRQ_HANDLED);
}
static void blk_request( struct request_queue_t *q)
{
    struct request *req;

/*
		while ((req = elv_next_request(q)) != NULL) {
						if (! blk_fs_request(req)) {
										printk (KERN_NOTICE "Skip non-CMD request\n");
										end_request(req, 0);
										continue;
						}
		//				sbd_transfer(&Device, req->sector, req->current_nr_sectors,
		//												req->buffer, rq_data_dir(req));
		//				end_request(req, 1);
		}
*/
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
	    //case HDIO_GETGEO:
/*
		size = Miniblock->size*(MINIBLOCK_SECTOR_SIZE/KERNEL_SECTOR_SIZE);
		geo.cylinders = (size & ~0x3f) >> 6;
		geo.heads = 4;
		geo.sectors = 16;
		geo.start = 4;
		if (copy_to_user((void *) arg, &geo, sizeof(geo)))
			return -EFAULT;
		return 0;
*/
	}

    return -ENOTTY; 
}

static struct AlteraBlockDevice{
	unsigned long size;
	spinlock_t lock;
	u64 *data;
	struct gendisk *gd;
} * altera_block_device = NULL;
static struct request_queue *altera_block_queue;
static struct block_device_operations miniblock_ops = {
    .owner           = THIS_MODULE,
    .ioctl	     = miniblock_ioctl
};

int block_device_init( u64* ram_base, u32 mem_size )
{
	int ret = register_blkdev(MAJOR_NUM, DEVICE_NAME);
	if (ret < 0) 
	{
		printk(KERN_WARNING "minibd: unable to get major number\n");
		return -EBUSY;
	}

	altera_block_device = kmalloc(sizeof(struct AlteraBlockDevice), GFP_KERNEL);

	if( altera_block_device == NULL )
	{
		printk(KERN_WARNING "minidb: unable to get memory with kmalloc\n");
		goto out_unregister;
	}
	
	memset( altera_block_device, 0, sizeof( struct AlteraBlockDevice ) );
	altera_block_device -> size = mem_size;
	altera_block_device -> data = ram_base;
	spin_lock_init(&altera_block_device->lock);

	altera_block_queue = blk_init_queue(altera_block_queue, &altera_block_device->lock);

	if (altera_block_queue == NULL) {
		printk(KERN_WARNING "minibd: error in blk_init_queue\n");
		goto out_free;
	}
	altera_block_device->gd = alloc_disk( MAX_PARTITION_NR );
	if (!altera_block_device->gd) {
		printk(KERN_WARNING "minibd: error in alloc_disk\n");
		goto out_free;
	}

	altera_block_device->gd->major = MAJOR_NUM;
	altera_block_device->gd->first_minor = 0;
	altera_block_device->gd->fops = &miniblock_ops;
	altera_block_device->gd->private_data = altera_block_device;
	snprintf(altera_block_device->gd->disk_name, 13, "%s%d", DEVICE_NAME, 0);
	set_capacity(altera_block_device->gd, mem_size);
	altera_block_device->gd->queue = altera_block_queue;

	add_disk(altera_block_device->gd);
	
	return 0;

out_free:
	kfree( altera_block_device );
out_unregister:
	unregister_blkdev(MAJOR_NUM, DEVICE_NAME);

	return -ENOMEM;

}

// Initialising of the module with output about the irq, I/O region and memory region.
static
int altera_device_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
//	long iosize_0,iosize_1;
	printk(KERN_DEBUG "altera : Device 0x%x has been found at"
		" bus %d dev %d func %d\n",
		dev->device, dev->bus->number, PCI_SLOT(dev->devfn),
		PCI_FUNC(dev->devfn));

	int err = pci_enable_device (dev); 
	printk("pci_enable_device = %d\n",err);
	if( err )
		return err;
	
	

	long iosize_0 = pci_resource_len( dev, 0 );
	long iosize_1 = pci_resource_len( dev, 1 );

	printk(" bar0 %d, bar1 %d\n",iosize_0, iosize_1);

	int test_bar_nr = 0;
//	pci_request_region(dev, 0,"altera_ram");
	pci_request_region(dev, test_bar_nr,"altera_rom");

	dev_err(&dev->dev, "mem resource at PCI BAR #%d, device resource flags is 0x%llx\n",1,(pci_resource_flags(dev,1)));
	{
		u32 base = pci_resource_start(dev, test_bar_nr);
		u32 len = pci_resource_len(dev, test_bar_nr);
		u32 idx = 0;
		u64 reg;
		u64 __iomem * ram_base = ioremap_nocache(base, len);

		if( ram_base == 0 )
		{
			printk(" there's error in ioremap \n");
			return 0;
		}
		else
		{
			block_device_init( ram_base, iosize_0 );
		}
	}

	printk(" hey i'm here in probing ");
	return (0);
}


static void
handler_altera_device_deinit( struct pci_dev *pdev )
{
//  unregister_blkdev (DEVICE_MAJOR, "altera");
//  if (pdev->irq)
//    free_irq( pdev->irq, pdev );
/*
  if( iolen )
    release_region( ioport, iolen );
  if( memlen )
    release_mem_region( memstart, memlen );
*/
	if(altera_block_device )
	{
		del_gendisk(altera_block_device->gd);
		put_disk(altera_block_device->gd);

		unregister_blkdev(MAJOR_NUM, "altera_ram");
		blk_cleanup_queue( altera_block_queue );

//		vfree(Miniblock->data);
		kfree( altera_block_device );

		printk(KERN_DEBUG "minibd: ownload with succes!\n");
	}
  return;
}

static struct block_device_operations altera_block_fops = {
    .owner	= THIS_MODULE,
    .ioctl	= block_ioctl
};

// device driver init/ 1st function call when insmode
static int __init pci_drv_init(void)
{
  int err =0;
  if ( err = register_blkdev(DEVICE_MAJOR,DEVICE_NAME)) {
	  printk("ALTERA: Unable to get major %d. error number =%d\n", DEVICE_MAJOR,err);
	  return 0;
  }
	// Queue
	{
		blk_queue = blk_init_queue(blk_request, &PrivateData.lock);
		if (blk_queue == NULL)
						goto out_disk;
//		blk_queue_hardsect_size( blk_queue, hardsect_size); 
	}
  // register disk and fops
  {
		altera_gendisk = alloc_disk(1);

	  if( !altera_gendisk )	goto out_disk; 

		altera_gendisk -> major = DEVICE_MAJOR;
		altera_gendisk -> first_minor = 0; 
		altera_gendisk -> fops = &altera_block_fops;
		altera_gendisk -> queue = blk_queue;

		sprintf(altera_gendisk->disk_name, "altera_ram"); 

		PrivateData.gd = altera_gendisk;
		add_disk( altera_gendisk );
  }

  printk (KERN_CRIT "pci_chrdev_template: initialising\n");
  if( 0 == pci_register_driver(&altera_pci_drv) )
	  return 0;
out_queue:
	
out_disk:
  unregister_blkdev( DEVICE_MAJOR, DEVICE_NAME);

  return (-EIO);
}

// device driver unload/ called when rmmod
static void __exit pci_drv_exit(void)
{

	del_gendisk(altera_gendisk);
	blk_cleanup_queue( altera_gendisk->queue ); 
	//put_disk(altera_gendisk);

	pci_unregister_driver( &altera_pci_drv );
	unregister_blkdev( DEVICE_MAJOR, DEVICE_NAME );
	return;
}

////////////////////////////////////// Block Driver Handle
static int block_ioctl (struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{
	long size;
//	struct hd_geometry geo;

	switch(cmd) {
	/*
	 * The only command we need to interpret is HDIO_GETGEO, since
	 * we can't partition the drive otherwise.  We have no real
	 * geometry, of course, so make something up.
	 */
	   // case HDIO_GETGEO:
/*
		size = Device.size*(hardsect_size/KERNEL_SECTOR_SIZE);
		geo.cylinders = (size & ~0x3f) >> 6;
		geo.heads = 4;
		geo.sectors = 16;
		geo.start = 4;
		if (copy_to_user((void *) arg, &geo, sizeof(geo)))
			return -EFAULT;
		return 0;
*/
    }

    return -ENOTTY; /* unknown command */
}


// driver init
module_init(pci_drv_init);
module_exit(pci_drv_exit);

