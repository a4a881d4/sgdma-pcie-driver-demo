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

#define DIR_BAR_NR 0
#define CSR_BAR_NR 2

MODULE_LICENSE("GPL");
MODULE_AUTHOR("kfchou at altera.com");
MODULE_DESCRIPTION("PCI +rom/ram device driver example");

// vendor and device id of the PCI device
#define VENDOR_ID 0x1172
#define DEVICE_ID 0xe001

#define DRV_NAME "ALTERA_TEST"
#define DEVICE_NAME "altera_test"
#define MAJOR_NUM 240 /* free mayor number, see devices.txt */
#define MAX_PARTITION_NR 16

#define ALT_MAX_REQ_SG 64
#define USE_DISK 
#undef USE_PURE_PCI_FUNC
#define USE_64_ADDR
#define SECTOR_SIZE 512


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
static int handler_altera_device_probe(struct pci_dev *, const struct pci_device_id *);
static void handler_altera_device_deinit( struct pci_dev *);

// block driver
static int block_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
/**************************Function Declarations (end)*************************************/

/**************************Structure Define*******************************************/
static struct pci_driver altera_pci_drv =
{
  .name= "altera",
  .id_table= altera_pci_ids,
  .probe= handler_altera_device_probe,
  .remove= handler_altera_device_deinit,
};

struct AlteraBlockDevice
{
	unsigned long size;
	spinlock_t lock;

#ifdef USE_64_ADDR
	u64 __iomem *data;
#else
	void  __iomem *data;
#endif


	void  __iomem *csr;


#ifdef USE_DISK 
	struct gendisk *gd;
#endif

	struct scatterlist              sg[ALT_MAX_REQ_SG];

} * altera_block_device = NULL;

//static struct request_queue *blk_queue; 

/**************************Structure Define (end)*************************************/
// example for reading a PCI config byte
/*
static unsigned char
get_revision(struct pci_dev *dev)
{
  u8 revision;

  pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
  return (revision);
}*/


void dump_memory_in_char( const char* str, volatile u32* mem, size_t n)
{
	int idx = 0;
//	printk(" (%s) dump address 0x%lx ",str,mem);
	printk(" (%s) dump  ",str);
	for( idx = 0 ; idx < n; idx ++ )
		printk("\'%c\'", mem[idx] );
	printk("\n");
}
void dump_memory_in_char2( const char* str, volatile u64 __iomem* mem, size_t n)
{
	int idx = 0;
	printk(" (%s) dump address 0x%llx ",str,mem);
	for( idx = 0 ; idx < n; idx ++ )
		printk("\'%c\'", (char) ioread32( mem+idx ) );
	printk("\n");
}
#ifdef USE_64_ADDR
void write_to_pci( volatile u64 __iomem * pci, u32* mem, size_t n )
#else
void write_to_pci( volatile u32 __iomem * pci, u32* mem, size_t n )
#endif
{
	n = n * (sizeof( u8 ) / sizeof ( u32 ));
	memcpy_toio(pci, mem, n);
	return;

#ifdef USE_64_ADDR
	volatile u64 __iomem * pci_before = pci;
#else
	volatile u32 __iomem * pci_before = pci;
#endif
	u32* mem_before = mem;

	n = n * (sizeof( u8 ) / sizeof ( u32 ));

	while( n -- )
	{
		//writeq( *mem++, pci++);
		//writel( *mem++, pci++);
#ifdef USE_64_ADDR
	//	*pci++ = (u64) * mem++;
		iowrite32(*mem++, pci++);
		wmb(  );
#else
		*pci++ = *mem++;
#endif
	}
	wmb();
	dump_memory_in_char( "memory before ", mem_before, 20);
	dump_memory_in_char2( "pci after ", pci_before, 20);
}
#ifdef USE_64_ADDR
void read_from_pci( volatile u64 __iomem * pci, u32* mem, size_t n )
#else
void read_from_pci( volatile u32 __iomem * pci, u32* mem, size_t n )
#endif
{
	n = n * (sizeof( u8 ) / sizeof ( u32 ));
	memcpy_fromio(mem, pci, n);
	return;

	n = n * (sizeof( u8 ) / sizeof ( u32 ));

	while( n -- )
	{
		//*mem++ = readl( pci++);
		//*mem++ = * pci++;
#ifdef USE_64_ADDR
		//*mem++ = (u64) * pci++;
		*mem++ = ioread32(pci++);
		rmb(  );
#else
		*mem++ = * pci++;
#endif
	}
}

/*
static irqreturn_t pci_isr( int irq, void *dev_id, struct pt_regs *regs )
{
  // nothing todo for the irq yet
  return (IRQ_HANDLED);
}
*/

static void blk_request( struct request_queue *q)
{
//	printk("\n in blk_request \n");
        struct request *req = blk_fetch_request(q);
	u32 block, count, cur_bytes;
	int res, idx, test;
	bool do_write;
	int size, n_elem;
	u64 __iomem* pci_addr = 0;

	struct AlteraBlockDevice* abd = NULL;
        while (req) {
//		printk("\n into blk_request loop \n");
                block = blk_rq_pos(req);
                count = blk_rq_cur_sectors(req);
		cur_bytes = blk_rq_cur_bytes(req);
                res = -EIO;
		

                if (req->cmd_type != REQ_TYPE_FS)
		{
			printk(" blk_request == TYPE_FS \n" );
                        goto done;
		}
                if (block + count > get_capacity(req->rq_disk))
		{
			printk(" blk_request out of size\n" );
                        goto done;
		}

		sg_init_table( altera_block_device->sg, ALT_MAX_REQ_SG);

//		n_elem = blk_rq_map_sg(q, req, altera_block_device->sg ); 
//		n_elem = pci_map_sg(host->pdev, sg, n_elem, pci_dir);


		do_write = (rq_data_dir(req) == WRITE );
		size	= blk_rq_bytes( req );

	//	printk(" --------------- cur_bytes = %lx block = %lx count = %lx size = %lx, buffer = %lx \n", cur_bytes, block, count, size,req->buffer );

		if ( 0 && do_write )
		{
			dump_memory_in_char( "before a dump", req->buffer, 20);
		}

		abd = (struct AlteraBlockDevice*) ( req->rq_disk->private_data );
		if(0)
		for( idx = 0 ; idx < 1024; idx ++ )
		{
		//	iowrite32( ram_base+idx, idx );
		//	test = ioread32( ram_base+ idx);
		//	ram_base[idx] = idx;
			test = *(abd->data+idx);
			printk("in io request mem %llx = 0x%x\n", abd->data+idx, test );
		}
		//pci_addr = abd -> data+(block<<9);
		pci_addr = abd -> data;

		if( do_write )
		{
			int n = size * (sizeof( u64 ) / sizeof ( u8 ));
			volatile u64 __iomem * copy_to = pci_addr;
			u32 idx = 0;
			for( idx = 0 ; idx < size ; idx ++ )
				*(copy_to ++ ) = (u64*)( req->buffer + idx );
		}
		else
		{
			size_t n = size * (sizeof( u64 ) / sizeof ( u8 ));
			volatile u64 __iomem * copy_from = pci_addr;
			u32 idx = 0;

			for( idx = 0 ; idx < n; idx ++ )
			//for( idx = 0 ; idx < 50; idx ++ )
			{
				//if( copy_from > abd->data && copy_from < abd->data+abd->size )
				if( (copy_from - abd->data) < 4096 && copy_from>=abd->data )
				{
					printk("in io request mem2 %llx = 0x%x\n", copy_from, *(copy_from));
					 *((u64*)( req->buffer + idx )) = *(copy_from);
				}
				else
					printk(" out of memory \n");

				printk(" just try to read 0x%lx\n",copy_from);
				copy_from ++;
			}

		}


		res = 0;
        done:  
                /* wrap up, 0 = success, -errno = fail */
                if (!__blk_end_request_cur(req, res))
                        req = blk_fetch_request(q);
        }

//	printk(" end blk_request \n");
}
int block_ioctl (struct inode *inode, struct file *filp,
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
		geo.sectors = SECTOR_SIZE;
		geo.start = 0;
		geo.heads = 64;

		geo.cylinders = (altera_block_device->size / (geo.sectors*geo.heads)*4 );
		if (copy_to_user((void *) arg, &geo, sizeof(geo)))
			return -EFAULT;
		return 0;
	}

    return -ENOTTY; 
}

static struct request_queue *altera_block_queue = NULL;
static struct block_device_operations altera_block_fops = {
    .owner           = THIS_MODULE,
    .ioctl	     = block_ioctl
};

int block_device_init( struct pci_dev *pdev )
{
	int ret = register_blkdev(MAJOR_NUM, DEVICE_NAME);

#ifdef USE_PURE_PCI_FUNC

#ifdef USE_64_ADDR
	u64  __iomem * ram_base = pci_ioremap_bar( pdev, DIR_BAR_NR);
#else
	void __iomem * ram_base = pci_ioremap_bar( pdev, DIR_BAR_NR);
#endif
	void __iomem * csr_base = pci_ioremap_bar( pdev, CSR_BAR_NR);
#else

#ifdef USE_64_ADDR
	u64 __iomem * ram_base = ioremap_nocache( pci_resource_start(pdev, DIR_BAR_NR), pci_resource_len(pdev, DIR_BAR_NR) );
//	u64 __iomem * ram_base = ioremap( pci_resource_start(pdev, DIR_BAR_NR), pci_resource_len(pdev, DIR_BAR_NR) );
#else
	void __iomem * ram_base = ioremap_nocache( pci_resource_start(pdev, DIR_BAR_NR), pci_resource_len(pdev, DIR_BAR_NR) );
#endif
	void __iomem * csr_base = ioremap_nocache( pci_resource_start(pdev, CSR_BAR_NR), pci_resource_len(pdev, CSR_BAR_NR));
#endif

	printk(" in blk device init pci=%lx - ram_base=%lx length=%lx\n", ram_base, pci_resource_start(pdev, DIR_BAR_NR), pci_resource_len(pdev, DIR_BAR_NR) );
	{
		int idx = 0, test = 0;;
		for( idx = 0 ; idx < 1024; idx ++ )
		{
		//	iowrite32( ram_base+idx, idx );
		//	test = ioread32( ram_base+ idx);
		//	ram_base[idx] = idx;
			test = *(ram_base+idx);
			printk("before pci mem %llx = 0x%x\n", ram_base+idx, test );
		}
/*
		for( idx = 0 ; idx < 4 ; idx ++ )
		{
		//	iowrite32( ram_base+idx, idx );
		//	test = ioread32( ram_base+ idx);
			*(ram_base+idx) = idx+3;
			test = ram_base[idx];
			printk("after pci mem %llx = 0x%x\n", ram_base+idx, test );
		}
*/

	}

	printk(KERN_WARNING "altera_drv: io resource start %llx\n",  pci_resource_start(pdev, DIR_BAR_NR));

	if (ret < 0) 
	{
		printk(KERN_WARNING "altera_drv: unable to get major number\n");
		return -EBUSY;
	}

	altera_block_device = kmalloc(sizeof(struct AlteraBlockDevice), GFP_KERNEL);

	if( altera_block_device == NULL )
	{
		printk(KERN_WARNING "minidb: unable to get memory with kmalloc\n");
		goto out_unregister;
	}
	
	memset( altera_block_device, 0, sizeof( struct AlteraBlockDevice ) );
//	altera_block_device -> size = mem_size;
	altera_block_device -> data = ram_base;
	altera_block_device -> csr  = csr_base;

//	printk(" %s - data address %x\n", __FUNCTION__, altera_block_device -> data );
	spin_lock_init(&altera_block_device->lock);

	altera_block_queue = blk_init_queue( blk_request, &altera_block_device->lock );
//	blk_queue_flush( altera_block_queue, REQ_FLUSH ); 

	if (altera_block_queue == NULL) {
		printk(KERN_WARNING "altera_drv: error in blk_init_queue\n");
		goto out_free;
	}
#ifdef USE_DISK
	altera_block_device->gd = alloc_disk( MAX_PARTITION_NR );
	if (!altera_block_device->gd) {
		printk(KERN_WARNING "altera_drv: error in alloc_disk\n");
		goto out_free;
	}

	altera_block_device->gd->major = MAJOR_NUM;
	altera_block_device->gd->first_minor = 0;
	altera_block_device->gd->fops = &altera_block_fops;
	altera_block_device->gd->private_data = altera_block_device;
	//snprintf(altera_block_device->gd->disk_name, 13, "%s%d", DEVICE_NAME, 0);
	sprintf(altera_block_device->gd->disk_name, "%s%d", DEVICE_NAME, 0);
	//strcpy(altera_block_device->gd->disk_name, DEVICE_NAME);
	//
	set_capacity( altera_block_device->gd, pci_resource_len(pdev, DIR_BAR_NR)>>10 );
	printk(" altera_block disk capacity = %x\n",get_capacity( altera_block_device->gd));

	altera_block_device->gd->queue = altera_block_queue;

	add_disk(altera_block_device->gd);
#endif
	
	return 0;

out_free:
	kfree( altera_block_device );
out_unregister:
	unregister_blkdev(MAJOR_NUM, DEVICE_NAME);

	return -ENOMEM;

}

// Initialising of the module with output about the irq, I/O region and memory region.
static
int handler_altera_device_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int err; 
	//const int test_bar_nr = 0;

//	long iosize_0,iosize_1;
	printk(KERN_DEBUG "altera : Device 0x%x has been found at"
		" bus %d dev %d func %d\n",
		dev->device, dev->bus->number, PCI_SLOT(dev->devfn),
		PCI_FUNC(dev->devfn));

	err = pcim_enable_device (dev); 
	printk("pci_enable_device = %d\n",err);
	if( err )
	{
		printk(" pci device enable failed \n");
		return err;
	}

	
	if(1)
	{
		void __iomem * ram_base = ioremap_nocache( pci_resource_start(dev, DIR_BAR_NR), pci_resource_len(dev, DIR_BAR_NR) );
		printk("<0> resource_start ==> %lx, ram_base =%lx\n", pci_resource_start(dev,DIR_BAR_NR), ram_base );

		volatile u64 *p;
		int idx = 0;
//		p = ioremap( ram_base, pci_resource_len(dev, DIR_BAR_NR) );
		p = ram_base;
		
		//for( idx = 0 ; idx < pci_resource_len(dev, DIR_BAR_NR) ; idx ++ )
		for( idx = 0 ; idx < 1024; idx ++ )
		{
			*(p+idx) = idx;
			if( *(p+idx) != idx )
				printk(" %lx error ", (p+idx));
			else
				printk(" %lx pass ", (p+idx));
			printk("\n");
		}
		printk("<0> CIH ==> %x\n", *p);

		iounmap( ram_base );

		printk(" ------------->bar01 0x%lx \n", pci_resource_flags(dev, 0) );
		printk(" ------------->bar 2 0x%lx \n", pci_resource_flags(dev, 2) );
	}	

//	long iosize_0 = pci_resource_len( dev, 0 );
//	long iosize_1 = pci_resource_len( dev, 1 );

//	printk(" bar0 %d, bar1 %d\n",iosize_0, iosize_1);

	if (pci_set_dma_mask(dev, DMA_BIT_MASK(64)) &&
			pci_set_dma_mask(dev, DMA_BIT_MASK(32))) {
		dev_printk(KERN_WARNING, &dev->dev, "NO suitable DMA found\n");
		return  -ENOMEM;
	}


	printk(" before pci_request_region \n");
	if( pci_request_region(dev, DIR_BAR_NR, DRV_NAME) )
	{
                printk(KERN_WARNING "altera_test0: can't request iomem (0x%llx).\n",
                       (unsigned long long)pci_resource_start(dev,0));
                return -EBUSY;
        }
	if( pci_request_region(dev, CSR_BAR_NR, DRV_NAME) )
	{
                printk(KERN_WARNING "altera_test0: can't request iomem (0x%llx).\n",
                       (unsigned long long)pci_resource_start(dev,CSR_BAR_NR));
                return -EBUSY;
        }
	printk(" after pci_request_region \n");


//	pci_set_master(dev);
//	pci_set_mwi( dev );


	dev_err(&dev->dev, "mem resource at PCI BAR #%d, device resource flags is 0x%lx\n",0,(pci_resource_flags(dev,0)));

	block_device_init( dev );

	printk(" hey i'm here in probing ");
	return (0);
}


static void
handler_altera_device_deinit( struct pci_dev *pdev )
{
	if(altera_block_device )
	{
#ifdef USE_DISK 
		del_gendisk(altera_block_device->gd);
		put_disk(altera_block_device->gd);
#endif

		unregister_blkdev(MAJOR_NUM, DEVICE_NAME);
		blk_cleanup_queue( altera_block_queue );

		if( altera_block_device -> data )
		{
#ifdef USE_PURE_PCI_FUNC
			pci_iounmap( pdev, altera_block_device -> data);
			pci_iounmap( pdev, altera_block_device -> csr );
#else
			iounmap( altera_block_device -> data );
			iounmap( altera_block_device -> csr  );
#endif
//			pci_iounmap( pdev, altera_block_device -> data);
		}

		kfree( altera_block_device );

		printk(KERN_DEBUG "altera_drv: ownload with succes!\n");
	}
	pci_release_region( pdev, DIR_BAR_NR );
	pci_release_region( pdev, CSR_BAR_NR );
	pci_disable_device( pdev ); 

	return;
}

// device driver init/ 1st function call when insmode
static int __init pci_drv_init(void)
{
	printk(KERN_DEBUG "altera_drv: installing !\n");
	
	if( 0 == pci_register_driver(&altera_pci_drv) )
		return 0;
	return 0;
}

// device driver unload/ called when rmmod
static void __exit pci_drv_exit(void)
{
	pci_unregister_driver( &altera_pci_drv );
	return;
}


// driver init
module_init(pci_drv_init);
module_exit(pci_drv_exit);
