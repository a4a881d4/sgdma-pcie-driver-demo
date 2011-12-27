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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/blkdev.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/compiler.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/hdreg.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/scatterlist.h>
#include <asm/io.h>
#include <asm/uaccess.h>



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
	unsigned long data_length;
	spinlock_t lock;

	u64 __iomem *data;
	void  __iomem *csr;

	struct gendisk *gd;

	struct scatterlist              sg[ALT_MAX_REQ_SG];

} * altera_block_device = NULL;

u32 __read( u32* __iomem base, u32 addr){

//	return *((u32*)(base+addr));
	return base[ addr / 4 ];
}

void __write( u32* __iomem base, u32 addr, u32 wdata){


	base[ addr / 4 ] = wdata ;
	return;
//   WDC_WriteAddr8(hDev,bar,addr   , wdata    & 0xFF);
	base[ addr / 4 ] = wdata &0xFF;
//   WDC_WriteAddr8(hDev,bar,addr+1 , (wdata>>8)  & 0xFF);
	base[ addr / 4 + 1 ] = (wdata>>8) & 0xFF;
//   WDC_WriteAddr8(hDev,bar,addr+2 , (wdata>>16) & 0xFF);
	base[ addr / 4 + 2 ] = (wdata>>16) & 0xFF;
//   WDC_WriteAddr8(hDev,bar,addr+3 , (wdata>>24) & 0xFF);
	base[ addr / 4 + 3 ] = (wdata>>24) & 0xFF; 
}


// PC -> FPGA
void DMA_Read_Test( u32* __iomem bar0, u32* __iomem bar2, u32 target_addr, int length )
{
	void * copy_from;
	struct page *pc_page;
	pc_page = alloc_pages(GFP_DMA | GFP_ATOMIC, 2);
	copy_from = kmap_atomic( pc_page, KM_USER1 );
	u32 control_bits;


	if ( 1 )
	{
		((u32*)copy_from)[ 0 ] = 1234;
		((u32*)copy_from)[ 1 ] = 5678;
		printk("  copy_from %x = %d\n", copy_from,  ((u32*)copy_from)[ 0 ] );
		printk("  copy_from %x = %d\n", copy_from+0x04,  *((u32*)(copy_from+0x04) ) );

		((u32*)copy_from)[ 0 ] = 0x0000;
		((u32*)copy_from)[ 1 ] = 0xffff;
		printk("  copy_from %x = %d\n", copy_from,  ((u32*)copy_from)[ 0 ] );
		printk("  copy_from %x = %d\n", copy_from+0x04,  *((u32*)(copy_from+0x04) ) );
		printk("  copy_from __pa(%x) = 0x%x\n", copy_from, __pa(copy_from) );
	}


	// Prepare Data
	

	__write( bar2, 0x1000, 0xFFFFFFFC);
	u32 a2p_mask = __read(bar2, 0x1000);//0x1000 is the offset for translation register
	// program address translation table
	// PCIe core limits the data length to be 1MByte, so it only needs 20bits of address.
	__write( bar2, 0x1000, __pa(copy_from)); //setting lower address
	__write( bar2, 0x1004, 0x0); // setting upper address   limited at hardIP for now.

	// clear the DMA contoller
	int res = __read( bar2, 0x06000000);
	__write( bar2, 0x06000000, 0x200);
	res = __read( bar2, 0x06000000);
	__write( bar2, 0x06000004, 0x02);  // issue reset dispatcher
	__write( bar2, 0x06000000, 0x0000); //clear all the status
	__write( bar2, 0x06000004, 0x10); // set IRQ enable
	res = __read( bar2, 0x06000000);
	

	
        //enable_global_interrupt_mask (alt_u32 csr_base)
        int irq_mask = __read( bar2, 0x06000004);
        irq_mask |= 0x10;
        __write( bar2, 0x06000004, irq_mask); //setting the IRQ enable flag at here 
        // clear the status
        int status = __read( bar2, 0x06000000);
        __write( bar2, 0x06000000, 0x0); //clear all the status
        status = __read( bar2, 0x06000000);
        

	while (( __read( bar2, 0x06000000) & 0x04) != 0) 
	{
		printk(" spin until there's room for another desciptor\n");
	}  // spin until there is room for another descriptor to be written to the SGDMA
	control_bits = (1<<14); //(1 << 24);  // 14bit is the IRQ, 24bit is the early done bit

	__write( bar2, 0x06000020, __pa(copy_from) & (~a2p_mask) );
	__write( bar2, 0x06000024, target_addr );
	__write( bar2, 0x06000028, 1024 );
	__write( bar2, 0x0600002C, control_bits | (1<<31) );


	res = __read( bar2, 0x06000000);
        if(res  == 0x202)  { // At least DMA has finished
		printk(" DMA finished \n");
        }
	else
		printk(" DMA did not finish\n");


	if( 1 )
	{
		__write( bar0, target_addr, 0x99887766 );
		__write( bar0, target_addr+0x4, 0x55443322);
		printk(" bar0 %x = 0x%x\n ", target_addr, __read( bar0, target_addr ) );
		printk(" bar0 %x = 0x%x\n ", target_addr + 0x04, __read( bar0, target_addr + 0x04) );
	}

	kunmap_atomic(copy_from, KM_USER1); 
	free_page( pc_page );
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
	printk("pci_enable_device = %d rev=%d\n",err,dev->revision);
	if( err )
	{
		printk(" pci device enable failed \n");
		return err;
	}

	
	if(1)
	{
		printk(" ------------->bar 0 0x%lx \n", pci_resource_flags(dev, 0) );
		printk(" ------------->bar 2 0x%lx \n", pci_resource_flags(dev, 2) );

		printk(" -----len----->bar 0 0x%lx \n", pci_resource_len(dev, 0) );
		printk(" -----len----->bar 2 0x%lx \n", pci_resource_len(dev, 2) );

		u32 __iomem * bar0 = ioremap_nocache( pci_resource_start(dev, 0), pci_resource_len(dev, 0) );
		u32 __iomem * bar2 = ioremap_nocache( pci_resource_start(dev, 2), pci_resource_len(dev, 2) );

		DMA_Read_Test( bar0, bar2, 0x7000000, 1024 );


		iounmap( bar0 );
		iounmap( bar2 );

	}	


	printk(" hey i'm here in probing ");
	return (0);
}


static void
handler_altera_device_deinit( struct pci_dev *pdev )
{
	pci_disable_device( pdev ); 
//	kfree( altera_block_device ); 
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
