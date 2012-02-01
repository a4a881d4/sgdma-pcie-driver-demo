// this test case is test and pass in FEDORA 15 32bit system.
// please add this two options into grub booting commands.
// /etc/grub.conf :  vmalloc=512MB intel_iommu=off
// vmalloc for large virtual addressing
// intel_iommu for disable intel chipset protection
//

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



// simple version number
#define _ALTERATW_SOFTWARE_VERSION_NUMBER 1.0

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)    /* not < 2.5 */
#  error "This kernel is too old: not supported by this file"
#endif

/* **************************************************************
 * Bar 0_1 - on-chip mem (r/w), data width 64, size 262144 bytes
 *           ddr2 sdram controller, data width 64
 *           so, we can address these mem from 0x0 ~ 0x1ffffff (128mb for 64 bit address)
 *
 * Bar 2 - Modular SGDMA dispatcher, mode: mem-mapped 2 mem-mapped
 *
 * **************************************************************/

#define DIR_BAR_NR 0
#define CSR_BAR_NR 2


// Please reference Qsys setting
#define ADDR_OCM	0x07000000	// ~ 0x0203ffff
#define ADDR_DDR	0x08000000	// ~ 0x07ffffff
#define ADDR_CRA	0x00000000	// ~ 0x00003fff
#define	ADDR_CSR	0x06000000	// ~ 0x0000401f
#define	ADDR_DES	0x06000020	// ~ 0x0000402f

MODULE_LICENSE("GPL");
MODULE_AUTHOR("kfchou at altera.com");
MODULE_DESCRIPTION("PCI +rom/ram device driver example");

// vendor and device id of the PCI device
#define VENDOR_ID 0x1172
#define DEVICE_ID 0xe001

#define DEBUG TRUE

static u32 patterns[] = {0x31a3, 0x7b821, 0x54ae, 0x4a42, 0 };

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

/**************************Function Declarations (end)*************************************/

/**************************Structure Define*******************************************/
static struct pci_driver altera_pci_drv =
{
  .name= "altera",
  .id_table= altera_pci_ids,
  .probe= handler_altera_device_probe,
  .remove= handler_altera_device_deinit,
};

void dump_memory( const char* msg, u32* __iomem begin, u32* __iomem end )
{
	if( begin > end )
		return;
	u32 counter = 1;
	printk( "dump memory begin (%s)\n",msg);
	for( counter = 2; begin < end ; begin ++, counter ++ )
	{
		printk(" %p = 0x%x ", begin, *(begin) );
		if( counter % 4 == 0 )
			printk("\n");
	}
	printk( "dump memory end (%s)\n",msg);
}

u32 __read( u32* __iomem bar_base, u32 addr)
{ 
	return bar_base[ addr / 4 ];
}

void __write( u32* __iomem bar_base, u32 addr, u32 wdata)
{ 
	bar_base[ addr / 4 ] = wdata ;
	wmb();
	return;
}


// PC -> FPGA
void DMA_Test_PC2FPGA( u32* __iomem bar0, u32* __iomem bar2, u32 target_addr, int length )
{
	u32 control_bits, a2p_mask, res, irq_mask ,status; 
	u32* copy_from = NULL;
	struct page *pc_page = NULL;
	u32 idx = 0;

	pc_page = alloc_pages(GFP_ATOMIC, 2);
	copy_from = kmap_atomic( pc_page, KM_USER1 );

	// prepare copy from patterns
	for( idx = 0 ; patterns[ idx ] ; idx ++ )
	{
		((u32*)copy_from)[ idx ] = patterns[idx];
	}

	// Prepare Data 
	__write( bar2, 0x1000, 0xFFFFFFFC);
	a2p_mask = __read(bar2, 0x1000);//0x1000 is the offset for translation register
	// program address translation table
	// PCIe core limits the data length to be 1MByte, so it only needs 20bits of address.
	if(a2p_mask == 0 ) {
		printk(" !!!!!!!!!!!!!!!!!!!!!!!!!error!!!!!!!!!!!!!!!!! a2p_mask = 0x%x\n", a2p_mask );
		goto err;
	}
	else
		printk("  a2p_mask = 0x%x\n", a2p_mask );

	__write( bar2, 0x1000, __pa(copy_from) & a2p_mask); //setting lower address
	__write( bar2, 0x1004, 0x0); // setting upper address   limited at hardIP for now.

	// clear the DMA contoller
	res = __read( bar2, ADDR_CSR);
	__write( bar2, ADDR_CSR, 0x200);
	res = __read( bar2, ADDR_CSR);
	__write( bar2, ADDR_CSR+0x04, 0x02);  // issue reset dispatcher
	__write( bar2, ADDR_CSR, 0x0000); //clear all the status
	__write( bar2, ADDR_CSR+0x04, 0x10); // set IRQ enable
	res = __read( bar2, ADDR_CSR);
	printk(" after reset %x\n", res);
	

	
        //enable_global_interrupt_mask (alt_u32 csr_base)
        irq_mask = __read( bar2, ADDR_CSR+0x04);
        irq_mask |= 0x10;
        __write( bar2, ADDR_CSR+0x04, irq_mask); //setting the IRQ enable flag at here 
	res = __read( bar2, ADDR_CSR+0x04);
        // clear the status
        status = __read( bar2, ADDR_CSR);
        __write( bar2, ADDR_CSR, 0x0); //clear all the status
        status = __read( bar2, ADDR_CSR);
        

	while (( __read( bar2, ADDR_CSR) & 0x04) != 0) {
		printk(" spin until there's room for another desciptor\n");
	}  // spin until there is room for another descriptor to be written to the SGDMA
	control_bits = (1<<14); //(1 << 24);  // 14bit is the IRQ, 24bit is the early done bit

	__write( bar2, ADDR_DES, __pa(copy_from) & (~a2p_mask) );
	__write( bar2, ADDR_DES + 4, target_addr );
	__write( bar2, ADDR_DES + 8, length);
	__write( bar2, ADDR_DES + 0xC, control_bits | (1<<31) );	// go


	for( idx = 0 ; idx < 50 ; idx ++ ) {
		res = __read( bar2, ADDR_CSR);
		if(res  == 0x202)  { // At least DMA has finished
			printk(" DMA finished \n");
			break;
		}
		else
			printk(" DMA did not finish CSR/Status 0x%04x\n",res);
	}

	res = 0;
	for( idx = 0 ; patterns[ idx ] ; idx ++ )
	{
		if(  __read( bar0, target_addr + idx*0x04 ) == patterns[idx] ) {
			printk(" DMA test pass (%d)\n", idx );
		} else {	
			printk(" DMA error !!! Patterns dismatch idx=%d expect=0x%x but get 0x%x\n",idx, patterns[idx],  __read( bar0, target_addr + idx*0x04 ));
			res = 1;
		}
	}
	if( res == 0)
		printk(" DMA Read / PC Memory -> FPGA success\n");
err:
	kunmap_atomic(copy_from, KM_USER1); 
//	free_page( pc_page );
}

// FPGA -> FPGA
void DMA_Test_FPGA2FPGA( u32* __iomem bar0, u32* __iomem bar2, u32 target_addr, u32 copy_from, int length )
{
	u32 control_bits, a2p_mask, res, irq_mask ,status; 
	u32 idx = 0;

	// prepare copy from patterns
	for( idx = 0 ; patterns[ idx ] ; idx ++ ) {
		*(bar0 + copy_from/4 + idx ) = patterns[idx];
	}

	// Prepare Data 
	__write( bar2, 0x1000, 0xFFFFFFFC);
	a2p_mask = __read(bar2, 0x1000);//0x1000 is the offset for translation register
	// program address translation table
	// PCIe core limits the data length to be 1MByte, so it only needs 20bits of address.
	if(a2p_mask == 0 ) {
		printk(" !!!!!!!!!!!!!!!!!!!!!!!!!error!!!!!!!!!!!!!!!!! a2p_mask = 0x%x\n", a2p_mask );
		goto err;
	}
	else
		printk("  a2p_mask = 0x%x\n", a2p_mask );

	__write( bar2, 0x1000, __pa(copy_from) & a2p_mask); //setting lower address
	__write( bar2, 0x1004, 0x0); // setting upper address   limited at hardIP for now.

	// clear the DMA contoller
	res = __read( bar2, ADDR_CSR);
	__write( bar2, ADDR_CSR, 0x200);
	res = __read( bar2, ADDR_CSR);
	__write( bar2, ADDR_CSR+0x04, 0x02);  // issue reset dispatcher
	__write( bar2, ADDR_CSR, 0x0000); //clear all the status
	__write( bar2, ADDR_CSR+0x04, 0x10); // set IRQ enable
	res = __read( bar2, ADDR_CSR);
	printk(" after reset %x\n", res);
	

	
        //enable_global_interrupt_mask (alt_u32 csr_base)
        irq_mask = __read( bar2, ADDR_CSR+0x04);
        irq_mask |= 0x10;
        __write( bar2, ADDR_CSR+0x04, irq_mask); //setting the IRQ enable flag at here 
	res = __read( bar2, ADDR_CSR+0x04);
        // clear the status
        status = __read( bar2, ADDR_CSR);
        __write( bar2, ADDR_CSR, 0x0); //clear all the status
        status = __read( bar2, ADDR_CSR);
        

	while (( __read( bar2, ADDR_CSR) & 0x04) != 0) {
		printk(" spin until there's room for another desciptor\n");
	}  // spin until there is room for another descriptor to be written to the SGDMA
	control_bits = (1<<14); //(1 << 24);  // 14bit is the IRQ, 24bit is the early done bit

	__write( bar2, ADDR_DES, copy_from );
	__write( bar2, ADDR_DES + 4, target_addr );
	__write( bar2, ADDR_DES + 8, length);
	__write( bar2, ADDR_DES + 0xC, control_bits | (1<<31) );	// go


	for( idx = 0 ; idx < 50 ; idx ++ ) {
		res = __read( bar2, ADDR_CSR);
		if(res  == 0x202)  { // At least DMA has finished
			printk(" DMA finished \n");
			break;
		}
		else
			printk(" DMA did not finish CSR/Status 0x%04x\n",res);
	}

	res = 0;
	for( idx = 0 ; patterns[ idx ] ; idx ++ )
	{
		if(  __read( bar0, target_addr + idx*0x04 ) == patterns[idx] ) {
			printk(" DMA test pass (%d)\n", idx );
		} else {	
			printk(" DMA error !!! Patterns dismatch idx=%d expect=0x%x but get 0x%x\n",idx, patterns[idx],  __read( bar0, target_addr + idx*0x04 ));
			res = 1;
		}
	}
	if( res == 0)
		printk(" DMA Read / FPGA -> FPGA success\n");
err:
	return;
}
// FPGA -> PC
void DMA_Test_FPGA2PC( u32* __iomem bar0, u32* __iomem bar2, u32 copy_from, int length )
{
	u32 control_bits, a2p_mask, res, irq_mask ,status; 
	struct page *pc_page = NULL;
	u32* target_to = 0;
	u32 idx = 0;

	pc_page = alloc_pages(GFP_ATOMIC, 2);
	target_to = kmap_atomic( pc_page, KM_USER1 );

	// prepare copy from patterns
	for( idx = 0 ; patterns[ idx ] ; idx ++ ) {
		*(bar0 + copy_from/4 + idx ) = patterns[idx];
	}

	// Prepare Data 
	__write( bar2, 0x1000, 0xFFFFFFFC);
	a2p_mask = __read(bar2, 0x1000);//0x1000 is the offset for translation register
	// program address translation table
	// PCIe core limits the data length to be 1MByte, so it only needs 20bits of address.
	if(a2p_mask == 0 ) {
		printk(" !!!!!!!!!!!!!!!!!!!!!!!!!error!!!!!!!!!!!!!!!!! a2p_mask = 0x%x\n", a2p_mask );
		goto err;
	}
	else
		printk("  a2p_mask = 0x%x\n", a2p_mask );

	__write( bar2, 0x1000, __pa(target_to) & a2p_mask); //setting lower address
	__write( bar2, 0x1004, 0x0); // setting upper address   limited at hardIP for now.

	// clear the DMA contoller
	res = __read( bar2, ADDR_CSR);
	__write( bar2, ADDR_CSR, 0x200);
	res = __read( bar2, ADDR_CSR);
	__write( bar2, ADDR_CSR+0x04, 0x02);  // issue reset dispatcher
	__write( bar2, ADDR_CSR, 0x0000); //clear all the status
	__write( bar2, ADDR_CSR+0x04, 0x10); // set IRQ enable
	res = __read( bar2, ADDR_CSR);
	printk(" after reset %x\n", res);
	

	
        //enable_global_interrupt_mask (alt_u32 csr_base)
        irq_mask = __read( bar2, ADDR_CSR+0x04);
        irq_mask |= 0x10;
        __write( bar2, ADDR_CSR+0x04, irq_mask); //setting the IRQ enable flag at here 
	res = __read( bar2, ADDR_CSR+0x04);
        // clear the status
        status = __read( bar2, ADDR_CSR);
        __write( bar2, ADDR_CSR, 0x0); //clear all the status
        status = __read( bar2, ADDR_CSR);
        

	while (( __read( bar2, ADDR_CSR) & 0x04) != 0) {
		printk(" spin until there's room for another desciptor\n");
	}  // spin until there is room for another descriptor to be written to the SGDMA
	control_bits = (1<<14); //(1 << 24);  // 14bit is the IRQ, 24bit is the early done bit

	__write( bar2, ADDR_DES, copy_from);
	__write( bar2, ADDR_DES + 4, __pa(target_to) & (~a2p_mask) );
	__write( bar2, ADDR_DES + 8, length);
	__write( bar2, ADDR_DES + 0xC, control_bits | (1<<31) );	// go


	for( idx = 0 ; idx < 50 ; idx ++ ) {
		res = __read( bar2, ADDR_CSR);
		if(res  == 0x202)  { // At least DMA has finished
			printk(" DMA finished \n");
			break;
		}
		else
			printk(" DMA did not finish CSR/Status 0x%04x\n",res);
	}

	res = 0;
	for( idx = 0 ; patterns[ idx ] ; idx ++ )
	{
		if(  target_to[idx] == patterns[idx] ) {
			printk(" DMA test pass (%d)\n", idx );
		} else {	
			printk(" DMA error !!! Patterns dismatch idx=%d expect=0x%x but get 0x%x\n",idx, patterns[idx], target_to[idx]);
			res = 1;
		}
	}
	if( res == 0)
		printk(" DMA Read / FPGA -> PC Memory success\n");
err:
	kunmap_atomic(target_to, KM_USER1); 
//	free_page( pc_page );
}

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

	err = pci_enable_device (dev); 
	printk("pci_enable_device = %d rev=%d\n",err,dev->revision);

	if( err ) {
		printk(" pci device enable failed \n");
		return err;
	}

	
	// try to access FPGA via BAR

	printk(" pci/fpga device rev id = 0x%x\n",dev->revision);

	u32 __iomem * bar0 = ioremap_nocache( pci_resource_start(dev, DIR_BAR_NR), pci_resource_len(dev, DIR_BAR_NR) );
	u32 __iomem * bar2 = ioremap_nocache( pci_resource_start(dev, CSR_BAR_NR), pci_resource_len(dev, CSR_BAR_NR) ); 

	DMA_Test_PC2FPGA( bar0, bar2, ADDR_OCM , 512 ); 
//	DMA_Test_FPGA2FPGA( bar0, bar2, ADDR_OCM, ADDR_OCM+0x20000 , 512 ); 
//	DMA_Test_FPGA2PC( bar0, bar2, ADDR_OCM , 512 ); 

	iounmap( bar0 );
	iounmap( bar2 ); 
	// try to access FPGA via BAR (end) 

	printk(" end of probing (success)\n");
	return (0);
}


static void
handler_altera_device_deinit( struct pci_dev *pdev )
{
	pci_disable_device( pdev ); 
	return;
}

// device driver init/ 1st function call when insmode
static int __init pci_drv_init(void)
{
	printk(KERN_DEBUG "sgdma_gen1x4: installing !\n"); 

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
