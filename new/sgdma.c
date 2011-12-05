
#undef DEBUG	

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/bio.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/gfp.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>

#include <linux/fcntl.h>        /* O_ACCMODE */
#include <linux/hdreg.h>  /* HDIO_GETGEO */

#include "sgdma.h"

#include <asm/uaccess.h>
#include <asm/io.h>

#include "csr_regs.h"

#define MM_MAXCARDS 4
#define MM_RAHEAD 2      /* two sectors */
#define MM_BLKSIZE 1024  /* 1k blocks */
#define MM_HARDSECT 512  /* 512-byte hardware sectors */



static int debug;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug bitmask");

static int major_nr;

#include <linux/blkdev.h>
#include <linux/blkpg.h>

struct CardInfo {
	struct pci_dev	*dev;

	unsigned char	__iomem *csr_remap;
	unsigned int	mm_size;  /* size in kbytes */

	struct bio	*bio, *currentbio, **biotail;

	int		current_idx;
	sector_t	current_sector;

	struct request_queue *queue;

#if 0
	struct mm_page {
		dma_addr_t		page_dma;
//		struct mm_dma_desc	*desc;
		int	 		cnt, headcnt;
		struct bio		*bio, **biotail;
		int			idx;
	} mm_pages[2];
//#define DESC_PER_PAGE ((PAGE_SIZE*2)/sizeof(struct mm_dma_desc))
#endif

	int  Active, Ready;

	struct tasklet_struct	tasklet;
	unsigned int dma_status;

	spinlock_t 	lock;

	int		flags;
};

static struct CardInfo cardinfo;

//static int num_cards;

static struct gendisk *alt_gendisk;



#ifdef MM_DIAG
static void dump_regs(struct CardInfo *card)
{
	unsigned char *p;
	int i, i1;

	p = card->csr_remap;
	for (i = 0; i < 8; i++) {
		printk(KERN_DEBUG "%p   ", p);

		for (i1 = 0; i1 < 16; i1++)
			printk("%02x ", *p++);

		printk("\n");
	}
}
#endif


/*
 * Theory of request handling
 *
 * Each bio is assigned to one mm_dma_desc - which may not be enough FIXME
 * We have two pages of mm_dma_desc, holding about 64 descriptors
 * each.  These are allocated at init time.
 * One page is "Ready" and is either full, or can have request added.
 * The other page might be "Active", which DMA is happening on it.
 *
 * Whenever IO on the active page completes, the Ready page is activated
 * and the ex-Active page is clean out and made Ready.
 * Otherwise the Ready page is only activated when it becomes full.
 *
 * If a request arrives while both pages a full, it is queued, and b_rdev is
 * overloaded to record whether it was a read or a write.
 *
 * The interrupt handler only polls the device to clear the interrupt.
 * The processing of the result is done in a tasklet.
 */

static void mm_start_io(struct CardInfo *card)
{
#if 0
	/* we have the lock, we know there is
	 * no IO active, and we know that card->Active
	 * is set
	 */
	struct mm_dma_desc *desc;
	struct mm_page *page;
	int offset;

	/* make the last descriptor end the chain */
	page = &card->mm_pages[card->Active];
	pr_debug("start_io: %d %d->%d\n",
		card->Active, page->headcnt, page->cnt - 1);
	desc = &page->desc[page->cnt-1];

	desc->control_bits |= cpu_to_le32(DMASCR_CHAIN_COMP_EN);
	desc->control_bits &= ~cpu_to_le32(DMASCR_CHAIN_EN);
	desc->sem_control_bits = desc->control_bits;

	desc = &page->desc[page->headcnt];
	writel(0, card->csr_remap + DMA_PCI_ADDR);
	writel(0, card->csr_remap + DMA_PCI_ADDR + 4);

	writel(0, card->csr_remap + DMA_LOCAL_ADDR);
	writel(0, card->csr_remap + DMA_LOCAL_ADDR + 4);

	writel(0, card->csr_remap + DMA_TRANSFER_SIZE);
	writel(0, card->csr_remap + DMA_TRANSFER_SIZE + 4);

	writel(0, card->csr_remap + DMA_SEMAPHORE_ADDR);
	writel(0, card->csr_remap + DMA_SEMAPHORE_ADDR + 4);

	offset = ((char *)desc) - ((char *)page->desc);
	writel(cpu_to_le32((page->page_dma+offset) & 0xffffffff),
	       card->csr_remap + DMA_DESCRIPTOR_ADDR);
	/* Force the value to u64 before shifting otherwise >> 32 is undefined C
	 * and on some ports will do nothing ! */
	writel(cpu_to_le32(((u64)page->page_dma)>>32),
	       card->csr_remap + DMA_DESCRIPTOR_ADDR + 4);

	/* Go, go, go */
	writel(cpu_to_le32(DMASCR_GO | DMASCR_CHAIN_EN | pci_cmds),
	       card->csr_remap + DMA_STATUS_CTRL);
#endif
}

static int add_bio(struct CardInfo *card);

#if 0
static void activate(struct CardInfo *card)
{
	/* if No page is Active, and Ready is
	 * not empty, then switch Ready page
	 * to active and start IO.
	 * Then add any bh's that are available to Ready
	 */

	do {
		while (add_bio(card))
			;

		if (card->Active == -1 &&
		    card->mm_pages[card->Ready].cnt > 0) {
			card->Active = card->Ready;
			card->Ready = 1-card->Ready;
			mm_start_io(card);
		}

	} while (card->Active == -1 && add_bio(card));
}

static inline void reset_page(struct mm_page *page)
{
	page->cnt = 0;
	page->headcnt = 0;
	page->bio = NULL;
	page->biotail = &page->bio;
}
#endif

/*
 * If there is room on Ready page, take
 * one bh off list and add it.
 * return 1 if there was room, else 0.
 */
static int add_bio(struct CardInfo *card)
{
#if 0
	struct mm_page *p;
	struct mm_dma_desc *desc;
	dma_addr_t dma_handle;
	int offset;
	struct bio *bio;
	struct bio_vec *vec;
	int idx;
	int rw;
	int len;

	bio = card->currentbio;
	if (!bio && card->bio) {
		card->currentbio = card->bio;
		card->current_idx = card->bio->bi_idx;
		card->current_sector = card->bio->bi_sector;
		card->bio = card->bio->bi_next;
		if (card->bio == NULL)
			card->biotail = &card->bio;
		card->currentbio->bi_next = NULL;
		return 1;
	}
	if (!bio)
		return 0;
	idx = card->current_idx;

	rw = bio_rw(bio);
	if (card->mm_pages[card->Ready].cnt >= DESC_PER_PAGE)
		return 0;

	vec = bio_iovec_idx(bio, idx);
	len = vec->bv_len;
	dma_handle = pci_map_page(card->dev,
				  vec->bv_page,
				  vec->bv_offset,
				  len,
				  (rw == READ) ?
				  PCI_DMA_FROMDEVICE : PCI_DMA_TODEVICE);

	p = &card->mm_pages[card->Ready];
	desc = &p->desc[p->cnt];
	p->cnt++;
	if (p->bio == NULL)
		p->idx = idx;
	if ((p->biotail) != &bio->bi_next) {
		*(p->biotail) = bio;
		p->biotail = &(bio->bi_next);
		bio->bi_next = NULL;
	}

	desc->data_dma_handle = dma_handle;

	desc->pci_addr = cpu_to_le64((u64)desc->data_dma_handle);
	desc->local_addr = cpu_to_le64(card->current_sector << 9);
	desc->transfer_size = cpu_to_le32(len);
	offset = (((char *)&desc->sem_control_bits) - ((char *)p->desc));
	desc->sem_addr = cpu_to_le64((u64)(p->page_dma+offset));
	desc->zero1 = desc->zero2 = 0;
	offset = (((char *)(desc+1)) - ((char *)p->desc));
	desc->next_desc_addr = cpu_to_le64(p->page_dma+offset);
	desc->control_bits = cpu_to_le32(DMASCR_GO|DMASCR_ERR_INT_EN|
					 DMASCR_PARITY_INT_EN|
					 DMASCR_CHAIN_EN |
					 DMASCR_SEM_EN |
					 pci_cmds);
	if (rw == WRITE)
		desc->control_bits |= cpu_to_le32(DMASCR_TRANSFER_READ);
	desc->sem_control_bits = desc->control_bits;

	card->current_sector += (len >> 9);
	idx++;
	card->current_idx = idx;
	if (idx >= bio->bi_vcnt)
		card->currentbio = NULL;

#endif
	return 1;
}

static void process_page(unsigned long data)
{
	/* check if any of the requests in the page are DMA_COMPLETE,
	 * and deal with them appropriately.
	 * If we find a descriptor without DMA_COMPLETE in the semaphore, then
	 * dma must have hit an error on that descriptor, so use dma_status
	 * instead and assume that all following descriptors must be re-tried.
	 */
	printk(" in process_page !!!\n");
	printk("%s:%d\n",__FUNCTION__,__LINE__);
#if 0
	struct mm_page *page;
	struct bio *return_bio = NULL;
	struct CardInfo *card = (struct CardInfo *)data;
	unsigned int dma_status = card->dma_status;

	spin_lock_bh(&card->lock);
	if (card->Active < 0)
		goto out_unlock;
	page = &card->mm_pages[card->Active];

	while (page->headcnt < page->cnt) {
		struct bio *bio = page->bio;
		struct mm_dma_desc *desc = &page->desc[page->headcnt];
		int control = le32_to_cpu(desc->sem_control_bits);
		int last = 0;
		int idx;

		if (!(control & DMASCR_DMA_COMPLETE)) {
			control = dma_status;
			last = 1;
		}
		page->headcnt++;
		idx = page->idx;
		page->idx++;
		if (page->idx >= bio->bi_vcnt) {
			page->bio = bio->bi_next;
			if (page->bio)
				page->idx = page->bio->bi_idx;
		}

		pci_unmap_page(card->dev, desc->data_dma_handle,
			       bio_iovec_idx(bio, idx)->bv_len,
				 (control & DMASCR_TRANSFER_READ) ?
				PCI_DMA_TODEVICE : PCI_DMA_FROMDEVICE);
		if (control & DMASCR_HARD_ERROR) {
			/* error */
			clear_bit(BIO_UPTODATE, &bio->bi_flags);
			dev_printk(KERN_WARNING, &card->dev->dev,
				"I/O error on sector %d/%d\n",
				le32_to_cpu(desc->local_addr)>>9,
				le32_to_cpu(desc->transfer_size));
			dump_dmastat(card, control);
		} else if ((bio->bi_rw & REQ_WRITE) &&
			   le32_to_cpu(desc->local_addr) >> 9 ==
				card->init_size) {
			card->init_size += le32_to_cpu(desc->transfer_size) >> 9;
			if (card->init_size >> 1 >= card->mm_size) {
				dev_printk(KERN_INFO, &card->dev->dev,
					"memory now initialised\n");
				set_userbit(card, MEMORY_INITIALIZED, 1);
			}
		}
		if (bio != page->bio) {
			bio->bi_next = return_bio;
			return_bio = bio;
		}

		if (last)
			break;
	}

	if (debug & DEBUG_LED_ON_TRANSFER)
		set_led(card, LED_REMOVE, LED_OFF);

	if (card->check_batteries) {
		card->check_batteries = 0;
		check_batteries(card);
	}
	if (page->headcnt >= page->cnt) {
		reset_page(page);
		card->Active = -1;
		activate(card);
	} else {
		/* haven't finished with this one yet */
		pr_debug("do some more\n");
		mm_start_io(card);
	}
 out_unlock:
	spin_unlock_bh(&card->lock);

	while (return_bio) {
		struct bio *bio = return_bio;

		return_bio = bio->bi_next;
		bio->bi_next = NULL;
		bio_endio(bio, 0);
	}
#endif
}

static int alt_make_request(struct request_queue *q, struct bio *bio)
{
	printk("%s:%d\n",__FUNCTION__,__LINE__);
	struct CardInfo *card = q->queuedata;
	printk("alt_make_request %llu %u\n",
		 (unsigned long long)bio->bi_sector, bio->bi_size);

	spin_lock_irq(&card->lock);
	*card->biotail = bio;
	bio->bi_next = NULL;
	card->biotail = &bio->bi_next;
	spin_unlock_irq(&card->lock);

	return 0;
}

static irqreturn_t alt_interrupt(int irq, void *__card)
{
	struct CardInfo *card = (struct CardInfo *) __card;

	printk(" ---- in alt_interrupt \n");
	return IRQ_HANDLED;

}
/*
 * Note no locks taken out here.  In a worst case scenario, we could drop
 * a chunk of system memory.  But that should never happen, since validation
 * happens at open or mount time, when locks are held.
 *
 *	That's crap, since doing that while some partitions are opened
 * or mounted will give you really nasty results.
 */
static int alt_revalidate(struct gendisk *disk)
{
	struct CardInfo *card = disk->private_data;
	set_capacity(disk, card->mm_size << 1);
	return 0;
}

static int alt_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct CardInfo *card = bdev->bd_disk->private_data;
	int size = card->mm_size * (1024 / MM_HARDSECT);

	/*
	 * get geometry: we have to fake one...  trim the size to a
	 * multiple of 2048 (1M): tell we have 32 sectors, 64 heads,
	 * whatever cylinders.
	 */
	geo->heads     = 64;
	geo->sectors   = 32;
	geo->cylinders = size / (geo->heads * geo->sectors);
	return 0;
}

static const struct block_device_operations alt_fops = {
	.owner		= THIS_MODULE,
	.getgeo		= alt_getgeo,
	.revalidate_disk = alt_revalidate,
};

static int __devinit alt_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret = -ENODEV, err;
	unsigned char	mem_present;
	unsigned char	batt_status;
	unsigned int	saved_bar, data;
	unsigned long	csr_base;
	unsigned long	csr_len;
	int		magic_number;
	static int	printed_version = 0;

	if (!printed_version++)
		printk(KERN_INFO ALT_DRIVER_VERSION " : " ALT_DRIVER_DESC "\n");

	ret = pci_enable_device(dev);
	if (ret)
	{
		printk(" failed to enable pci device \n");
		return ret;
	}

//	pci_write_config_byte(dev, PCI_LATENCY_TIMER, 0xF8);
	pci_set_master(dev);

	cardinfo.dev         = dev;

	csr_base = pci_resource_start(dev, CSR_BAR_NR);
	csr_len  = pci_resource_len(dev, CSR_BAR_NR);
	if (!csr_base || !csr_len)
	{
		printk(" failed to get csr range \n");
		return -ENODEV;
	}

	dev_printk(KERN_INFO, &dev->dev, "altera sgdma memory found\n");
	printk(KERN_INFO "altera sgdma memory found\n");

	if (pci_set_dma_mask(dev, DMA_BIT_MASK(64)) &&
	    pci_set_dma_mask(dev, DMA_BIT_MASK(32))) {
		dev_printk(KERN_WARNING, &dev->dev, "NO suitable DMA found\n");
		return  -ENOMEM;
	}

	ret = pci_request_regions(dev, ALT_DRIVER_NAME);
	if (ret) {
		dev_printk(KERN_ERR, &cardinfo.dev->dev,
			"Unable to request memory region\n");
		goto failed_req_csr;
	}

	cardinfo.csr_remap = ioremap_nocache(csr_base, csr_len);
	if (!cardinfo.csr_remap) {
		dev_printk(KERN_ERR, &cardinfo.dev->dev,
			"Unable to remap memory region\n");
		ret = -ENOMEM;

		goto failed_remap_csr;
	}
	sgdma_reset_dispatcher( cardinfo.csr_remap );
	while( sgdma_read_resetting(cardinfo.csr_remap ) )
	{
		printk(" sgdma read resetting !!! \n");
	}

	dev_printk(KERN_INFO, &cardinfo.dev->dev,
		"CSR 0x%08lx -> 0x%p (0x%lx)\n",
	       csr_base, cardinfo.csr_remap, csr_len);


/*
	cardinfo.mm_pages[0].desc = pci_alloc_consistent(cardinfo.dev,
						PAGE_SIZE * 2,
						&(cardinfo.mm_pages[0].page_dma));
	cardinfo.mm_pages[1].desc = pci_alloc_consistent(cardinfo.dev,
						PAGE_SIZE * 2,
						&(cardinfo.mm_pages[1].page_dma));
	if (cardinfo.mm_pages[0].desc == NULL ||
	    cardinfo.mm_pages[1].desc == NULL) {
		dev_printk(KERN_ERR, &(cardinfo.dev->dev), "alloc failed\n");
		goto failed_alloc;
	}
	reset_page(&cardinfo.mm_pages[0]);
	reset_page(&cardinfo.mm_pages[1]);
*/
	cardinfo.Ready = 0;	/* page 0 is ready */
	cardinfo.Active = -1;	/* no page is active */
	cardinfo.bio = NULL;
	cardinfo.biotail = &cardinfo.bio;

	cardinfo.queue = blk_alloc_queue(GFP_KERNEL);
	if (!cardinfo.queue)
	{
		
		printk(KERN_ERR "Unable to allocate q\n");
		goto failed_alloc;
	}

	blk_queue_make_request(cardinfo.queue, alt_make_request);
	cardinfo.queue->queue_lock = &cardinfo.lock;
	cardinfo.queue->queuedata = &cardinfo;

	tasklet_init(&cardinfo.tasklet, process_page, (unsigned long)(&cardinfo));

	cardinfo.mm_size = 1024 * 128;

/*
	if (request_irq(dev->irq, alt_interrupt, IRQF_SHARED, ALT_DRIVER_NAME,
			&cardinfo)) {
		dev_printk(KERN_ERR, &cardinfo.dev->dev, "Unable to allocate IRQ\n");
		printk(KERN_ERR "Unable to allocate IRQ\n");
		ret = -ENODEV;
		goto failed_req_irq;
	}
*/

	dev_printk(KERN_INFO, &cardinfo.dev->dev,
		"Window size %d bytes, IRQ %d\n", data, dev->irq);

	spin_lock_init(&cardinfo.lock);

	pci_set_drvdata(dev, &cardinfo);



	// user layer intf prepare
	err = major_nr = register_blkdev(0, ALT_DRIVER_NAME);
	if (err < 0) {
		printk(" failed to request block\n");
		goto failed_req_blk;
	}
	cardinfo.mm_size = 1024 * 128;

	//alt_gendisk = alloc_disk( MAX_PARTITION_NR );
	alt_gendisk = alloc_disk( 1 );
	if ( !alt_gendisk )
	{
		printk(" failed to alloc_disk \n");
		goto failed_req_irq;
	}

	//sprintf(alt_gendisk->disk_name, "sgdma%d", 0);
	sprintf(alt_gendisk->disk_name, "sgdma0");
	spin_lock_init(&cardinfo.lock);
	alt_gendisk->major = major_nr;
	//alt_gendisk->first_minor  = MAX_PARTITION_NR;
	alt_gendisk->first_minor  = 0;
	alt_gendisk->fops = &alt_fops;
	alt_gendisk->private_data = &cardinfo;
	alt_gendisk->queue = cardinfo.queue;
	set_capacity(alt_gendisk, cardinfo.mm_size << 1);
	add_disk(alt_gendisk);


	printk(" the end of probe\n");
	return 0;

 failed_req_irq:
 failed_alloc:
/*
	if (cardinfo.mm_pages[0].desc)
		pci_free_consistent(cardinfo.dev, PAGE_SIZE*2,
				    cardinfo.mm_pages[0].desc,
				    cardinfo.mm_pages[0].page_dma);
	if (cardinfo.mm_pages[1].desc)
		pci_free_consistent(cardinfo.dev, PAGE_SIZE*2,
				    cardinfo.mm_pages[1].desc,
				    cardinfo.mm_pages[1].page_dma);
*/
	put_disk(alt_gendisk);
 failed_req_blk:
	unregister_blkdev(major_nr, ALT_DRIVER_NAME);
 failed_magic:
	iounmap(cardinfo.csr_remap);
 failed_remap_csr:
	pci_release_regions(dev);
 failed_req_csr:

	return ret;
}

static void alt_pci_remove(struct pci_dev *dev)
{
	struct CardInfo *card = pci_get_drvdata(dev);
	printk(" in pci_remove %d\n",__LINE__);

	tasklet_kill(&card->tasklet);
//	free_irq(dev->irq, card);
	iounmap(card->csr_remap);

	printk(" in pci_remove %d\n",__LINE__);
/*
	if (card->mm_pages[0].desc)
		pci_free_consistent(card->dev, PAGE_SIZE*2,
				    card->mm_pages[0].desc,
				    card->mm_pages[0].page_dma);
	if (card->mm_pages[1].desc)
		pci_free_consistent(card->dev, PAGE_SIZE*2,
				    card->mm_pages[1].desc,
				    card->mm_pages[1].page_dma);
*/
	printk(" in pci_remove %d\n",__LINE__);

	blk_cleanup_queue(card->queue);

	pci_release_regions(dev);
	pci_disable_device(dev);

	del_gendisk(alt_gendisk);
	put_disk(alt_gendisk);

	unregister_blkdev(major_nr, ALT_DRIVER_NAME);
	printk(" in pci_remove %d\n",__LINE__);
}

static const struct pci_device_id alt_pci_ids[] = {
  { PCI_DEVICE(0x1172, 0xe001), },
  { 0, }

};

MODULE_DEVICE_TABLE(pci, alt_pci_ids);

static struct pci_driver sgdma_pci_driver = {
	.name		= ALT_DRIVER_NAME,
	.id_table	= alt_pci_ids,
	.probe		= alt_pci_probe,
	.remove		= alt_pci_remove,
};

static int __init sgdma_init(void)
{
	int retval;

	retval = pci_register_driver(&sgdma_pci_driver);
	if (retval)
		return -ENOMEM;

	return 0;

out:
	pci_unregister_driver(&sgdma_pci_driver);

	return -ENOMEM;
}

static void __exit sgdma_cleanup(void)
{
	pci_unregister_driver(&sgdma_pci_driver); 
}

module_init(sgdma_init);
module_exit(sgdma_cleanup);

MODULE_AUTHOR(ALT_DRIVER_AUTHOR);
MODULE_DESCRIPTION(ALT_DRIVER_DESC);
MODULE_LICENSE("GPL");
