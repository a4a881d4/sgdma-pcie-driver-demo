#include <time.h>
#include "wdc_defs.h"
#include "wdc_lib.h"
#include "stdio.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define NUMBER_OF_DESCRPT 2

unsigned int OnRCSlaveRead(WDC_DEVICE_HANDLE hDev, int bar, unsigned int addr){
	unsigned int u32Data=0;
	WDC_ReadAddr32(hDev, bar, addr, &u32Data);
	return u32Data;
}

void OnRCSlaveWrite(WDC_DEVICE_HANDLE hDev, int bar, unsigned int addr, unsigned int wdata){
   WDC_WriteAddr8(hDev,bar,addr   , wdata    & 0xFF);
   WDC_WriteAddr8(hDev,bar,addr+1 , (wdata>>8)  & 0xFF);
   WDC_WriteAddr8(hDev,bar,addr+2 , (wdata>>16) & 0xFF);
   WDC_WriteAddr8(hDev,bar,addr+3 , (wdata>>24) & 0xFF);
}

// PC -> FPGA
void DMA_Read(WDC_DEVICE_HANDLE hDev, unsigned int target_addr, int length){
	int l;
	clock_t start,end;
	double accum_time = 0;
	LARGE_INTEGER IFreq, IStart, IEnd;
	int test = 0;
	int iteration = 100;

	int i;

	unsigned int address_offset=0;

	int data_pattern = 1;
	int data_check_option = 1;
	



	DWORD dwStatus;
	HANDLE hWD;
	WD_DMA dma_buff;

	// getting low level driver handle
	hWD = WD_Open();
	if (hWD == INVALID_HANDLE_VALUE)
		printf("DMA read : Cannot open WinDriver device");


	// This cardReg is required to use low level driver
	WD_CARD_REGISTER cardReg;
	BZERO(cardReg);
	cardReg.Card.dwItems = 1;
	cardReg.Card.Item[0].item = ITEM_IO;
	cardReg.Card.Item[0].fNotSharable = TRUE;
	cardReg.Card.Item[0].I.IO.dwAddr = 0x378;
	cardReg.Card.Item[0].I.IO.dwBytes = 8;
	WD_CardRegister(hWD, &cardReg);
	if (cardReg.hCard == 0){
		printf("DMA read : Failed locking device");
		return;
	}


	// allocating the user memory area
	BZERO(dma_buff);
	dma_buff.dwBytes = length + 32; // additional 32byte will be used to detect the end of process.
	// Set contiguous Buffer
	dma_buff.dwOptions = DMA_KERNEL_BUFFER_ALLOC | DMA_KBUF_BELOW_16M;
	dma_buff.hCard = cardReg.hCard;

	dwStatus = WD_DMALock(hWD, &dma_buff);
	if(dwStatus){
		printf("DMA read : Failed to allocate memory");
		return;
	}

	// generating random data
	DWORD random_data;
	unsigned current_time = (unsigned)time(NULL);
	srand(current_time);
	random_data=rand();

	DWORD *pSBuf; // Soft Buffer
	// This is the data send to FPGA
	pSBuf= (DWORD *) dma_buff.pUserAddr;
		for (i = 0; i < length / 4 ; i++) {
			if(data_pattern == 1){
				random_data=rand();
				double tmp = (double)random_data / 17737;
				DWORD t = tmp * 0xFFFFFFFF;
				pSBuf[i] = t;
			}else
				pSBuf[i] = i;
	}


	// clearing out the location for DMA end detection
	for (i = length / 4; i < (length+32) / 4 ; i++) {
			pSBuf[i] = 0;
	}

	// Set up the variable for DMA end detection at here.
	DWORD expected_data = pSBuf[32/4 - 1];
	DWORD read_data = pSBuf[(length + 32)/4 - 1]; // Not yet transmitted, so it should get 0 at here.


	// trying and check the address translation path through
	OnRCSlaveWrite(hDev, 2, 0x1000, 0xFFFFFFFC); //0x1000 is the offset for translation register

	// reading out the resulted mask of path through
	UINT32 a2p_mask = OnRCSlaveRead(hDev, 2, 0x1000);//0x1000 is the offset for translation register
			
	// program address translation table
	// PCIe core limits the data length to be 1MByte, so it only needs 20bits of address.
	OnRCSlaveWrite(hDev, 2, 0x1000, dma_buff.Page[0].pPhysicalAddr & a2p_mask); //setting lower address
	OnRCSlaveWrite(hDev, 2, 0x1004, 0x0); // setting upper address   limited at hardIP for now.



/////////////////////////////////////////////////////////////////////////////////////////////////
//	sgdma_standard_descriptor a_descriptor[NUMBER_OF_DESCRPT];
	unsigned long length_mod[NUMBER_OF_DESCRPT];
	unsigned long read_address[NUMBER_OF_DESCRPT];
	unsigned long write_address[NUMBER_OF_DESCRPT];


	// clear the DMA contoller
	int res = OnRCSlaveRead(hDev, 2, 0x06000000);
	OnRCSlaveWrite(hDev, 2, 0x06000000, 0x200);
	res = OnRCSlaveRead(hDev, 2, 0x06000000);
	OnRCSlaveWrite(hDev, 2, 0x06000004, 0x02);  // issue reset dispatcher
	OnRCSlaveWrite(hDev, 2, 0x06000000, 0x0000); //clear all the status
	OnRCSlaveWrite(hDev, 2, 0x06000004, 0x10); // set IRQ enable
	res = OnRCSlaveRead(hDev, 2, 0x06000000);


	//enable_global_interrupt_mask (alt_u32 csr_base)
	int irq_mask = OnRCSlaveRead(hDev, 2, 0x06000004);
	irq_mask |= 0x10;
	OnRCSlaveWrite(hDev, 2, 0x06000004, irq_mask); //setting the IRQ enable flag at here 
	// clear the status
	int status = OnRCSlaveRead(hDev, 2, 0x06000000);
	OnRCSlaveWrite(hDev, 2, 0x06000000, 0x0); //clear all the status
	status = OnRCSlaveRead(hDev, 2, 0x06000000);


    // generate buffer base addresses and lengths
    for(i = 0; i < NUMBER_OF_DESCRPT-1; i++){ // the last descriptor will be used to detect the end of process
      read_address[i] = (DWORD) dma_buff.Page[0].pPhysicalAddr & ~a2p_mask;
      write_address[i] = target_addr; //target_addr is the SOPC address for DDR/OCM
      length_mod[i] = length;
    }


	//// This is generating descriptor for DMA end detection
    read_address[NUMBER_OF_DESCRPT-1] = target_addr; // reading data from the very begining of the target memory 32byte of data
    write_address[NUMBER_OF_DESCRPT-1] = ((DWORD) dma_buff.Page[0].pPhysicalAddr & ~a2p_mask) + length; // write it right after the end of data
    length_mod[NUMBER_OF_DESCRPT-1] = 32; // has to be aligned length to be burst


    // start assembring descriptors
	unsigned long control_bits = 0;
	int failed_transaction = 0;

	// start monitoring the time
	QueryPerformanceFrequency(&IFreq); 
	QueryPerformanceCounter(&IStart);

    // start writing descriptors to the SGDMA
    for(i = 0; i < NUMBER_OF_DESCRPT; i++){
		while ((OnRCSlaveRead(hDev, 2, 0x06000000) & 0x04) != 0) {}  // spin until there is room for another descriptor to be written to the SGDMA
		control_bits = (i == (NUMBER_OF_DESCRPT-1))? (1<<14) : (1 << 24);  // 14bit is the IRQ, 24bit is the early done bit

		OnRCSlaveWrite(hDev, 2, 0x06000020, read_address[i]);
		OnRCSlaveWrite(hDev, 2, 0x06000024, write_address[i]);
		OnRCSlaveWrite(hDev, 2, 0x06000028, length_mod[i]);
		OnRCSlaveWrite(hDev, 2, 0x0600002C, control_bits | (1<<31));
    }

	// checking the end of DMA transaction.
	while(expected_data != read_data){
		read_data = pSBuf[(length + 32)/4 - 1];
		QueryPerformanceCounter(&IEnd);
		int t = (IEnd.QuadPart-IStart.QuadPart)/IFreq.QuadPart;
		if(t > 1){
			failed_transaction = 1;
			break;
		}
	}

	// This is just a trick when the correct data was not came back as end of DMA data.
	if(failed_transaction == 1){
		res = OnRCSlaveRead(hDev, 2, 0x06000000);
		if(res  == 0x202)  { // At least DMA has finished
			failed_transaction = 0;
		}
	}

	// data verification
	if(data_check_option == 1){
		unsigned int a, b;
		for(int i=0; i<length/4; i++){
			a = OnRCSlaveRead(hDev, 0, target_addr + i*4); // it is slow check, but works.
			b = pSBuf[i];
			if(b != a){
				failed_transaction = 1;
				printf("DMA read : Data verify failed");
				break;
			}
		}
	}



	double MBpss = 0;

	if(failed_transaction == 0){
		// clear the DMA
		OnRCSlaveWrite(hDev, 2, 0x06000000, 0x200);
		OnRCSlaveWrite(hDev, 2, 0x06000000, 0x200);

		// calculate the performance
		accum_time += double(IEnd.QuadPart-IStart.QuadPart)/IFreq.QuadPart;
		MBpss = (length / accum_time) / 1000000;
		printf("DMA read :  %f MByte/sec\n", MBpss);

		// close the driver and used memory
	   WD_DMAUnlock(hWD, &dma_buff);
	   WD_CardUnregister(hWD, &cardReg);
	   WD_Close(hWD);
	}else{
		// need to reset the system...get stuck at somewhere
		OnRCSlaveWrite(hDev, 2, 0x06000000, 0x200);
		res = OnRCSlaveRead(hDev, 2, 0x06000000);

		OnRCSlaveWrite(hDev, 2, 0x06000004, 0x02);  // issue reset dispatcher
		OnRCSlaveWrite(hDev, 2, 0x06000000, 0x0000); //clear all the status
		OnRCSlaveWrite(hDev, 2, 0x06000004, 0x10); // set IRQ enable

		res = OnRCSlaveRead(hDev, 2, 0x06000000);

	   WD_DMAUnlock(hWD, &dma_buff);
	   WD_CardUnregister(hWD, &cardReg);
	   WD_Close(hWD);

		printf("DMA read : timed out");
		return;
	}
}



// FPGA -> PC
void DMA_Write(WDC_DEVICE_HANDLE hDev, unsigned int target_addr, int length){
	int l;
	clock_t start,end;
	double accum_time = 0;
	LARGE_INTEGER IFreq, IStart, IEnd;
	int test = 0;
	int iteration = 100;

	int i;
	int data_pattern = 1;
	int data_check_option = 1;

	DWORD dwStatus;


	HANDLE hWD;
	WD_DMA dma_buff;

	hWD = WD_Open();
	if (hWD == INVALID_HANDLE_VALUE)
	{
		printf("DMA write : Cannot open WinDriver device");
		return;
	}



	WD_CARD_REGISTER cardReg;
	BZERO(cardReg);
	cardReg.Card.dwItems = 1;
	cardReg.Card.Item[0].item = ITEM_IO;
	cardReg.Card.Item[0].fNotSharable = TRUE;
	cardReg.Card.Item[0].I.IO.dwAddr = 0x378;
	cardReg.Card.Item[0].I.IO.dwBytes = 8;
	WD_CardRegister(hWD, &cardReg);
	if (cardReg.hCard == 0){
		printf("DMA write : Failed locking device");
		return;
	}



	BZERO(dma_buff);
	dma_buff.dwBytes = length + 32;
	// Set contiguous Buffer
	dma_buff.dwOptions = DMA_KERNEL_BUFFER_ALLOC | DMA_KBUF_BELOW_16M;
	dma_buff.hCard = cardReg.hCard;

	dwStatus = WD_DMALock(hWD, &dma_buff);
	if(dwStatus){
		printf("DMA write : Failed allocating memory");
		return;
	}

	DWORD random_data;
	unsigned current_time = (unsigned)time(NULL);
	srand(current_time);
	random_data=rand();

	DWORD *pSBuf; // Soft Buffer
	DWORD *pSBuf_org; // original data that will be written into the target memory
	// Setting data on the target
	pSBuf= (DWORD *) dma_buff.pUserAddr;
	pSBuf_org = (DWORD *) malloc(sizeof(DWORD)*length);
	for (i = 0; i < length / 4 ; i++) {
		if(data_pattern == 1){
			random_data=rand();
			double tmp = (double)random_data / 17737;
			DWORD t = tmp * 0xFFFFFFFF;
			pSBuf_org[i] = t;
			OnRCSlaveWrite(hDev, 0, target_addr + i*4, t); // setting data to the target
		}else{
			OnRCSlaveWrite(hDev, 0, target_addr + i*4, i); // setting data to the target
			pSBuf_org[i] = i;
		}
	}

	// Set up the variable for DMA end detection at here.
	DWORD expected_data = pSBuf_org[32/4 - 1];
	DWORD read_data = 0;

	// clearing out the location for DMA end detection
	for (i = length / 4; i < (length+32) / 4 ; i++) {
			pSBuf[i] = 0;
	}


	// trying and check the address translation path through
	OnRCSlaveWrite(hDev, 2, 0x1000, 0xFFFFFFFC); //0x1000 is the offset for translation register

	// reading out the resulted mask of path through
	UINT32 a2p_mask = OnRCSlaveRead(hDev, 2, 0x1000);//0x1000 is the offset for translation register
			
	// program address translation table
	OnRCSlaveWrite(hDev, 2, 0x1000, dma_buff.Page[0].pPhysicalAddr & a2p_mask); //setting lower address
	OnRCSlaveWrite(hDev, 2, 0x1004, 0x0); // setting upper address


/////////////////////////////////////////////////////////////////////////////////////////////////
	unsigned long length_mod[NUMBER_OF_DESCRPT];
	unsigned long read_address[NUMBER_OF_DESCRPT];
	unsigned long write_address[NUMBER_OF_DESCRPT];

	// clear the DMA controller and status
	int res = OnRCSlaveRead(hDev, 2, 0x06000000);
	OnRCSlaveWrite(hDev, 2, 0x06000004, 0x02);  // issue reset dispatcher
	OnRCSlaveWrite(hDev, 2, 0x06000000, 0x0000); //clear all the status
	OnRCSlaveWrite(hDev, 2, 0x06000004, 0x10); // set IRQ enable

	res = OnRCSlaveRead(hDev, 2, 0x06000000);


	//enable_global_interrupt_mask (alt_u32 csr_base)
	int irq_mask = OnRCSlaveRead(hDev, 2, 0x06000004);
	irq_mask |= 0x10;
	OnRCSlaveWrite(hDev, 2, 0x06000004, irq_mask); //setting the IRQ enable flag at here 
	// clear the status
	int status = OnRCSlaveRead(hDev, 2, 0x06000000);
	OnRCSlaveWrite(hDev, 2, 0x06000000, 0x0); //clear all the status
	status = OnRCSlaveRead(hDev, 2, 0x06000000);


    // generate buffer base addresses and lengths
    for(i = 0; i < NUMBER_OF_DESCRPT-1; i++)
    {
      read_address[i] = target_addr; //target_addr is the SOPC address for DDR/OCM
	  write_address[i] = (DWORD) dma_buff.Page[0].pPhysicalAddr & ~a2p_mask;
      length_mod[i] = length;
    }

	//// This is generating descriptor for DMA end detection
    read_address[NUMBER_OF_DESCRPT-1] = target_addr;
    write_address[NUMBER_OF_DESCRPT-1] = ((DWORD) dma_buff.Page[0].pPhysicalAddr & ~a2p_mask) + length; // write it right after the end of data
    length_mod[NUMBER_OF_DESCRPT-1] = 32; // has to be aligned length to be burst


    // start assembring descriptors
	unsigned long control_bits = 0;
	res = OnRCSlaveRead(hDev, 2, 0x06000000);

	int failed_transaction = 0;
	// start monitoring the time
	QueryPerformanceFrequency(&IFreq); 
	QueryPerformanceCounter(&IStart);

    // start writing descriptors to the SGDMA
    for(i = 0; i < NUMBER_OF_DESCRPT; i++){
		while ((OnRCSlaveRead(hDev, 2, 0x06000000) & 0x04) != 0) {}  // spin until there is room for another descriptor to be written to the SGDMA
		control_bits = (i == (NUMBER_OF_DESCRPT-1))? (1<<14) : (1<<24);  // go bit is handled 'construct_standard_mm_to_mm_descriptor'

		OnRCSlaveWrite(hDev, 2, 0x06000020, read_address[i]);
		OnRCSlaveWrite(hDev, 2, 0x06000024, write_address[i]);
		OnRCSlaveWrite(hDev, 2, 0x06000028, length_mod[i]);
		OnRCSlaveWrite(hDev, 2, 0x0600002C, control_bits | (1<<31));
    }

	// Detecting the end of DMA trasaction
	while(expected_data != read_data){
		read_data = pSBuf[(length + 32)/4 - 1];
		QueryPerformanceCounter(&IEnd);
		int t = (IEnd.QuadPart-IStart.QuadPart)/IFreq.QuadPart;
		if(t > 1){
			failed_transaction = 1;
			break;
		}
	}

	// This is just a trick when the correct data was not came back as end of DMA data.
	if(failed_transaction == 1){
		res = OnRCSlaveRead(hDev, 2, 0x06000000);
		if(res  == 0x202)  { // At least DMA has finished
			failed_transaction = 0;
		}
	}

	// data verification
	if(data_check_option == 1){
		for(int i=0; i<length/4; i++){
			if(pSBuf[i] != pSBuf_org[i]){
				failed_transaction = 1;
				printf("DMA write : Data verify failed");
				break;
			}
		}
	}


	double MBpss = 0;
	if(failed_transaction == 0){
		OnRCSlaveWrite(hDev, 2, 0x06000000, 0x200);
		OnRCSlaveWrite(hDev, 2, 0x06000000, 0x200);
		accum_time += double(IEnd.QuadPart-IStart.QuadPart)/IFreq.QuadPart;
		MBpss = (length / accum_time) / 1000000;
		printf("DMA write : %f MByte/sec\n", MBpss);

	   WD_DMAUnlock(hWD, &dma_buff);
	   WD_CardUnregister(hWD, &cardReg);
	   WD_Close(hWD);
	   free(pSBuf_org);

	}else{
		// clear the system
		OnRCSlaveWrite(hDev, 2, 0x06000004, 0x02);  // issue reset dispatcher
		OnRCSlaveWrite(hDev, 2, 0x06000000, 0x0000); //clear all the status
		OnRCSlaveWrite(hDev, 2, 0x06000004, 0x10); // set IRQ enable
		res = OnRCSlaveRead(hDev, 2, 0x06000000);

	   WD_DMAUnlock(hWD, &dma_buff);
	   WD_CardUnregister(hWD, &cardReg);
	   WD_Close(hWD);
	   free(pSBuf_org);

		printf("DMA write : timed out");
		return;
	}
}

