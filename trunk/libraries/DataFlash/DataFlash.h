/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

// arduino mega SPI pins
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	#define DF_DATAOUT 51        // MOSI
	#define DF_DATAIN  50        // MISO 
	#define DF_SPICLOCK  52      // SCK
	#define DF_SLAVESELECT 53    // SS     (PB0)
    #define DF_RESET 31          // RESET  (PC6)
#else  // normal arduino SPI pins...
	#define DF_DATAOUT 11     //MOSI
	#define DF_DATAIN  12     //MISO 
	#define DF_SPICLOCK  13   //SCK
	#define DF_SLAVESELECT 10 //SS
#endif

// AT45DB161D Commands (from Datasheet)
#define DF_TRANSFER_PAGE_TO_BUFFER_1   0x53
#define DF_TRANSFER_PAGE_TO_BUFFER_2   0x55
#define DF_STATUS_REGISTER_READ   0xD7
#define DF_READ_MANUFACTURER_AND_DEVICE_ID   0x9F
#define DF_PAGE_READ   0xD2
#define DF_BUFFER_1_READ   0xD4
#define DF_BUFFER_2_READ   0xD6
#define DF_BUFFER_1_WRITE   0x84
#define DF_BUFFER_2_WRITE   0x87
#define DF_BUFFER_1_TO_PAGE_WITH_ERASE   0x83
#define DF_BUFFER_2_TO_PAGE_WITH_ERASE   0x86
#define DF_PAGE_ERASE   0x81
#define DF_BLOCK_ERASE   0x50
#define DF_SECTOR_ERASE   0x7C
#define DF_CHIP_ERASE_0   0xC7
#define DF_CHIP_ERASE_1   0x94
#define DF_CHIP_ERASE_2   0x80
#define DF_CHIP_ERASE_3   0x9A

class DataFlash_Class
{
  private:
	// DataFlash Log variables...	
	unsigned char df_BufferNum;
	unsigned char df_Read_BufferNum;
	uint16_t df_BufferIdx;
	uint16_t df_Read_BufferIdx;
	uint16_t df_PageAdr;
	uint16_t df_Read_PageAdr;
	unsigned char df_Read_END;
	unsigned char df_Stop_Write;
	//Methods
	unsigned char BufferRead (unsigned char BufferNum, uint16_t IntPageAdr);
	void BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data);
	void BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait);
	void PageToBuffer(unsigned char BufferNum, uint16_t PageAdr);
	void WaitReady();
	unsigned char ReadStatusReg();
	unsigned char ReadStatus();
	uint16_t PageSize();

  public:
	unsigned char df_manufacturer;
	unsigned char df_device_0;
	unsigned char df_device_1;
	uint16_t df_PageSize;

	DataFlash_Class(); // Constructor
	void Init();
	void ReadManufacturerID();
	int16_t GetPage();
	int16_t GetWritePage();
	void PageErase (uint16_t PageAdr);
	void ChipErase ();
	// Write methods
	void StartWrite(int16_t PageAdr);
	void FinishWrite();
	void WriteByte(unsigned char data);
	void WriteInt(int16_t data);
	void WriteLong(int32_t data);

	// Read methods
	void StartRead(int16_t PageAdr);
	unsigned char ReadByte();
	int16_t ReadInt();
	int32_t ReadLong();
};

extern DataFlash_Class DataFlash;

#endif
