
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"

#include "nrf_drv_spi.h"
#include "w25q128.h"

//SPI驱动程序实例ID,ID和外设编号对应，0:SPI0  1:SPI1 2:SPI2
#define SPI_INSTANCE  0 
//定义名称为spi的SPI驱动程序实例
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  
//SPI传输完成标志
static volatile bool spi_xfer_done;  
//SPI发送缓存数组，使用EasyDMA时一定要定义为static类型
static uint8_t    spi_tx_buf[6];  
//SPI接收缓存数组，使用EasyDMA时一定要定义为static类型
static uint8_t    spi_rx_buf[6];  


/*****************************************************************************
** 描  述：写入一个字节
** 参  数：Dat：待写入的数据
** 返回值：无
******************************************************************************/
void Spi_WriteOneByte(uint8_t Dat)
{   
	  spi_tx_buf[0] = Dat;
	  spi_xfer_done = false;
	  SPIFlash_CS_LOW;
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 1, spi_rx_buf, 0));
    while(!spi_xfer_done);
	  SPIFlash_CS_HIGH;
}
/*****************************************************************************
** 描  述：写使能
** 参  数：无
** 返回值：无
******************************************************************************/
static void SpiFlash_Write_Enable(void)
{
  Spi_WriteOneByte(SPIFlash_WriteEnable);
}
/*****************************************************************************
** 描  述：读取W25Q128芯片ID
** 参  数：无
** 返回值：16位ID，W25Q128芯片ID为：0xEF17
******************************************************************************/
uint16_t SpiFlash_ReadID(void)
{
  uint16_t dat = 0;
	//准备数据
	spi_tx_buf[0] = SPIFlash_ReadID;
	spi_tx_buf[1] = 0x00;
	spi_tx_buf[2] = 0x00;
	spi_tx_buf[3] = 0x00;
	spi_tx_buf[4] = 0xFF;
	spi_tx_buf[5] = 0xFF;
	//传输完成标志设置为false
	spi_xfer_done = false;
	//拉低CS，使能W25Q128FV
	SPIFlash_CS_LOW;
	//启动数据传输
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 6, spi_rx_buf, 6));
	//等待SPI传输完成
  while(!spi_xfer_done);
  //拉高CS，释放W25Q128FV
	SPIFlash_CS_HIGH;
	//接收数组最后两个字节才是读取的ID
	dat|=spi_rx_buf[4]<<8;  
	dat|=spi_rx_buf[5];	

  return dat;		
}
/*****************************************************************************
** 描  述：读取W25Q128状态寄存器
** 参  数：无
** 返回值：
******************************************************************************/
static uint8_t SpiFlash_ReadSR(void)
{
	  spi_tx_buf[0] = SPIFlash_ReadStatusReg;
	  spi_tx_buf[1] = 0x00;
    //传输完成标志设置为false
	  spi_xfer_done = false;
	  SPIFlash_CS_LOW;
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 2, spi_rx_buf, 2));
    while(!spi_xfer_done);
    SPIFlash_CS_HIGH;
    return spi_rx_buf[1];
}
//等待W25Q128就绪
void SpiFlash_Wait_Busy(void)   
{   
	while((SpiFlash_ReadSR()&0x01)==0x01);  		// 等待BUSY位清空
} 
/*****************************************************************************
** 描  述：擦除扇区，W25Q128FVSIG最小的擦除单位是扇区
** 参  数：[in]SecAddr：扇区地址
** 返回值：无
******************************************************************************/
void SPIFlash_Erase_Sector(uint32_t SecAddr)
{
		//发送写使能命令
    SpiFlash_Write_Enable();
		
		//扇区擦除命令
		spi_tx_buf[0] = SPIFlash_SecErase;		
		//24位地址
    spi_tx_buf[1] = (uint8_t)((SecAddr&0x00ff0000)>>16);
    spi_tx_buf[2] = (uint8_t)((SecAddr&0x0000ff00)>>8);
    spi_tx_buf[3] = (uint8_t)SecAddr;
    //传输完成标志设置为false
    spi_xfer_done = false;
	  //拉低CS，使能W25Q128FV
	  SPIFlash_CS_LOW;
	  //启动数据传输：发送长度4个字节，读取长度0字节
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, SPIFLASH_CMD_LENGTH, spi_rx_buf, 0));
	  //等待SPI传输完成
    while(!spi_xfer_done);
	  //拉高CS，释放W25Q128FV
    SPIFlash_CS_HIGH;
	  //等待W25Q128FV完成操作
    SpiFlash_Wait_Busy();
}
/*****************************************************************************
** 描  述：全片擦除W25Q128FV，全片擦除所需的时间典型值为：40秒
** 参  数：无
** 返回值：无
******************************************************************************/
void SPIFlash_Erase_Chip(void)
{
		//发送写使能命令
    SpiFlash_Write_Enable();
		//全片擦除命令
		spi_tx_buf[0] = SPIFlash_ChipErase;
	  //传输完成标志设置为false
    spi_xfer_done = false;
	  //拉低CS，使能W25Q128FV
	  SPIFlash_CS_LOW;
	  //启动数据传输：发送长度1个字节，读取长度0字节
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 1, spi_rx_buf, 0));
    while(!spi_xfer_done);
	  //拉高CS，释放W25Q128FV
	  SPIFlash_CS_HIGH;
    //等待W25Q128FV完成操作
    SpiFlash_Wait_Busy();
}
/*****************************************************************************
** 描  述：向指定的地址写入数据,最大写入的长度不能超过该地址所处页面的剩余空间
**         *pBuffer:指向待写入的数据缓存
**         WriteAddr:写入的起始地址
**         WriteBytesNum:写入的字节数，一次最多256个字节
** 返回值：RET_SUCCESS
******************************************************************************/
uint8_t SpiFlash_Write_Page(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t size)
{
	  //检查写入的数据长度是否合法，写入长度不能超过页面的大小
	  if (size > (SPIFlash_PAGE_SIZE - (WriteAddr%SPIFlash_PAGE_SIZE)))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
	  if (size == 0) return NRF_ERROR_INVALID_LENGTH;
	
    //发送写使能命令
    SpiFlash_Write_Enable();
    //页编程命令
		spi_tx_buf[0] = SPIFlash_PageProgram;
		//24位地址，高地址在前
    spi_tx_buf[1] = (uint8_t)((WriteAddr&0x00ff0000)>>16);
    spi_tx_buf[2] = (uint8_t)((WriteAddr&0x0000ff00)>>8);
    spi_tx_buf[3] = (uint8_t)WriteAddr;
	  spi_tx_buf[4] = *pBuffer;
	  
	
	  //拉低CS，使能W25Q128FV
	  SPIFlash_CS_LOW;
	  //传输完成标志设置为false
		spi_xfer_done = false;
	  //启动数据传输
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, SPIFLASH_CMD_LENGTH+1, spi_rx_buf, 0));
	  //等待SPI传输完成
    while(!spi_xfer_done);

	
	  //传输完成标志设置为false
		spi_xfer_done = false;
	  //启动数据传输
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, pBuffer+1, size-1, spi_rx_buf, 0));
	  //等待SPI传输完成
    while(!spi_xfer_done);
	  //拉高CS，释放W25Q128FV
		SPIFlash_CS_HIGH;
	  //等待W25Q128FV完成操作
		SpiFlash_Wait_Busy();

    return NRF_SUCCESS;
}
/*****************************************************************************
** 描  述：向指定的地址写入数据，可写入多个页
**         *pBuffer:指向待写入的数据
**         WriteAddr:写入的起始地址
**         size:写入的字节数
** 返回值：RET_SUCCESS
******************************************************************************/
uint8_t SpiFlash_Write_Buf(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t size)
{
    uint32_t PageByteRemain = 0;
	//计算起始地址所处页面的剩余空间
    PageByteRemain = SPIFlash_PAGE_SIZE - WriteAddr%SPIFlash_PAGE_SIZE;
	//如果编程的数据长度不大于页面的剩余空间，编程数据长度等于size
    if(size <= PageByteRemain)
    {
        PageByteRemain = size;
    }
	//分次编程，直到所有的数据编程完成
    while(true)
    {
        //编程PageByteRemain个字节
		SpiFlash_Write_Page(pBuffer,WriteAddr,PageByteRemain);
		//如果编程完成，退出循环
        if(size == PageByteRemain)
        {
            break;
        }
        else
        {
            //计算编程取数据的缓存地址
			pBuffer += PageByteRemain;
			//计算编程地址
            WriteAddr += PageByteRemain;
			//数据长度减去PageByteRemain
            size -= PageByteRemain;
			//计算下次编程的数据长度
            if(size > SPIFlash_PAGE_SIZE)
            {
                PageByteRemain = SPIFlash_PAGE_SIZE;
            }
            else
            {
                PageByteRemain = size;
            }
        }
    }
    return NRF_SUCCESS;
}
/*****************************************************************************
** 描  述：从指定的地址读出指定长度的数据
** 参  数：pBuffer：指向存放读出数据的首地址       
**         ReadAddr：待读出数据的起始地址
**         size：读出的字节数，注意size不能超过pBuffer的大小，否则数组会溢出
** 返回值：
******************************************************************************/
uint8_t SpiFlash_Read(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t size)
{
		uint8_t read_size;
	  spi_tx_buf[0] = SPIFlash_ReadData;
		
		//24位地址，高地址在前
    spi_tx_buf[1] = (uint8_t)((ReadAddr&0x00ff0000)>>16);
    spi_tx_buf[2] = (uint8_t)((ReadAddr&0x0000ff00)>>8);
    spi_tx_buf[3] = (uint8_t)ReadAddr;
	
	  //拉低CS，使能W25Q128FV
	  SPIFlash_CS_LOW;
	  //传输完成标志设置为false
		spi_xfer_done = false;
	  //启动数据传输
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, SPIFLASH_CMD_LENGTH, spi_rx_buf, 0));
	  //等待SPI传输完成
    while(!spi_xfer_done);
	  //开始读取数据
	  while(size!=0)
		{
			if(size<=SPI_TXRX_MAX_LEN)
			{
				read_size = size;
				size = 0;
			}
			else
			{
				read_size = SPI_TXRX_MAX_LEN;
				size -= SPI_TXRX_MAX_LEN;
			}
			//传输完成标志设置为false
		  spi_xfer_done = false;
	    //启动数据传输
	    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 0, pBuffer, read_size));
	    //等待SPI传输完成
      while(!spi_xfer_done);
			pBuffer += read_size;
		}
	  //拉高CS，释放W25Q128FV
    SPIFlash_CS_HIGH;

    return NRF_SUCCESS;
}
//SPI事件处理函数
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
  //设置SPI传输完成  
	spi_xfer_done = true;
}

/*****************************************************************************
** 描  述：配置用于驱动W25Q128的管脚
** 入  参：无
** 返回值：无
******************************************************************************/
void SPI_Flash_Init(void)
{
    //配置用于SPI片选的引脚为输出
	  nrf_gpio_cfg_output(SPI_SS_PIN);
	  //拉高CS
	  SPIFlash_CS_HIGH;
	  //使用默认配置参数初始化SPI配置结构体
	  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	  //重写SPI信号连接的引脚配置
    spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
	  //初始化SPI
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}
/********************************************END FILE*******************************************/



                                                                                                
