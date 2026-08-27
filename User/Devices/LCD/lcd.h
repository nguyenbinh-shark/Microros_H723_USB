#ifndef __LCD_H
#define __LCD_H


#include "main.h"
#include "spi.h"


#define USE_ANALOG_SPI 0	// å¯é€‰çš„è½¯ä»¶SPIæˆ–è€…ç¡¬ä»¶SPI		0: ç¡¬ä»¶SPI  1: è½¯ä»¶SPI
#define USE_HORIZONTAL 2  // è®¾ç½®æ¨ªå±æˆ–è€…ç«–å±æ˜¾ç¤º 0æˆ–1ä¸ºç«–å± 2æˆ–3ä¸ºæ¨ªå±

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif

 
//-----------------LCDå¼•è„šå®šä¹‰---------------- 
#if USE_ANALOG_SPI
#define LCD_SCLK_Clr() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port,LCD_SCK_Pin, GPIO_PIN_RESET)//SCL=SCLK
#define LCD_SCLK_Set() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port,LCD_SCK_Pin, GPIO_PIN_SET)

#define LCD_MOSI_Clr() HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_RESET)//SDA=MOSI
#define LCD_MOSI_Set() HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_SET)
#endif

#define LCD_RES_Clr()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_SET)
 		     
#define LCD_CS_Clr()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_RESET)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_SET)

#define LCD_BLK_Clr()  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin, GPIO_PIN_RESET)
#define LCD_BLK_Set()  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin, GPIO_PIN_SET)

void LCD_Init(void); 						//LCDåˆå§‹åŒ–
void LCD_Writ_Bus(uint8_t dat);	// æ¨¡æ‹ŸSPIæ—¶åº
void LCD_WR_DATA8(uint8_t dat);	// å†™å…¥ä¸€ä¸ªå­—èŠ‚
void LCD_WR_DATA(uint16_t dat);	// å†™å…¥ä¸¤ä¸ªå­—èŠ‚
void LCD_WR_REG(uint8_t dat);		// å†™å…¥ä¸€ä¸ªæŒ‡ä»¤
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2); // è®¾ç½®åæ ‡å‡½æ•°

void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color); // æŒ‡å®šåŒºåŸŸå¡«å……é¢œè‰²
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color); // åœ¨æŒ‡å®šä½ç½®ç”»ä¸€ä¸ªç‚¹
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color); // åœ¨æŒ‡å®šä½ç½®ç”»ä¸€æ¡çº¿
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color); // åœ¨æŒ‡å®šä½ç½®ç”»ä¸€ä¸ªçŸ©å½¢
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color); // åœ¨æŒ‡å®šä½ç½®ç”»ä¸€ä¸ªåœ†

void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); 			// æ˜¾ç¤ºæ±‰å­—ä¸²
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // æ˜¾ç¤ºå•ä¸ª12x12æ±‰å­—
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // æ˜¾ç¤ºå•ä¸ª16x16æ±‰å­—
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // æ˜¾ç¤ºå•ä¸ª24x24æ±‰å­—
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // æ˜¾ç¤ºå•ä¸ª32x32æ±‰å­—

void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);				// æ˜¾ç¤ºä¸€ä¸ªå­—ç¬¦
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // æ˜¾ç¤ºå­—ç¬¦ä¸²
uint32_t mypow(uint8_t m,uint8_t n); // æ±‚å¹‚
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey); // æ˜¾ç¤ºæ•´æ•°å˜é‡
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey);	// æ˜¾ç¤ºå¸¦ç¬¦å·çš„æµ®ç‚¹æ•°
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey);	// æ˜¾ç¤ºæ­£çš„æµ®ç‚¹æ•°
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[]); //æ˜¾ç¤ºå›¾ç‰‡


//ç”»ç¬”é¢œè‰²

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //æ£•è‰²
#define BRRED 			     0XFC07 //æ£•çº¢è‰²
#define GRAY  			     0X8430 //ç°è‰²
#define DARKBLUE      	 0X01CF	//æ·±è“è‰²
#define LIGHTBLUE      	 0X7D7C	//æµ…è“è‰²
#define GRAYBLUE       	 0X5458 //ç°è“è‰²
#define LIGHTGREEN     	 0X841F //æµ…ç»¿è‰²
#define LGRAY 			     0XC618 //æµ…ç°è‰²(PANNEL),çª—ä½“èƒŒæ™¯è‰²
#define LGRAYBLUE        0XA651 //æµ…ç°è“è‰²(ä¸­é—´å±‚é¢œè‰²)
#define LBBLUE           0X2B12 //æµ…æ£•è“è‰²(é€‰æ‹©æ¡ç›®çš„åè‰²)
#endif





