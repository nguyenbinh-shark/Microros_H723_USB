#include "lcd.h"
#include "lcdfont.h"

/**
************************************************************************
* @brief:      	LCD_Writ_Bus: LCD´®ÐÐÊý¾ÝÐ´Èëº¯Êý
* @param:      	dat - ÒªÐ´ÈëµÄ´®ÐÐÊý¾Ý
* @retval:     	void
* @details:    	½«´®ÐÐÊý¾ÝÐ´ÈëLCD£¬¸ù¾ÝÊ¹ÓÃµÄÍ¨ÐÅ·½Ê½Ñ¡ÔñÊ¹ÓÃÈí¼þÄ£ÄâSPI£¨USE_ANALOG_SPIºê¶¨ÒåÎªÕæ£©»òÓ²¼þSPI¡£
*               - µ±USE_ANALOG_SPIºê¶¨ÒåÎªÕæÊ±£¬Í¨¹ýGPIO¿ØÖÆSCLK¡¢MOSIºÍCSÐÅºÅ£¬Ñ­»·½«8Î»Êý¾ÝÐ´Èë¡£
*               - µ±USE_ANALOG_SPIºê¶¨ÒåÎª¼ÙÊ±£¬Í¨¹ýHAL_SPI_Transmitº¯ÊýÊ¹ÓÃÓ²¼þSPI´«Êä1×Ö½ÚÊý¾Ý¡£
************************************************************************
**/
void LCD_Writ_Bus(uint8_t dat) 
{	
	LCD_CS_Clr();
#if USE_ANALOG_SPI
	for(uint8_t i=0;i<8;i++) {			  
		LCD_SCLK_Clr();
		if(dat&0x80) {
		   LCD_MOSI_Set();
		} else {
		   LCD_MOSI_Clr();
		}
		LCD_SCLK_Set();
		dat<<=1;
	}
#else
	HAL_SPI_Transmit(&hspi1, &dat, 1, 0xffff);
#endif	
  LCD_CS_Set();	
}
/**
************************************************************************
* @brief:      	LCD_WR_DATA8: ÏòLCDÐ´Èë8Î»Êý¾Ý
* @param:      	dat - ÒªÐ´ÈëµÄ8Î»Êý¾Ý
* @retval:     	void
* @details:    	µ÷ÓÃLCD_Writ_Busº¯Êý½«8Î»Êý¾ÝÐ´ÈëLCD¡£
************************************************************************
**/
void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}
/**
************************************************************************
* @brief:      	LCD_WR_DATA: ÏòLCDÐ´Èë16Î»Êý¾Ý
* @param:      	dat - ÒªÐ´ÈëµÄ16Î»Êý¾Ý
* @retval:     	void
* @details:    	µ÷ÓÃLCD_Writ_Busº¯Êý½«16Î»Êý¾ÝµÄ¸ß8Î»ºÍµÍ8Î»·Ö±ðÐ´ÈëLCD¡£
************************************************************************
**/
void LCD_WR_DATA(uint16_t dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}
/**
************************************************************************
* @brief:      	LCD_WR_REG: ÏòLCDÐ´Èë¼Ä´æÆ÷µØÖ·
* @param:      	dat - ÒªÐ´ÈëµÄ¼Ä´æÆ÷µØÖ·
* @retval:     	void
* @details:    	Í¨¹ýµ÷ÓÃLCD_Writ_Busº¯ÊýÏòLCDÐ´Èë¼Ä´æÆ÷µØÖ·¡£
************************************************************************
**/
/* ---- Bulk writes ---------------------------------------------------
 * The drawing code thinks in pixels, but the panel must not be written one HAL
 * call per pixel. HAL_SPI_Transmit() pays a lock, a state check, a timeout
 * setup and a completion spin no matter how few bytes it carries, and
 * LCD_Writ_Bus() paid all of that per BYTE -- plus a CS pulse the ST7789 does
 * not need in the middle of a burst. A full-screen fill is 134,400 bytes.
 *
 * These helpers hold CS down for a whole run and hand the peripheral as many
 * bytes as it will take per call.
 *
 * s_burst is shared and unguarded. Every drawing call comes either from main()
 * before the scheduler starts or from LCD_Task, never from two contexts at
 * once. Draw from a second task and this buffer is what breaks first.
 */
#define LCD_MAX_GLYPH_PX   (16U * 32U)                  /* 16x32, largest font */
#define LCD_BURST_BYTES    (LCD_MAX_GLYPH_PX * 2U)      /* one glyph, one shot */

static uint8_t s_burst[LCD_BURST_BYTES];

static void LCD_Write_Burst(const uint8_t *buf, uint32_t len)
{
	LCD_CS_Clr();
	while (len)
	{
		uint16_t chunk = (len > 0xFFFFU) ? 0xFFFFU : (uint16_t)len;
		HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, chunk, HAL_MAX_DELAY);
		buf += chunk;
		len -= chunk;
	}
	LCD_CS_Set();
}

/* Repeat one colour for px pixels. The pattern is built once and the same
   buffer re-sent, so clearing the screen costs 132 transfers, not 134,400. */
static void LCD_Write_Color(uint16_t color, uint32_t px)
{
	uint32_t i;
	uint32_t run = (px < LCD_BURST_BYTES / 2U) ? px : (LCD_BURST_BYTES / 2U);

	for (i = 0; i < run; i++)
	{
		s_burst[2U * i]      = (uint8_t)(color >> 8);
		s_burst[2U * i + 1U] = (uint8_t)color;
	}

	LCD_CS_Clr();
	while (px)
	{
		uint32_t chunk = (px < run) ? px : run;
		HAL_SPI_Transmit(&hspi1, s_burst, (uint16_t)(chunk * 2U), HAL_MAX_DELAY);
		px -= chunk;
	}
	LCD_CS_Set();
}

/* One row-byte of a glyph. Returns 0 for a size with no table; callers screen
   for that before drawing anything. */
static uint8_t LCD_FontByte(uint8_t idx, uint16_t i, uint8_t sizey)
{
	switch (sizey)
	{
		case 12: return ascii_1206[idx][i];
		case 16: return ascii_1608[idx][i];
		case 24: return ascii_2412[idx][i];
		case 32: return ascii_3216[idx][i];
		default: return 0;
	}
}

void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr();//Ð´ÃüÁî
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//Ð´Êý¾Ý
}
/**
************************************************************************
* @brief:      	LCD_Address_Set: ÉèÖÃLCDÏÔÊ¾ÇøÓòµÄµØÖ··¶Î§
* @param:      	x1, y1, x2, y2 - ÏÔÊ¾ÇøÓòµÄ×óÉÏ½ÇºÍÓÒÏÂ½Ç×ø±ê
* @retval:     	void
* @details:    	¸ù¾ÝÆÁÄ»·½ÏòÍ¨¹ýµ÷ÓÃLCD_WR_REGºÍLCD_WR_DATAº¯Êý£¬ÉèÖÃLCDµÄÁÐµØÖ·ºÍÐÐµØÖ·£¬È»ºóÐ´Èë´¢´æÆ÷½øÐÐÏÔÊ¾¡£
************************************************************************
**/
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//ÁÐµØÖ·ÉèÖÃ
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//ÐÐµØÖ·ÉèÖÃ
		LCD_WR_DATA(y1+20);
		LCD_WR_DATA(y2+20);
		LCD_WR_REG(0x2c);//´¢´æÆ÷Ð´
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//ÁÐµØÖ·ÉèÖÃ
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//ÐÐµØÖ·ÉèÖÃ
		LCD_WR_DATA(y1+20);
		LCD_WR_DATA(y2+20);
		LCD_WR_REG(0x2c);//´¢´æÆ÷Ð´
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//ÁÐµØÖ·ÉèÖÃ
		LCD_WR_DATA(x1+20);
		LCD_WR_DATA(x2+20);
		LCD_WR_REG(0x2b);//ÐÐµØÖ·ÉèÖÃ
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//´¢´æÆ÷Ð´
	}
	else
	{
		LCD_WR_REG(0x2a);//ÁÐµØÖ·ÉèÖÃ
		LCD_WR_DATA(x1+20);
		LCD_WR_DATA(x2+20);
		LCD_WR_REG(0x2b);//ÐÐµØÖ·ÉèÖÃ
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//´¢´æÆ÷Ð´
	}
}
/**
************************************************************************
* @brief:      	LCD_Fill: ÔÚLCDÖ¸¶¨ÇøÓòÌî³äÖ¸¶¨ÑÕÉ«
* @param:      	xsta, ysta, xend, yend - Ìî³äÇøÓòµÄ×óÉÏ½ÇºÍÓÒÏÂ½Ç×ø±ê
*              	color - Ìî³äÑÕÉ«
* @retval:     	void
* @details:    	Í¨¹ýµ÷ÓÃLCD_Address_Setº¯ÊýÉèÖÃLCDÏÔÊ¾ÇøÓòµÄµØÖ··¶Î§£¬È»ºóÔÚ¸Ã·¶Î§ÄÚÌî³äÖ¸¶¨ÑÕÉ«¡£
************************************************************************
**/
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{
	/* xend/yend are exclusive. Clamping them is not cosmetic: the UI draws its
	   rules as LCD_DrawLine(0, y, LCD_W, y), one pixel past the right edge,
	   which the old per-pixel loop absorbed silently and an address window
	   would not. */
	if (xend > LCD_W) xend = LCD_W;
	if (yend > LCD_H) yend = LCD_H;
	if (xend <= xsta || yend <= ysta) return;

	LCD_Address_Set(xsta,ysta,xend-1,yend-1);
	LCD_Write_Color(color, (uint32_t)(xend - xsta) * (uint32_t)(yend - ysta));
}
/**
************************************************************************
* @brief:      	LCD_DrawPoint: ÔÚLCDÖ¸¶¨Î»ÖÃ»­µã
* @param:      	x, y - µãµÄ×ø±ê
*              	color - µãµÄÑÕÉ«
* @retval:     	void
* @details:    	Í¨¹ýµ÷ÓÃLCD_Address_Setº¯ÊýÉèÖÃLCDÏÔÊ¾ÇøÓòµÄµØÖ··¶Î§£¬È»ºóÔÚÖ¸¶¨Î»ÖÃ»­µã£¬ÑÕÉ«ÓÉcolor²ÎÊýÖ¸¶¨¡£
************************************************************************
**/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Address_Set(x,y,x,y);//ÉèÖÃ¹â±êÎ»ÖÃ 
	LCD_WR_DATA(color);
} 
/**
************************************************************************
* @brief:      	LCD_DrawLine: ÔÚLCDÉÏ»­Ïß
* @param:      	x1, y1 - ÏßµÄÆðÊ¼×ø±ê
*              	x2, y2 - ÏßµÄ½áÊø×ø±ê
*              	color - ÏßµÄÑÕÉ«
* @retval:     	void
* @details:    	Í¨¹ý¼ÆËã×ø±êÔöÁ¿£¬Ñ¡Ôñ»ù±¾ÔöÁ¿×ø±êÖá£¬´ÓÆðÊ¼×ø±êµ½½áÊø×ø±êÖðµã»­Ïß£¬ÑÕÉ«ÓÉcolor²ÎÊýÖ¸¶¨¡£
************************************************************************
**/
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;

	/* Horizontal and vertical runs are the only lines this UI draws -- card
	   borders and rules -- and LCD_DrawPoint sets a fresh address window for
	   every pixel, 13 bytes where 2 will do. LCD_Fill sets one and streams.
	   Coordinates here are inclusive; LCD_Fill's ends are exclusive. */
	if (y1 == y2)
	{
		uint16_t xa = (x1 < x2) ? x1 : x2;
		uint16_t xb = (x1 < x2) ? x2 : x1;
		LCD_Fill(xa, y1, (uint16_t)(xb + 1U), (uint16_t)(y1 + 1U), color);
		return;
	}
	if (x1 == x2)
	{
		uint16_t ya = (y1 < y2) ? y1 : y2;
		uint16_t yb = (y1 < y2) ? y2 : y1;
		LCD_Fill(x1, ya, (uint16_t)(x1 + 1U), (uint16_t)(yb + 1U), color);
		return;
	}

	/* Bresenham, unchanged, for the diagonals nothing currently asks for. */
	delta_x=x2-x1;
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;
	if(delta_x>0)incx=1;
	else if (delta_x==0)incx=0;
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x;
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}
/**
************************************************************************
* @brief:      	LCD_DrawRectangle: ÔÚLCDÉÏ»­¾ØÐÎ
* @param:      	x1, y1 - ¾ØÐÎµÄ×óÉÏ½Ç×ø±ê
*              	x2, y2 - ¾ØÐÎµÄÓÒÏÂ½Ç×ø±ê
*              	color - ¾ØÐÎµÄÑÕÉ«
* @retval:     	void
* @details:    	Í¨¹ýµ÷ÓÃLCD_DrawLineº¯Êý»­³ö¾ØÐÎµÄËÄÌõ±ß£¬ÑÕÉ«ÓÉcolor²ÎÊýÖ¸¶¨¡£
************************************************************************
**/
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}
/**
************************************************************************
* @brief:      	Draw_Circle: ÔÚLCDÉÏ»­Ô²
* @param:      	x0, y0 - Ô²ÐÄ×ø±ê
*              	r - Ô²µÄ°ë¾¶
*              	color - Ô²µÄÑÕÉ«
* @retval:     	void
* @details:    	Ê¹ÓÃÖÐµã»­Ô²·¨£¬ÒÔÔ²ÐÄ(x0, y0)ÎªÖÐÐÄ£¬°ë¾¶Îªr£¬ÔÚLCDÉÏ»­³öÔ²£¬
*              	ÑÕÉ«ÓÉcolor²ÎÊýÖ¸¶¨¡£
************************************************************************
**/
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//ÅÐ¶ÏÒª»­µÄµãÊÇ·ñ¹ýÔ¶
		{
			b--;
		}
	}
}
/**
************************************************************************
* @brief:      	LCD_ShowChinese: ÔÚLCDÉÏÏÔÊ¾ºº×Ö×Ö·û´®
* @param:      	x, y - ÆðÊ¼×ø±ê£¬ÏÔÊ¾ºº×Ö×Ö·û´®µÄ×óÉÏ½Ç×ø±ê
*              	s - ÒªÏÔÊ¾µÄºº×Ö×Ö·û´®£¬Ã¿¸öºº×ÖÕ¼Á½¸ö×Ö½Ú
*              	fc - ×ÖÌåÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ºº×Ö×ÖÌå´óÐ¡£¬Ö§³Ö12x12¡¢16x16¡¢24x24¡¢32x32
*              	mode - ÏÔÊ¾Ä£Ê½£¬1±íÊ¾·´É«£¬0±íÊ¾Õý³£ÏÔÊ¾
* @retval:     	void
* @details:    	¸ù¾ÝsizeyÑ¡Ôñ×ÖÌå´óÐ¡£¬ÔÚLCDÉÏÏÔÊ¾ºº×Ö×Ö·û´®£¬¿ÉÒÔÑ¡ÔñÏÔÊ¾Ä£Ê½¡£
************************************************************************
**/
void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	while(*s!=0)
	{
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		s+=2;
		x+=sizey;
	}
}
/**
************************************************************************
* @brief:      	LCD_ShowChinese12x12: ÔÚLCDÉÏÏÔÊ¾12x12ºº×Ö
* @param:      	x, y - ÆðÊ¼×ø±ê£¬ÏÔÊ¾ºº×ÖµÄ×óÉÏ½Ç×ø±ê
*              	s - ÒªÏÔÊ¾µÄºº×Ö£¬Ã¿¸öºº×ÖÕ¼Á½¸ö×Ö½Ú
*              	fc - ×ÖÌåÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ºº×Ö×ÖÌå´óÐ¡£¬Ö§³Ö12x12
*              	mode - ÏÔÊ¾Ä£Ê½£¬1±íÊ¾·´É«£¬0±íÊ¾Õý³£ÏÔÊ¾
* @retval:     	void
* @details:    	ÔÚLCDÉÏÏÔÊ¾12x12ºº×Ö£¬¿ÉÒÔÑ¡ÔñÏÔÊ¾Ä£Ê½¡£
************************************************************************
**/
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//ºº×ÖÊýÄ¿
	uint16_t TypefaceNum;//Ò»¸ö×Ö·ûËùÕ¼×Ö½Ú´óÐ¡
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//Í³¼Æºº×ÖÊýÄ¿
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//·Çµþ¼Ó·½Ê½
					{
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//µþ¼Ó·½Ê½
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//»­Ò»¸öµã
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //²éÕÒµ½¶ÔÓ¦µãÕó×Ö¿âÁ¢¼´ÍË³ö£¬·ÀÖ¹¶à¸öºº×ÖÖØ¸´È¡Ä£´øÀ´Ó°Ïì
	}
} 
/**
************************************************************************
* @brief:      	LCD_ShowChinese16x16: ÔÚLCDÉÏÏÔÊ¾16x16ºº×Ö
* @param:      	x, y - ÆðÊ¼×ø±ê£¬ÏÔÊ¾ºº×ÖµÄ×óÉÏ½Ç×ø±ê
*              	s - ÒªÏÔÊ¾µÄºº×Ö£¬Ã¿¸öºº×ÖÕ¼Á½¸ö×Ö½Ú
*              	fc - ×ÖÌåÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ºº×Ö×ÖÌå´óÐ¡£¬Ö§³Ö16x16
*              	mode - ÏÔÊ¾Ä£Ê½£¬1±íÊ¾·´É«£¬0±íÊ¾Õý³£ÏÔÊ¾
* @retval:     	void
* @details:    	ÔÚLCDÉÏÏÔÊ¾16x16ºº×Ö£¬¿ÉÒÔÑ¡ÔñÏÔÊ¾Ä£Ê½¡£
************************************************************************
**/
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//ºº×ÖÊýÄ¿
	uint16_t TypefaceNum;//Ò»¸ö×Ö·ûËùÕ¼×Ö½Ú´óÐ¡
	uint16_t x0=x;
  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//Í³¼Æºº×ÖÊýÄ¿
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//·Çµþ¼Ó·½Ê½
					{
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//µþ¼Ó·½Ê½
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//»­Ò»¸öµã
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //²éÕÒµ½¶ÔÓ¦µãÕó×Ö¿âÁ¢¼´ÍË³ö£¬·ÀÖ¹¶à¸öºº×ÖÖØ¸´È¡Ä£´øÀ´Ó°Ïì
	}
} 
/**
************************************************************************
* @brief:      	LCD_ShowChinese24x24: ÔÚLCDÉÏÏÔÊ¾24x24ºº×Ö
* @param:      	x, y - ÆðÊ¼×ø±ê£¬ÏÔÊ¾ºº×ÖµÄ×óÉÏ½Ç×ø±ê
*              	s - ÒªÏÔÊ¾µÄºº×Ö£¬Ã¿¸öºº×ÖÕ¼Á½¸ö×Ö½Ú
*              	fc - ×ÖÌåÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ºº×Ö×ÖÌå´óÐ¡£¬Ö§³Ö24x24
*              	mode - ÏÔÊ¾Ä£Ê½£¬1±íÊ¾·´É«£¬0±íÊ¾Õý³£ÏÔÊ¾
* @retval:     	void
* @details:    	ÔÚLCDÉÏÏÔÊ¾24x24ºº×Ö£¬¿ÉÒÔÑ¡ÔñÏÔÊ¾Ä£Ê½¡£
************************************************************************
**/
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//ºº×ÖÊýÄ¿
	uint16_t TypefaceNum;//Ò»¸ö×Ö·ûËùÕ¼×Ö½Ú´óÐ¡
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//Í³¼Æºº×ÖÊýÄ¿
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//·Çµþ¼Ó·½Ê½
					{
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//µþ¼Ó·½Ê½
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//»­Ò»¸öµã
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //²éÕÒµ½¶ÔÓ¦µãÕó×Ö¿âÁ¢¼´ÍË³ö£¬·ÀÖ¹¶à¸öºº×ÖÖØ¸´È¡Ä£´øÀ´Ó°Ïì
	}
} 
/**
************************************************************************
* @brief:      	LCD_ShowChinese32x32: ÔÚLCDÉÏÏÔÊ¾32x32ºº×Ö
* @param:      	x, y - ÆðÊ¼×ø±ê£¬ÏÔÊ¾ºº×ÖµÄ×óÉÏ½Ç×ø±ê
*              	s - ÒªÏÔÊ¾µÄºº×Ö£¬Ã¿¸öºº×ÖÕ¼Á½¸ö×Ö½Ú
*              	fc - ×ÖÌåÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ºº×Ö×ÖÌå´óÐ¡£¬Ö§³Ö32x32
*              	mode - ÏÔÊ¾Ä£Ê½£¬1±íÊ¾·´É«£¬0±íÊ¾Õý³£ÏÔÊ¾
* @retval:     	void
* @details:    	ÔÚLCDÉÏÏÔÊ¾32x32ºº×Ö£¬¿ÉÒÔÑ¡ÔñÏÔÊ¾Ä£Ê½¡£
************************************************************************
**/
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//ºº×ÖÊýÄ¿
	uint16_t TypefaceNum;//Ò»¸ö×Ö·ûËùÕ¼×Ö½Ú´óÐ¡
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//Í³¼Æºº×ÖÊýÄ¿
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//·Çµþ¼Ó·½Ê½
					{
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//µþ¼Ó·½Ê½
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//»­Ò»¸öµã
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //²éÕÒµ½¶ÔÓ¦µãÕó×Ö¿âÁ¢¼´ÍË³ö£¬·ÀÖ¹¶à¸öºº×ÖÖØ¸´È¡Ä£´øÀ´Ó°Ïì
	}
}
/**
************************************************************************
* @brief:      	LCD_ShowChar: ÔÚLCDÉÏÏÔÊ¾µ¥¸ö×Ö·û
* @param:      	x, y - ÆðÊ¼×ø±ê£¬ÏÔÊ¾×Ö·ûµÄ×óÉÏ½Ç×ø±ê
*              	num - ÒªÏÔÊ¾µÄ×Ö·ûµÄASCIIÂëÖµ
*              	fc - ×ÖÌåÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ×ÖÌå´óÐ¡£¬Ö§³Ö12x6, 16x8, 24x12, 32x16
*              	mode - ÏÔÊ¾Ä£Ê½£¬1±íÊ¾·´É«£¬0±íÊ¾Õý³£ÏÔÊ¾
* @retval:     	void
* @details:    	ÔÚLCDÉÏÏÔÊ¾µ¥¸ö×Ö·û£¬¿ÉÒÔÑ¡ÔñÏÔÊ¾Ä£Ê½ºÍ²»Í¬µÄ×ÖÌå´óÐ¡¡£
************************************************************************
**/
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;
	uint16_t x0=x;

	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';

	if(sizey!=12 && sizey!=16 && sizey!=24 && sizey!=32) return;

	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);

	if(!mode)
	{
		/* Opaque mode -- the one the whole UI uses. Every pixel of the cell is
		   written, foreground or background, so the glyph can be built in RAM
		   and pushed in a single transfer: 144 bytes for 6x12, and even 16x32
		   fits s_burst exactly. */
		uint32_t px = 0;

		for(i=0;i<TypefaceNum;i++)
		{
			temp = LCD_FontByte((uint8_t)num, i, sizey);
			for(t=0;t<8;t++)
			{
				uint16_t c = (temp&(0x01<<t)) ? fc : bc;
				s_burst[2U*px]      = (uint8_t)(c >> 8);
				s_burst[2U*px + 1U] = (uint8_t)c;
				px++;
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
		}
		LCD_Write_Burst(s_burst, px * 2U);
		return;
	}

	/* Overlay mode leaves the background untouched, so it has to go pixel by
	   pixel. Nothing in this project asks for it. */
	for(i=0;i<TypefaceNum;i++)
	{
		temp = LCD_FontByte((uint8_t)num, i, sizey);
		for(t=0;t<8;t++)
		{
			if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);
			x++;
			if((x-x0)==sizex)
			{
				x=x0;
				y++;
				break;
			}
		}
	}
}
/**
************************************************************************
* @brief:      	LCD_ShowString: ÔÚLCDÉÏÏÔÊ¾×Ö·û´®
* @param:      	x, y - ÆðÊ¼×ø±ê£¬ÏÔÊ¾×Ö·û´®µÄ×óÉÏ½Ç×ø±ê
*              	p - ÒªÏÔÊ¾µÄ×Ö·û´®£¬ÒÔ'\0'½áÎ²
*              	fc - ×ÖÌåÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ×ÖÌå´óÐ¡£¬Ö§³Ö12x6, 16x8, 24x12, 32x16
*              	mode - ÏÔÊ¾Ä£Ê½£¬1±íÊ¾·´É«£¬0±íÊ¾Õý³£ÏÔÊ¾
* @retval:     	void
* @details:    	ÔÚLCDÉÏÏÔÊ¾×Ö·û´®£¬¿ÉÒÔÑ¡ÔñÏÔÊ¾Ä£Ê½ºÍ²»Í¬µÄ×ÖÌå´óÐ¡¡£
************************************************************************
**/
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}
/**
************************************************************************
* @brief:      	mypow: ¼ÆËãmµÄn´Î·½
* @param:      	m - µ×Êý
*              	n - Ö¸Êý
* @retval:     	uint32_t - mµÄn´Î·½
* @details:    	¼ÆËãmµÄn´Î·½£¬·µ»Ø½á¹û¡£
************************************************************************
**/
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}
/**
************************************************************************
* @brief:      	LCD_ShowIntNum: ÔÚLCDÉÏÏÔÊ¾ÕûÊýÊý×Ö
* @param:      	x - x×ø±ê
*              	y - y×ø±ê
*              	num - ÒªÏÔÊ¾µÄÕûÊý
*              	len - Êý×ÖÏÔÊ¾µÄÎ»Êý
*              	fc - ×ÖÌåÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ×ÖÌå´óÐ¡
* @retval:     	void
* @details:    	ÔÚLCDÉÏÏÔÊ¾ÕûÊýÊý×Ö£¬Ö§³ÖÉèÖÃÏÔÊ¾µÄÎ»Êý¡¢×ÖÌåÑÕÉ«¡¢±³¾°ÑÕÉ«ºÍ×ÖÌå´óÐ¡¡£
************************************************************************
**/
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 
/**
************************************************************************
* @brief:      	LCD_ShowFloatNum: ÔÚLCDÉÏÏÔÊ¾¸ñÊ½»¯µÄ¸¡µãÊý£¬Ö§³Ö¸ºÊý
* @param:      	x - x×ø±ê
*              	y - y×ø±ê
*              	num - ÒªÏÔÊ¾µÄ¸¡µãÊý
*              	len - ÕûÊýÎ»Êý
*              	decimal - Ð¡ÊýÎ»Êý
*              	fc - ×ÖµÄÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ×ÖÌå´óÐ¡
* @retval:     	void
* @details:    	ÔÚLCDÉÏÏÔÊ¾¸ñÊ½»¯µÄ¸¡µãÊý£¬Ö§³ÖÉèÖÃÕûÊýÎ»Êý¡¢Ð¡ÊýÎ»Êý¡¢×ÖÌåÑÕÉ«¡¢±³¾°ÑÕÉ«ºÍ×ÖÌå´óÐ¡¡£
************************************************************************
**/
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
		int16_t num_int;
		uint8_t t, temp, sizex;
    sizex = sizey / 2;
    num_int = num * mypow(10, decimal);

    if (num < 0)
    {
        LCD_ShowChar(x, y, '-', fc, bc, sizey, 0);
        num_int = -num_int;
        x += sizex;
        len++;
    }
    else
    {
        LCD_ShowChar(x, y, ' ', fc, bc, sizey, 0);
        num_int = num_int;
        x += sizex;
        len++;
    }

    // ÔÚ¸üÐÂÊý×ÖÊ±Ë¢ÐÂÏÔÊ¾µÄÎ»ÖÃ
    LCD_Fill(x, y, x + len * sizex + decimal + 1, y + sizey + 1, bc);

    for (t = 0; t < len; t++)
    {
        if (t == (len - decimal))
        {
            LCD_ShowChar(x + (len - decimal) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        temp = ((num_int / mypow(10, len - t - 1)) % 10) + '0';
        LCD_ShowChar(x + t * sizex, y, temp, fc, bc, sizey, 0);
    }
}
/**
************************************************************************
* @brief:      	LCD_ShowFloatNum1: ÔÚLCDÉÏÏÔÊ¾¸ñÊ½»¯µÄ¸¡µãÊý£¬²»Ö§³Ö¸ºÊý
* @param:      	x - x×ø±ê
*              	y - y×ø±ê
*              	num - ÒªÏÔÊ¾µÄ¸¡µãÊý
*              	len - ÕûÊýÎ»Êý
*              	decimal - Ð¡ÊýÎ»Êý
*              	fc - ×ÖµÄÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ×ÖÌå´óÐ¡
* @retval:     	void
* @details:    	ÔÚLCDÉÏÏÔÊ¾¸ñÊ½»¯µÄ¸¡µãÊý£¬Ö§³ÖÉèÖÃÕûÊýÎ»Êý¡¢Ð¡ÊýÎ»Êý¡¢×ÖÌåÑÕÉ«¡¢±³¾°ÑÕÉ«ºÍ×ÖÌå´óÐ¡¡£
************************************************************************
**/
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
		int16_t num_int;
		uint8_t t, temp, sizex;
    sizex = sizey / 2;
    num_int = num * mypow(10, decimal);


		num_int = num_int;
		x += sizex;
		len++;

    // ÔÚ¸üÐÂÊý×ÖÊ±Ë¢ÐÂÏÔÊ¾µÄÎ»ÖÃ
    LCD_Fill(x, y, x + len * sizex + decimal + 1, y + sizey + 1, bc);

    for (t = 0; t < len; t++)
    {
        if (t == (len - decimal))
        {
            LCD_ShowChar(x + (len - decimal) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        temp = ((num_int / mypow(10, len - t - 1)) % 10) + '0';
        LCD_ShowChar(x + t * sizex, y, temp, fc, bc, sizey, 0);
    }
}
/**
************************************************************************
* @brief:      	LCD_ShowFloatNum1: ÔÚLCDÉÏÏÔÊ¾¸ñÊ½»¯µÄ¸¡µãÊý£¬²»Ö§³Ö¸ºÊý
* @param:      	x - x×ø±ê
*              	y - y×ø±ê
*              	num - ÒªÏÔÊ¾µÄ¸¡µãÊý
*              	len - ÕûÊýÎ»Êý
*              	decimal - Ð¡ÊýÎ»Êý
*              	fc - ×ÖµÄÑÕÉ«
*              	bc - ±³¾°ÑÕÉ«
*              	sizey - ×ÖÌå´óÐ¡
* @retval:     	void
* @details:    	ÔÚLCDÉÏÏÔÊ¾¸ñÊ½»¯µÄ¸¡µãÊý£¬Ö§³ÖÉèÖÃÕûÊýÎ»Êý¡¢Ð¡ÊýÎ»Êý¡¢×ÖÌåÑÕÉ«¡¢±³¾°ÑÕÉ«ºÍ×ÖÌå´óÐ¡¡£
************************************************************************
**/
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[])
{
	/* pic[] is already big-endian RGB565 in the panel's own order, so it goes
	   out as one run instead of two HAL calls per pixel. */
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	LCD_Write_Burst(pic, (uint32_t)length * (uint32_t)width * 2U);
}
/**
************************************************************************
* @brief:      	LCD_Init: LCD³õÊ¼»¯º¯Êý
* @param:      	void
* @details:    	Ö´ÐÐLCDµÄ³õÊ¼»¯¹ý³Ì£¬°üÀ¨¸´Î»¡¢±³¹â¿ØÖÆ¡¢¼Ä´æÆ÷ÅäÖÃµÈ
* @retval:     	void
************************************************************************
**/
void LCD_Init(void)
{
	
	LCD_RES_Clr();//¸´Î»
	HAL_Delay(100);
	LCD_RES_Set();
	HAL_Delay(100);
	
	LCD_BLK_Set();//´ò¿ª±³¹â
  HAL_Delay(100);
	
	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	HAL_Delay(120);              //Delay 120ms 
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);			
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);			
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C); 
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0x33); 
	LCD_WR_DATA8(0x33); 			

	LCD_WR_REG(0xB7);			
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xBB);			
	LCD_WR_DATA8(0x32); //Vcom=1.35V
					
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);			
	LCD_WR_DATA8(0x15); //GVDD=4.8V  ÑÕÉ«Éî¶È
				
	LCD_WR_REG(0xC4);			
	LCD_WR_DATA8(0x20); //VDV, 0x20:0v

	LCD_WR_REG(0xC6);			
	LCD_WR_DATA8(0x0F); //0x0F:60Hz        	

	LCD_WR_REG(0xD0);			
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1); 

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);   

	LCD_WR_REG(0xE1);     
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);
	LCD_WR_REG(0x21); 

	LCD_WR_REG(0x29);
}

