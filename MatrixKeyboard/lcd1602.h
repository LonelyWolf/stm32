#define	pin_E	GPIO_Pin_10
#define	pin_RS	GPIO_Pin_11
#define pin_DB4 GPIO_Pin_15
#define pin_DB5 GPIO_Pin_14
#define pin_DB6 GPIO_Pin_13
#define pin_DB7	GPIO_Pin_12

/*
 *   Declare Functions
 */
extern void LCD_Init(void);
extern void LCD_cmd_4bit(uc8 cmd);
extern void LCD_data_4bit(uc8 data);
extern void LCD_Cls(void);
extern void LCD_GotoXY(int column, int line);
extern void LCD_Print(char *string);
extern void LCD_PrintI(uint32_t num);
extern void LCD_PrintH(uint32_t num);
extern void LCD_PrintB8(uint8_t num);
extern void LCD_PrintB16(uint16_t num);
