//prototypes

//Initiation
extern void UART1Init(int BAUDRATEREG1);

//UART transmit function
extern void  UART1PutChar(char Ch);
extern void UART1PutStr(char *s);
    
//UART receive function
extern char UART1GetChar();