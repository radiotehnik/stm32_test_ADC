#ifndef __USART_UTILS_H
#define __USART_UTILS_H

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h> 


int parseInts(char* buf, int* data, char div); // ¬ходной буфер и выходные данные, разделитель
int amount(char *buf, char div); 




#endif /* __USART_UTILS_H */
