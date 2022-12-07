#include "Usart_utils.h"


int parseInts(char* buf, int* data, char div) // ¬ходной буфер и выходные данные, разделитель
{
    int count = 0;
    char* offset = buf;
	
    while (true) 
	{
        data[count++] = atoi(offset);
        offset = strchr(offset, div);
        if (offset) 
		{
			offset++;
		}
        else 
		{
			break;
		}
     }
	 
    return count;
}


int amount(char *buf, char div) 
	{
        int i = 0, count = 0;
        while (buf[i]) if (buf[i++] == div) count++;  // подсчЄт разделителей
        return ++count;
    }