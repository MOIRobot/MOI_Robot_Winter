#include <iostream>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>

int main(int argc ,char*argv[])
{
	float **p;
	int size = 5;
	p = (float **)malloc(sizeof(float *) * size);
	if(p == NULL){
		printf("Error\n");
		return -1;
	}
	for(int i = 0; i < size; i++)
		*(p + i) = (float*)malloc(sizeof(float)*size);
	
	
	for(int i = 0; i < size; i++)
	{
		for(int j = 0; j < size; j++)
		{
			*(*(p+i) + j) = rand()%3;
			printf("%6.3f", p[i][j]);
		}
		printf("\n");
	}
	return 0;
}











