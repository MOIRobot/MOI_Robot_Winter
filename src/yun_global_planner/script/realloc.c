#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>

int main(int argc, char* argv[]){

	int *p = NULL;
	p = malloc(sizeof(int)*5);
	int *q = p;
	printf("before realloc.\r\n");
	printf("p = %ld, q = %ld\t\n", sizeof(p), sizeof(q));
	p = (int*)realloc(p, sizeof(int)*10);
	printf("after realloc.\r\n");
	printf("p = %ld, q = %ld\t\n", sizeof(p), sizeof(q));
	free(p);
	free(q);
/*
	int *p = NULL;
	int a;
	int i;
	//p = (int *)malloc(sizeof(int) * 10);
	//memoryset
	//memset(p, 0, sizeof(int) * 10);
	p = (int *)calloc(10, sizeof(int));
	for(i = 0; i < 10; i++){
		a = *(p + i);
		printf("a = %d\r\n", a);
	}
	printf("sizeof int : %ld\n", sizeof(int));
	free(p);
	*/
	return 0;
	
}
