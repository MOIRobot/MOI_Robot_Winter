#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

int main()
{
	if((write(1, "Here is some data\n", 18) != 18))
		write(2, "error !\n", 40);
	
	
	
	char *menu[] = {
		"a - add new record",
		"d - delete record",
		"q - quit",
	};
	char **option;
	
	option = menu;
	while(*option){
		printf("%c\n", *option[0]);
		option++;
	}
	
	exit(0);

}
