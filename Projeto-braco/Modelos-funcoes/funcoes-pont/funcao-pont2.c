#include <stdio.h>
#include <stdlib.h>

int main()
{
int n=10, i = 0;
char *p;
char *pinit;

	if((p=malloc(n*sizeof(char))) == NULL)
		{
			printf(" Nao foi possível alocar memoria \n");
			exit(1);
		}
  pinit = p;
	printf (" %p \n", pinit);

  p = pinit;
  while(i<n){
	printf(" p%d : %p \n *p : %c \n", i+1, p, *p);
  p++;
  i++;
  }
	return(0);
}
