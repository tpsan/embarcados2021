#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

pthread_t id0, id1, id2, id3;
int flag = 1;

void prtcl(char net[4],int ndId, char rw[5], char index[4], char subindex[2], int valor) {

  char  call[43]  ="";
  char  call1[20] ="";
  char  nd[3]     ="";
  char  indx1[2]  ="";
  char  indx2[2]  ="";
  char  sindx[2]  ="";
  char  valc[8]   ="";
  char  valc1[2]  ="";
  char  valc2[2]  ="";
  char  valc3[2]  ="";
  char  valc4[2]  ="";
  int   id=600+ndId;
  int   val1;
  int   i=0;
  int   j=0;

  //formato da string no final deve ser
  // "./send.sh can0 602#40.10.17.00.00.00.00.00"

  //Define o Script a ser chamado
  strcat(call, "./send");
  strcat(call, ".sh ");

  // Define a Rede que se encontra os nós
  strcat(call, net);
  strcat(call, " ");

  // Define o nó para qual será enviado
  sprintf(nd, "%d", id);
  strcat(call, nd);
  strcat(call, "#");

  //Define se vai ser Read ou Write no barramento
  if (!(strcmp(rw,"write"))){
    strcat(call,"2B.");
  }
  else {
    strcat(call,"40.");
  }

  //Define o Index do nó chamado
  indx1[0] = index[2];
  indx1[1] = index[3];
  strcat(call, indx1);
  strcat(call, ".");

  indx2[0] = index[0];
  indx2[1] = index[1];
  strcat(call, indx2);
  strcat(call, ".");

  //Define o SubIndex do nó chamado
  strcat(call, subindex);
  strcat(call, ".");

  //Valor a ser passado de até 8 digitos em HEX
  sprintf(valc, "%X", valor);
  val1 = strlen(valc);

  while (i<8){
    while (j<val1){
      valc[8-val1+j]= valc[j];
      j++;
      i++;
    }
    while(8-i > 0){
      valc[7-i] = 48;
      i++;
    }
  }

  i=0;
  j=0;

  valc1[0] = valc[6];
  valc1[1] = valc[7];
  strcat(call1, valc1);
  strcat(call1, ".");

  valc2[0] = valc[4];
  valc2[1] = valc[5];
  strcat(call1, valc2);
  strcat(call1, ".");

  valc3[0] = valc[2];
  valc3[1] = valc[3];
  strcat(call1, valc3);
  strcat(call1, ".");

  valc4[0] = valc[0];
  valc4[1] = valc[1];
  strcat(call1, valc4);

  strcat(call,call1);
  system(call);
}

void * iniciate (void *apelido) {
  int i=0;
  while(flag){
    if (i<1){
      system("./config.sh");
      i++;
    }
  }
	pthread_exit(NULL);
}

void * send (void *apelido) {
	prtcl("can0", 2, "write", "1017", "00", 0);
	pthread_exit(NULL);
}

void * minha_thread_2(void *apelido) {
	sleep(1);
  int i =0;
	while (i<30) {
		printf("	%d\n",i);
		sleep(2);
    i++;
	}
  flag = 0;
	pthread_exit(NULL);
}

void * minha_thread_3(void *apelido) {
	sleep(1);
	while (flag) {
		printf("---\n");
		sleep(2);
	}
	pthread_exit(NULL);
}

int main(int argc, char *argv[]) {

  pthread_create(&id1, NULL , (void *) iniciate, NULL);
  sleep(10);

	pthread_create(&id2,NULL , (void *) minha_thread_2,NULL);

	pthread_create(&id3,NULL , (void *) minha_thread_3,NULL);

	while (flag);

  system("./close.sh");

	return 0;
}
