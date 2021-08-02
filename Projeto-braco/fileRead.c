#include <stdio.h>

#define BUF_SIZE  (4096)

int main(void) {
   int i;
   const char* fileName = "logfile.log";
   char buf[BUF_SIZE] = { 0 };
   int bytesRead = 0;
   FILE* fp;               /* handle for the input file */
   size_t fileSize;        /* size of the input file */
   int lastXBytes = 48;  /* number of bytes at the end-of-file to read */

   /* open file as a binary file in read-only mode */
   if ((fp = fopen("./logfile.log", "rb")) == NULL) {
      printf("Could not open input file; Aborting\n");
      return 1;
   }

   /* find out the size of the file; reset pointer to beginning of file */
   fseek(fp, 0L, SEEK_END);
   fileSize = ftell(fp);
   fseek(fp, 0L, SEEK_SET);

   /* make sure the file is big enough to read lastXBytes of data */
   if (fileSize < lastXBytes) {
      printf("File too small; Aborting\n");
      fclose(fp);
      return 1;
   } else {
      /* read lastXBytes of file */
      fseek(fp, -lastXBytes, SEEK_END);
      bytesRead = fread(buf, sizeof(char), lastXBytes, fp);
      printf("Read %d bytes from %s, expected %d\n", bytesRead, fileName, lastXBytes);
      if (bytesRead > 0) {
         for (i=0; i<bytesRead; i++) {
            printf("%c", buf[i]);
         }
      }
   }

   fclose(fp);
   return 0;
}