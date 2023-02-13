#include <stdio.h>

int main()
{
    char buf[128];
    FILE *fp = fopen("/dev/test_misc", "rw");
    fread(buf, sizeof(buf), 1, fp);
    printf("%s\n", buf);
    fclose(fp);
    printf("Hello world\n");
    return 0;
}

