#include <stdio.h>

int main()
{
	int rc = -1;

	if (rc)
		printf("okay\n");
	else
		printf("disable\n");

	rc = 0;

	if (rc)
		printf("okay\n");
	else
		printf("disable\n");

	rc = 1;

	if (rc)
		printf("okay\n");
	else
		printf("disable\n");

	if (-1)
		printf("okay\n");
	else
		printf("disable\n");
	return 0;
}
