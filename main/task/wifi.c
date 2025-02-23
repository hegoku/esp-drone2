#include "platform/drivers/wifi.h"
#include "task/task.h"

static unsigned int a = 0;
void wifi_task()
{
	if (a%25==0) {
		wifi_flush();
	}
	a++;
}